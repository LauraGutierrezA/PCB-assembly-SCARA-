"""
wiimote_control.py - Logica del control con Wiimote (sin ROS, sin Qt, sin NiceGUI).

Es matematica pura, igual que robot_model.py: se puede probar en cualquier
PC con datos simulados (ver test/test_wiimote_control.py).

Entradas en cada paso (update):
  dt        segundos desde el paso anterior
  buttons   conjunto de botones presionados ahora (nombres de BUTTON_NAMES)
  gyro      {'x','y','z'} velocidad angular del Wiimote en grados/s (ya sin sesgo)

Botones:
  1      modo cinematica INVERSA (IK)
  2      modo cinematica DIRECTA (DK)
  B      dead-man: el robot solo se mueve mientras B esta presionado
  HOME   paro de emergencia
  A      sin funcion por ahora

Modo IK:  D-pad  -> X / Y  (Arriba = +X; +Y es la DERECHA si PLAN_Y_RIGHT, la izquierda si no:
          siempre el mismo lado que muestra el gemelo)
          + / -  -> z sube / baja
Modo DK:  el giro del Wiimote se integra a un angulo RELATIVO (se pone en 0 cada
          vez que se presiona B). Pasada la zona muerta (dead_angle_deg) la
          articulacion se mueve con una velocidad proporcional al exceso, hasta
          el maximo en full_angle_deg. Volver al centro = detenerse (estilo joystick).
          Un giro por debajo de rate_gate_dps no se integra: el ruido y el sesgo
          residual del giroscopio no mueven nada.

Un solo estado interno (th1, th2, z) para los dos modos: al cambiar de modo no
hay saltos. En IK el codo queda fijo con el signo de th2 al entrar al modo.

Seguridad:
  - Al presionar B la consigna arranca en la posicion MEDIDA (si hay homing).
  - La consigna nunca se aleja de la posicion medida mas de lead_deg / lead_mm.
  - Cambiar de modo o HOME exigen soltar y volver a presionar B.
"""
import math
from dataclasses import dataclass, field

from scara_app.robot_model import (
    JOINT_HOME, JOINT_LIMITS, PLAN_Y_RIGHT, codo_for, fk_xy, ik_xy, in_limits,
)

MODE_IK = 'IK'
MODE_DK = 'DK'

# Orden de sensor_msgs/Joy.buttons en /wiimote/joy (lo publica wiimote_node.py)
BUTTON_NAMES = ('UP', 'DOWN', 'LEFT', 'RIGHT', 'PLUS', 'MINUS',
                'A', 'B', 'ONE', 'TWO', 'HOME')

JOINTS = ('th1', 'th2', 'z')

# Ejes del giroscopio (x, y, z de /wiimote/imu) -> articulacion, con signo.
# PROVISIONAL: se verifica girando el Wiimote y viendo que barra se mueve.
DEFAULT_GYRO_AXES = {
    'th1': ('x', 1.0),
    'th2': ('y', 1.0),
    'z':   ('z', 1.0),
}


@dataclass
class WiimoteParams:
    dead_angle_deg: float = 25.0     # zona muerta del angulo relativo
    full_angle_deg: float = 70.0     # angulo de velocidad maxima
    speed_th_dps: float = 15.0       # velocidad maxima de th1 y th2 [grados/s]
    speed_z_mmps: float = 10.0       # velocidad maxima de z [mm/s] (DK y botones +/-)
    speed_xy_mmps: float = 20.0      # velocidad del D-pad en X / Y [mm/s]
    rate_gate_dps: float = 2.0       # giros mas lentos que esto no se integran
    lead_deg: float = 10.0           # cuanto puede adelantarse la consigna al robot real
    lead_mm: float = 10.0
    ramp_s: float = 0.5              # segundos hasta velocidad completa en IK (suavizado)
    send_period_ms: float = 80.0     # cada cuanto se manda una consigna nueva al ESP32
    gyro_axes: dict = field(default_factory=lambda: dict(DEFAULT_GYRO_AXES))

    def validate(self):
        """Corrige valores imposibles (se llama despues de editar desde la interfaz)."""
        self.dead_angle_deg = max(1.0, self.dead_angle_deg)
        self.full_angle_deg = max(self.dead_angle_deg + 1.0, self.full_angle_deg)
        for name in ('speed_th_dps', 'speed_z_mmps', 'speed_xy_mmps',
                     'rate_gate_dps', 'lead_deg', 'lead_mm', 'ramp_s'):
            setattr(self, name, max(0.0, getattr(self, name)))
        self.send_period_ms = _clamp(self.send_period_ms, 40.0, 500.0)


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def _sign(v):
    return 1.0 if v > 0 else -1.0 if v < 0 else 0.0


class WiimoteController:
    def __init__(self, d1, d2, params=None):
        self.d1, self.d2 = d1, d2
        self.params = params or WiimoteParams()
        self.mode = MODE_IK
        self.q = dict(JOINT_HOME)            # consigna actual {'th1','th2','z'}
        self.codo = codo_for(self.q['th2'])  # codo fijo en modo IK
        self.actual = None                   # posicion medida o None (sin homing)
        self.held = False                    # B presionado y permitido
        self.blocked = False                 # se pidio un movimiento imposible (limite)
        self.angles = {k: 0.0 for k in JOINTS}   # angulo relativo integrado (DK)
        self.events = []                     # [(tipo, texto)] pendientes de leer
        self.simulate = False                # True: sin correa ni resincronizar con el robot
        self.engage_q = dict(self.q)         # consigna en el instante de presionar B
        self._prev = frozenset()
        self._need_repress = False
        self._vel = [0.0, 0.0, 0.0]          # velocidad suavizada del D-pad (vx, vy, vz) en -1..1

    # ------------------------------------------------------------------
    def xy(self):
        return fk_xy(self.q['th1'], self.q['th2'], self.d1, self.d2)

    def set_actual(self, q):
        """q: {'th1','th2','z'} medido, o None si no hay homing o conexion."""
        self.actual = dict(q) if q is not None else None

    def set_mode(self, mode):
        if mode == self.mode:
            return
        self.mode = mode
        if mode == MODE_IK:
            self.codo = codo_for(self.q['th2'])
        self.angles = {k: 0.0 for k in JOINTS}
        self._release('mode')
        self._need_repress = True
        self.events.append(('mode', mode))

    def reset(self, buttons=frozenset()):
        """Al activar el control: B debe soltarse y volver a presionarse."""
        self.held = False
        self.blocked = False
        self.angles = {k: 0.0 for k in JOINTS}
        self._prev = frozenset(buttons)
        self._need_repress = 'B' in buttons

    # ------------------------------------------------------------------
    def update(self, dt, buttons, gyro):
        """Un paso. Devuelve True si la consigna cambio."""
        buttons = frozenset(buttons)
        rising = buttons - self._prev
        self._prev = buttons

        if 'HOME' in rising:
            self._release('home')
            self._need_repress = True
            self.events.append(('stop', 'HOME'))
            return False
        if 'ONE' in rising:
            self.set_mode(MODE_IK)
        elif 'TWO' in rising:
            self.set_mode(MODE_DK)

        if 'B' not in buttons:
            self._need_repress = False
            if self.held:
                self._release('b')
            self.idle_sync()
            return False
        if self._need_repress:
            self.idle_sync()
            return False
        if not self.held:
            self._engage()

        before = dict(self.q)
        if self.mode == MODE_IK:
            self._step_ik(dt, buttons)
        else:
            self._step_dk(dt, gyro)
        self._apply_lead()
        return any(abs(self.q[k] - before[k]) > 1e-9 for k in JOINTS)

    # ------------------------------------------------------------------
    def _engage(self):
        self._vel = [0.0, 0.0, 0.0]
        self.held = True
        self.blocked = False
        self.angles = {k: 0.0 for k in JOINTS}
        self.idle_sync()
        self.engage_q = dict(self.q)
        self.events.append(('engage', ''))

    def idle_sync(self, force=False):
        """Sin B presionado la consigna sigue a la posicion medida (asi al presionar B no
        hay saltos y el gemelo no muestra una consigna vieja). En simulacion no, salvo
        force=True. Solo se llama con B suelto (o al presionarlo, antes de mover)."""
        if self.actual is None or (self.simulate and not force):
            return
        self.q = {k: _clamp(self.actual[k], *JOINT_LIMITS[k]) for k in JOINTS}
        if self.mode == MODE_IK:
            self.codo = codo_for(self.q['th2'])

    def _release(self, why):
        self._vel = [0.0, 0.0, 0.0]
        if self.held:
            self.events.append(('release', why))
        self.held = False
        self.blocked = False

    def _set_blocked(self, blocked):
        if blocked and not self.blocked:
            self.events.append(('blocked', ''))
        self.blocked = blocked

    # ---------------- Modo IK ----------------
    def _smooth_input(self, dt, target):
        """Lleva la velocidad pedida por el D-pad hacia el valor objetivo con una rampa
        (aceleracion limitada): arranca y frena suave en vez de dar saltos."""
        ramp = self.params.ramp_s
        step = 1.0 if ramp <= 1e-6 else dt / ramp
        for i in range(3):
            d = target[i] - self._vel[i]
            self._vel[i] += max(-step, min(step, d))

    def _step_ik(self, dt, buttons):
        p = self.params
        want = (('UP' in buttons) - ('DOWN' in buttons),
                (('RIGHT' in buttons) - ('LEFT' in buttons)) if PLAN_Y_RIGHT
                else (('LEFT' in buttons) - ('RIGHT' in buttons)),   # +Y como en el gemelo
                ('PLUS' in buttons) - ('MINUS' in buttons))
        norm = math.hypot(want[0], want[1])
        if norm > 1.0:                                           # diagonal: misma velocidad
            want = (want[0] / norm, want[1] / norm, want[2])
        self._smooth_input(dt, want)
        vx, vy, vz = self._vel
        blocked = False

        if math.hypot(vx, vy) > 1e-3:
            dx, dy = vx * p.speed_xy_mmps * dt, vy * p.speed_xy_mmps * dt
            x, y = self.xy()
            sol = self._ik_solution(x + dx, y + dy)
            if sol is None and dx and dy:        # deslizar por un solo eje
                sol = self._ik_solution(x + dx, y) or self._ik_solution(x, y + dy)
            if sol is None:
                blocked = True
                self._vel[0] = self._vel[1] = 0.0
            else:
                self._move_joints(sol, dt)
        if abs(vz) > 1e-3:
            lo, hi = JOINT_LIMITS['z']
            new_z = _clamp(self.q['z'] + vz * p.speed_z_mmps * dt, lo, hi)
            blocked = blocked or new_z == self.q['z']
            if new_z == self.q['z']:
                self._vel[2] = 0.0
            self.q['z'] = new_z
        self._set_blocked(blocked)

    def _ik_solution(self, x, y):
        q = ik_xy(x, y, self.codo, self.d1, self.d2)
        if q is None:
            return None
        cand = {'th1': q[0], 'th2': q[1], 'z': self.q['z']}
        return q if in_limits(cand) else None

    def _move_joints(self, sol, dt):
        """Va hacia (th1, th2) sin pasar el tope de velocidad de las articulaciones
        (cerca de la singularidad un paso pequeno en XY pide un salto enorme)."""
        d1, d2 = sol[0] - self.q['th1'], sol[1] - self.q['th2']
        cap = self.params.speed_th_dps * dt
        big = max(abs(d1), abs(d2))
        scale = 1.0 if big <= cap or big == 0 else cap / big
        self.q['th1'] += d1 * scale
        self.q['th2'] += d2 * scale

    # ---------------- Modo DK ----------------
    def _step_dk(self, dt, gyro):
        p = self.params
        blocked = False
        for joint in JOINTS:
            axis, sign = p.gyro_axes[joint]
            rate = sign * gyro.get(axis, 0.0)
            if abs(rate) < p.rate_gate_dps:
                rate = 0.0
            ang = _clamp(self.angles[joint] + rate * dt, -p.full_angle_deg, p.full_angle_deg)
            self.angles[joint] = ang

            span = p.full_angle_deg - p.dead_angle_deg
            mag = _clamp((abs(ang) - p.dead_angle_deg) / span, 0.0, 1.0)
            if mag == 0.0:
                continue
            speed = p.speed_z_mmps if joint == 'z' else p.speed_th_dps
            lo, hi = JOINT_LIMITS[joint]
            new = _clamp(self.q[joint] + _sign(ang) * mag * speed * dt, lo, hi)
            if new == self.q[joint]:
                blocked = True          # esta contra el limite de la articulacion
            self.q[joint] = new
        self._set_blocked(blocked)

    # ---------------- Correa (lead) ----------------
    def _apply_lead(self):
        if self.actual is None or self.simulate:
            return
        for joint in JOINTS:
            lead = self.params.lead_mm if joint == 'z' else self.params.lead_deg
            self.q[joint] = _clamp(self.q[joint],
                                   self.actual[joint] - lead, self.actual[joint] + lead)

    # ---------------- Lectura para la interfaz ----------------
    def pop_events(self):
        ev, self.events = self.events, []
        return ev
