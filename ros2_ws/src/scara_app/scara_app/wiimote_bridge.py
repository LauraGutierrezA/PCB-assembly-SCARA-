"""
wiimote_bridge.py - Pega entre el Wiimote (ROS) y el robot. Sin Qt ni NiceGUI.

Hay UNA instancia por nodo de interfaz (gui/ros_node.py y web/ros_node.py la
crean). Asi la logica del Wiimote vive en un solo lugar y las pestanas (Qt y
web) solo muestran el estado y cambian los parametros; si hay varias pantallas
abiertas (PC + iPad + celular) no se mandan consignas duplicadas.

Entra:   on_joy / on_imu   lo que publica wiimote_node.py (/wiimote/joy, /wiimote/imu)
         on_limits         posicion medida (/scara/limits)
         on_ik_result      respuesta del firmware a cada consigna (/scara/ik)
         tick()            cada ~20 ms (un timer de ROS)
Sale:    send_joints(th1, th2, z)   consigna (como la pestana DK)
         deadman_beat() / deadman_release()   latido "dm1" / "dm0"
         emergency_stop()           HOME -> "stop"
         log(texto, nivel)          mensajes para la consola de la pestana
         on_state(dict)             estado para pintar la pestana (~15 Hz)

Reglas de seguridad (ademas de las de wiimote_control.py):
  - Desactivado por defecto. Hay que activarlo en la pestana, y se desactiva
    solo al salir de la pestana o con STOP / Esc / HOME.
  - Modo simulacion (por defecto): calcula todo y lo muestra en el gemelo,
    pero NO manda consignas ni dead-man.
  - Solo se mueve el robot con homing hecho (hay posicion medida).
  - Si dejan de llegar mensajes de /wiimote/joy por mas de LINK_TIMEOUT_S
    (Bluetooth caido, nodo cerrado, pilas) se suelta el dead-man.
  - El latido del dead-man (dm1) sale cada BEAT_PERIOD_S solo mientras B esta
    presionado; si este programa se congela, el firmware para a los 300 ms.
"""
import math
import threading
import time

from scara_app.wiimote_control import (
    BUTTON_NAMES, JOINTS, MODE_IK, WiimoteController, WiimoteParams,
)

BEAT_PERIOD_S = 0.1      # latido del dead-man (el firmware espera < 300 ms)
SEND_EPS = 0.05          # grados / mm: no se reenvia si la consigna casi no cambio
LINK_TIMEOUT_S = 0.4     # sin /wiimote/joy por mas de esto = Wiimote perdido
STATE_PERIOD_S = 0.066   # ~15 Hz hacia la interfaz
MAX_DT_S = 0.1           # un tick mas lento que esto se trata como 0.1 s

# Mismos textos que gui/point_to_point_tab.py (REASONS), resumidos.
REASONS = {
    'OUT_OF_REACH': 'fuera del alcance en XY',
    'BAD_PARAMS':   'parametros invalidos',
    'BAD_MESSAGE':  'mensaje con formato incorrecto',
    'OUT_OF_RANGE': 'fuera del rango de la articulacion',
}


def real_pose(d):
    """dict de /scara/limits -> {'th1','th2','z'} o None si no hay homing."""
    if d and d.get('CAL') and all(f'Q{i}' in d for i in (1, 2, 3)):
        return {'th1': d['Q1'], 'th2': d['Q2'], 'z': d['Q3']}
    return None


class WiimoteBridge:
    def __init__(self, robot_params, send_joints, deadman_beat, deadman_release,
                 emergency_stop, log=None, on_state=None, calibrate=None,
                 clock=time.monotonic):
        """robot_params: {'d1', 'd2', ...} en mm (los mismos de la pestana DK)."""
        self._lock = threading.RLock()
        self._clock = clock
        self._send_joints = send_joints
        self._beat = deadman_beat
        self._release = deadman_release
        self._estop = emergency_stop
        log = log or (lambda text, level: None)
        self._log = lambda text, level='info': log(text, level)   # siempre (texto, nivel)
        self._on_state = on_state or (lambda state: None)
        self._calibrate = calibrate         # pide a wiimote_node medir de nuevo el sesgo

        self.ctrl = WiimoteController(robot_params['d1'], robot_params['d2'], WiimoteParams())
        self.enabled = False
        self.dry_run = True

        self._buttons = frozenset()
        self._gyro = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self._last_joy = None
        self._last_imu = None
        self._link = False
        self._moving = False         # estamos mandando consignas y latidos
        self._last_tick = None
        self._last_beat = 0.0
        self._last_send = 0.0
        self._last_sent = None
        self._last_state = 0.0
        self._awaiting = 0.0         # hora limite para una respuesta /scara/ik
        self._warned_homing = False

    # ---------------- Entradas ----------------
    def on_joy(self, names):
        """names: nombres de los botones presionados (BUTTON_NAMES)."""
        with self._lock:
            self._buttons = frozenset(n for n in names if n in BUTTON_NAMES)
            self._last_joy = self._clock()

    def on_imu(self, wx_rad, wy_rad, wz_rad):
        with self._lock:
            self._gyro = {'x': math.degrees(wx_rad), 'y': math.degrees(wy_rad),
                          'z': math.degrees(wz_rad)}
            self._last_imu = self._clock()

    def on_limits(self, d):
        with self._lock:
            self.ctrl.set_actual(real_pose(d))

    def on_ik_result(self, r):
        with self._lock:
            if self._clock() > self._awaiting:
                return                      # respuesta a otra pestana
            self._awaiting = 0.0
            if r.get('ok'):
                return
            reason = REASONS.get(r.get('reason', ''), r.get('reason', 'error'))
            if 'joint' in r:
                reason += f" [articulacion {int(r['joint'])}]"
            self._log(f'Consigna rechazada: {reason}', 'error')

    # ---------------- Controles de la interfaz ----------------
    def set_enabled(self, on):
        with self._lock:
            on = bool(on)
            if on == self.enabled:
                return
            self.enabled = on
            if on:
                self.ctrl.reset(self._buttons)
                self._warned_homing = False
                self._log('Control con Wiimote activado'
                          + (' (simulacion: no mueve el robot)' if self.dry_run else ''))
            else:
                self._stop_motion()
                self.ctrl.reset()
                self.dry_run = True          # al volver a activar siempre empieza en simulacion
                self.ctrl.simulate = True
                self._log('Control con Wiimote desactivado')
            self._emit_state(force=True)

    def set_dry_run(self, on):
        with self._lock:
            on = bool(on)
            if on == self.dry_run:
                return
            self.dry_run = on
            self._stop_motion()
            self.ctrl.reset(self._buttons)
            self.ctrl.simulate = on
            self.ctrl.idle_sync(force=True)   # la simulacion arranca en la posicion real
            self._log('Modo simulacion: no se manda nada al robot' if on else
                      'Modo REAL: el robot se mueve mientras mantengas B')
            self._emit_state(force=True)

    def update_params(self, **kw):
        """Cambia parametros de WiimoteParams (dead_angle_deg, ..., gyro_axes)."""
        with self._lock:
            p = self.ctrl.params
            for key, value in kw.items():
                if not hasattr(p, key):
                    raise AttributeError(key)
                setattr(p, key, value)
            p.validate()
            self._emit_state(force=True)

    def calibrate_gyro(self):
        """Pide a wiimote_node recalibrar el giroscopio (el Wiimote debe estar quieto)."""
        with self._lock:
            if self.ctrl.held:
                self._log('Suelta B para recalibrar el giroscopio', 'warn')
            elif self._calibrate is not None:
                self._calibrate()
                self._log('Recalibrando el giroscopio: deja el Wiimote quieto')

    def go_to_actual(self):
        """Pone la consigna en la posicion medida (para la simulacion)."""
        with self._lock:
            if self.ctrl.actual is not None and not self.ctrl.held:
                self.ctrl.q = dict(self.ctrl.actual)
                self._emit_state(force=True)

    # ---------------- Ciclo ----------------
    def tick(self):
        with self._lock:
            now = self._clock()
            dt = 0.0 if self._last_tick is None else min(max(now - self._last_tick, 0.0), MAX_DT_S)
            self._last_tick = now

            link = self._last_joy is not None and (now - self._last_joy) < LINK_TIMEOUT_S
            if link != self._link:
                self._link = link
                self._log('Wiimote conectado' if link else
                          'Wiimote sin senal: dead-man soltado', 'info' if link else 'error')
            imu_ok = self._last_imu is not None and (now - self._last_imu) < LINK_TIMEOUT_S
            gyro = self._gyro if imu_ok else {'x': 0.0, 'y': 0.0, 'z': 0.0}

            self.ctrl.simulate = self.dry_run
            if self.enabled:
                buttons = self._buttons if link else frozenset()
                self.ctrl.update(dt, buttons, gyro)
                self._handle_events()
                self._drive(now)
            else:
                self.ctrl.idle_sync(force=True)   # el gemelo muestra la posicion real
            self._emit_state()

    def _handle_events(self):
        for kind, text in self.ctrl.pop_events():
            if kind == 'mode':
                name = 'cinematica inversa (IK)' if text == MODE_IK else 'cinematica directa (DK)'
                self._log(f'Modo: {name}. Suelta y presiona B para mover')
            elif kind == 'engage':
                self._log('B presionado (simulacion)' if self.dry_run
                          else 'B presionado: moviendo')
            elif kind == 'blocked':
                self._log('Limite: no se puede seguir en esa direccion', 'warn')
            elif kind == 'stop':
                self._log('HOME: paro de emergencia', 'error')
                self._stop_motion()
                self._estop()
                self.set_enabled(False)      # idempotente: el nodo ya pudo desactivarlo

    def _drive(self, now):
        """Manda consignas y latidos segun el estado del controlador."""
        if not self.enabled:
            return
        want = self.ctrl.held and not self.dry_run
        if want and self.ctrl.actual is None:
            if not self._warned_homing:
                self._warned_homing = True
                self._log('Sin posicion medida: haz el homing primero', 'error')
            want = False
        if not want:
            self._stop_motion()
            return

        q = self.ctrl.q
        if not self._moving:
            self._moving = True
            self._send(now, self.ctrl.engage_q)   # primero la consigna (= posicion medida)
            self._beat()
            self._last_beat = now
            return
        if now - self._last_beat >= BEAT_PERIOD_S:
            self._beat()
            self._last_beat = now
        if now - self._last_send >= self.ctrl.params.send_period_ms / 1000.0 and self._changed(q):
            self._send(now, q)

    def _changed(self, q):
        if self._last_sent is None:
            return True
        return any(abs(q[k] - self._last_sent[k]) > SEND_EPS for k in JOINTS)

    def _send(self, now, q):
        self._last_send = now
        self._last_sent = dict(q)
        self._awaiting = now + 1.5
        self._send_joints(q['th1'], q['th2'], q['z'])

    def _stop_motion(self):
        if self._moving:
            self._moving = False
            self._last_sent = None
            self._release()

    # ---------------- Estado para la interfaz ----------------
    def snapshot(self):
        with self._lock:
            c = self.ctrl
            x, y = c.xy()
            return {
                'enabled': self.enabled,
                'dry_run': self.dry_run,
                'link': self._link,
                'mode': c.mode,
                'held': c.held,
                'moving': self._moving,
                'blocked': c.blocked,
                'homed': c.actual is not None,
                'q': dict(c.q),
                'xy': (x, y),
                'codo': c.codo,
                'actual': dict(c.actual) if c.actual is not None else None,
                'gyro': dict(self._gyro),
                'angles': dict(c.angles),
                'params': {
                    'dead_angle_deg': c.params.dead_angle_deg,
                    'full_angle_deg': c.params.full_angle_deg,
                    'speed_th_dps': c.params.speed_th_dps,
                    'speed_z_mmps': c.params.speed_z_mmps,
                    'speed_xy_mmps': c.params.speed_xy_mmps,
                    'rate_gate_dps': c.params.rate_gate_dps,
                    'lead_deg': c.params.lead_deg,
                    'lead_mm': c.params.lead_mm,
                    'ramp_s': c.params.ramp_s,
                    'send_period_ms': c.params.send_period_ms,
                    'gyro_axes': {k: tuple(v) for k, v in c.params.gyro_axes.items()},
                },
            }

    def _emit_state(self, force=False):
        now = self._clock()
        if not force and now - self._last_state < STATE_PERIOD_S:
            return
        self._last_state = now
        self._on_state(self.snapshot())
