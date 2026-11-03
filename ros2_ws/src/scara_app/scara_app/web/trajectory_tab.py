"""
trajectory_tab.py - Pestana "Trayectoria" (web): ciclo pick & place con bloques.

  ┌─────────────────────────────────────────────────────────────────────────┐
  │ [GO TO POSE] [PICK] [PLACE] [ROTATE]   Despeje Z  Repet.  [Iniciar] [Detener]
  ├───────────────────────────────────────────────┬─────────────────────────┤
  │ Sandbox: bloques conectados con flechas       │ Estado del ciclo        │
  │  ┌GO TO┐→┌PICK┐→┌GO TO┐→┌PLACE┐ ...            │ Gemelo (consigna/real)  │
  │                                               │ [ Mantener para mover ] │
  │ Consola                                       │                         │
  └───────────────────────────────────────────────┴─────────────────────────┘

Flujo:
  1. "Iniciar trayectoria" pide el diagrama al navegador, lo ordena y lo
     valida COMPLETO con la misma IK del firmware (scara_app/trajectory.py).
     Si algo no se puede (fuera de alcance, fuera de rango, flechas mal), el
     bloque se marca en rojo y no se mueve nada.
  2. El ciclo queda ARMADO: se mueve SOLO mientras se mantiene el dead-man
     (boton o barra espaciadora). Al soltar se PAUSA (el tiempo del perfil
     se congela y el firmware frena); al volver a mantener sigue en el
     mismo punto del mismo bloque.
  3. GO TO POSE: cada tramo (subir / viaje XY / bajar) se manda como UNA
     meta con su duracion (send_goal, 13 valores en /scara/target) y el
     ESP32 interpola cada 8 ms (firmware/components/control/profile.c).
     Puntos intermedios: cuando /scara/limits dice PG >= BLEND_PCT (avance
     del tramo) se manda el siguiente y el ESP32 los superpone, sin frenar.
     Meta final: se espera PG = 100 y luego ARR = 1 (llegada estricta).
     Si el ESP32 no confirma un tramo (GS = id), se reenvia con el mismo id.
     PICK/PLACE: "vac1"/"vac0", confirmacion de VAC y espera.
     ROTATE: "rot +90"/"rot -90" + espera.
  4. STOP (encabezado), Esc, "Detener", cambiar de pestana, perder el ESP32
     o un rechazo de la IK del firmware -> el ciclo se cancela.

Los parametros del robot (d1..av) son los de Punto a punto por defecto
(ROBOT_DEFAULTS de gui/point_to_point_tab.py).
"""
import json
import random
import time
from dataclasses import dataclass

from nicegui import ui

from scara_app.gui import theme as t
from scara_app.gui.point_to_point_tab import REASONS, ROBOT_DEFAULTS
from scara_app.robot_model import ik_xy, in_limits
from scara_app.trajectory import (
    KIND_LABEL, TUNING_RANGES, Z_CLEAR_DEFAULT, describe_error, Z_CLEAR_MAX, Arrive, BlockStart, Command, Dwell,
    Goal, PlanError, Pose, Tuning, VacConfirm, cartesian, plan_cycle,
)

from . import trajectory_sandbox as sandbox
from .dk_tab import real_pose
from .twin import Twin
from .widgets import LogView, button, caps_label, deadman_button, number_box, read_number

ROBOT = dict(ROBOT_DEFAULTS)

TICK_S = 0.05             # revision del ciclo (el perfil lo genera el ESP32)
MAX_DT_S = 0.2            # si la pagina se atrasa, no saltar mas que esto
ARRIVE_GRACE_S = 0.25     # tras el ultimo punto, ignorar ARR viejos
GOAL_RESEND_S = 0.6       # sin GS = id en este tiempo (con dead-man) -> reenviar el tramo
GOAL_MAX_RETRIES = 4
SETTLE_WARN_S = 8.0       # sin ARR=1 en este tiempo (con dead-man): AVISO en la consola y se
                          # sigue esperando (el RST sigue corrigiendo). Se repite cada 8 s.
TOL_DEG, TOL_MM = 1.0, 1.0   # si el firmware no manda ARR: llegada por encoders
VAC_RETRY_S = 0.4         # sin confirmacion de VAC en este tiempo -> reenviar vac0/vac1
VAC_MAX_RETRIES = 3       # despues de esto se cancela el ciclo

FF_RETRY_S = 0.4          # sin confirmacion de FF en /scara/limits -> reenviar "ff"
FF_MAX_RETRIES = 3

# Casillas de ajuste: (clave en TUNING_RANGES, etiqueta, formato, paso, atributo de Tuning)
TUNING_FIELDS = [
    ('V_MAX_DEG',     'V_MAX_DEG [°/s]',   '%.0f', 5,    'v_max_deg'),
    ('A_MAX_DEG',     'A_MAX_DEG [°/s²]',  '%.0f', 10,   'a_max_deg'),
    ('V_MAX_Z',       'V_MAX_Z [mm/s]',    '%.2f', 0.1,  'v_max_z'),
    ('A_MAX_Z',       'A_MAX_Z [mm/s²]',   '%.1f', 0.5,  'a_max_z'),
    ('FF_GAIN',       'FF_GAIN',           '%.2f', 0.05, 'ff_gain'),
    ('BLEND_PCT',     'BLEND_PCT [%]',     '%.0f', 5,    'blend_pct'),
    ('MIN_SEGMENT_S', 'MIN_SEGMENT_S [s]', '%.2f', 0.05, 'min_segment_s'),
]


@dataclass
class FfSet:
    """Primer paso del ciclo: manda FF_GAIN al ESP32 y espera FF en /scara/limits."""
    value: float


STATE_STYLE = {
    'idle':    ('LISTO', t.TEXT_MUTED),
    'armed':   ('ARMADO', t.CONNECTED),
    'running': ('CORRIENDO', t.ACCENT),
    'paused':  ('PAUSADO', t.CONNECTED),
    'done':    ('TERMINADO', t.ACCENT),
    'error':   ('CANCELADO', t.DANGER),
}


class TrajectoryTab:
    def __init__(self, node, hub):
        self.node = node
        self.state = 'idle'
        self.held = False
        self.plan = None
        self.i = 0                 # paso actual
        self.step_t = 0.0          # tiempo (con dead-man) dentro del paso
        self.step_wall = 0.0       # hora en que empezo el paso (para ARR viejos)
        self.last_tick = None
        self.limits = {}
        self.limits_time = 0.0
        self.active_block = None
        self.block_info = ''
        self.repeats = 1
        self.retries = 0
        self.goal_id = None            # id del tramo en curso (None: aun no se manda)
        self.tuning = Tuning()
        self._last_wait = ''
        self.goal_sent_at = 0.0
        self._id_counter = random.randint(1, 4000)   # ids distintos tras recargar la pagina

        ui.add_head_html(sandbox.head_html())

        with ui.element('div').classes('tab-body traj'):
            # ---------- Barra superior: bloques + iniciar ----------
            with ui.element('div').classes('traj-toolbar'):
                with ui.element('div').classes('traj-palette'):
                    caps_label('Bloques  ·  toca para agregar al final, o arrastra')
                    with ui.element('div').classes('traj-palette-row'):
                        for kind, label in KIND_LABEL.items():
                            item = ui.element('div').classes('tp-item').style(
                                f'--k:{sandbox.KIND_COLOR[kind]}')
                            item._props['data-kind'] = kind
                            with item:
                                ui.label(label)
                                ui.label('+').classes('tp-plus')
                ui.element('div').classes('grow')
                self.btn_start = button('Iniciar trayectoria', 'primary', self._start)
                self.btn_stop = button('Detener', 'danger', self._stop)

            # ---------- Ajustes del perfil (rangos en trajectory.TUNING_RANGES) ----------
            with ui.element('div').classes('traj-params'):
                self.tune = {}
                for key, label, fmt, step, _attr in TUNING_FIELDS:
                    lo, hi, default = TUNING_RANGES[key]
                    with ui.element('div').classes('field traj-field'):
                        caps_label(label)
                        box = (ui.number(value=default, format=fmt, step=step, min=lo, max=hi)
                               .props('outlined dense hide-bottom-space')
                               .classes('num num-param'))
                        box.tooltip(f'{lo:g} a {hi:g}')
                        self.tune[key] = box
                with ui.element('div').classes('field traj-field'):
                    caps_label('Despeje Z [mm]')
                    self.z_clear = number_box(Z_CLEAR_DEFAULT, 'param', minimum=0,
                                              maximum=Z_CLEAR_MAX)
                with ui.element('div').classes('field traj-field'):
                    caps_label('Repeticiones')
                    self.reps = (ui.number(value=1, format='%.0f', step=1, min=1, max=99)
                                 .props('outlined dense hide-bottom-space')
                                 .classes('num num-param'))
                button('Valores por defecto', 'text', self._tuning_defaults)

            with ui.element('div').classes('traj-split'):
                # ---------- Sandbox + consola ----------
                with ui.element('div').classes('traj-left'):
                    with ui.element('div').classes('traj-sb-head'):
                        caps_label('Sandbox  ·  arrastra desde el punto de salida → al siguiente bloque')
                        with ui.element('div').classes('btn-row'):
                            button('Ejemplo', 'text', lambda: self._js('scaraTraj.example()'))
                            button('Limpiar', 'text', lambda: self._js('scaraTraj.clearAll()'))
                    self.sb = ui.element('div').classes('traj-sb').props('id=traj-sb')
                    self.console = LogView('Consola')

                # ---------- Estado + gemelo + dead-man ----------
                with ui.element('div').classes('traj-right'):
                    with ui.element('div').classes('traj-state'):
                        self.chip = ui.label().classes('traj-chip')
                        self.progress = ui.html('', sanitize=False).classes('traj-progress')
                    self.twin = Twin(ROBOT['d1'], ROBOT['d2'])
                    deadman_button('traj')

        ui.on('traj_select', self._on_select)
        hub.on('limits', self._on_limits)
        hub.on('ik', self._on_ik)
        hub.on('connection', self._on_connection)
        hub.on('estop', lambda _p: self._abort('STOP: ciclo cancelado.', release=False))
        ui.timer(TICK_S, self._tick)
        ui.timer(0.3, lambda: self._js('scaraTraj.tryMount()'), once=True)

        self._on_limits(node.last_limits)
        self._on_connection(node.connected)
        self._render_state()

    # ================================================================
    #   Botones
    # ================================================================
    async def _start(self):
        if self.state in ('armed', 'running'):
            return
        if not self.node.connected:
            self.console.append('ESP32 sin conexion.', t.DANGER)
            return
        try:
            program = await ui.run_javascript(
                'return window.scaraTraj ? window.scaraTraj.get() : null', timeout=3.0)
        except Exception:   # noqa: BLE001  (timeout o pagina cerrada)
            program = None
        if not program:
            self.console.append('No se pudo leer el sandbox (recarga la pagina).', t.DANGER)
            return

        self._js('scaraTraj.clearMarks()')
        z_clear = read_number(self.z_clear)
        reps = read_number(self.reps)
        if z_clear is None or reps is None:
            self.console.append('Llena Despeje Z y Repeticiones.', t.DANGER)
            return
        tuning = self._read_tuning()
        if tuning is None:
            return
        q = real_pose(self.limits)
        start = Pose(q['th1'], q['th2'], q['z']) if q else None
        try:
            plan = plan_cycle(program, start, self.limits.get('Q4', 0.0), int(round(reps)),
                              z_clear, ROBOT['d1'], ROBOT['d2'], tuning)
        except PlanError as e:
            self.console.append(f'No se puede iniciar: {e}', t.DANGER)
            if e.block_id:
                self._js(f'scaraTraj.mark({json.dumps(e.block_id)}, "error")')
            self.state = 'error'
            self._render_state()
            return

        for w in plan.warnings:
            self.console.append(f'Aviso: {w}', t.CONNECTED)
        plan.steps.insert(0, FfSet(tuning.ff_gain))
        self.tuning = tuning
        self.console.append(
            f'Ajustes: V_MAX_DEG={tuning.v_max_deg:g} °/s · A_MAX_DEG={tuning.a_max_deg:g} °/s² · '
            f'V_MAX_Z={tuning.v_max_z:g} mm/s · A_MAX_Z={tuning.a_max_z:g} mm/s² · '
            f'FF_GAIN={tuning.ff_gain:.2f} · BLEND_PCT={tuning.blend_pct:g} % · '
            f'MIN_SEGMENT_S={tuning.min_segment_s:g} s · despeje {z_clear:g} mm')
        self.plan, self.i, self.step_t = plan, 0, 0.0
        self.repeats = int(round(reps))
        self.active_block, self.block_info = None, ''
        n = len(plan.order)
        self.console.append(
            f'Ciclo listo: {n} bloque(s) × {self.repeats} · ~{plan.duration:.0f} s de '
            f'movimiento y esperas. Manten el dead-man para empezar.')
        self.state = 'running' if self.held else 'armed'
        if self.held:
            self.step_wall = time.monotonic()
        self.last_tick = time.monotonic()
        self._js('scaraTraj.setLocked(true)')
        self._render_state()

    def _read_tuning(self):
        """Lee las casillas de ajuste; None (y mensaje) si alguna esta vacia o fuera de rango."""
        values = {}
        for key, _label, _fmt, _step, attr in TUNING_FIELDS:
            v = read_number(self.tune[key])
            lo, hi, _ = TUNING_RANGES[key]
            if v is None or not lo <= v <= hi:
                self.console.append(f'{key} fuera de rango: debe estar entre {lo:g} y {hi:g}.',
                                    t.DANGER)
                return None
            values[attr] = v
        return Tuning(**values)

    def _tuning_defaults(self):
        for key, *_ in TUNING_FIELDS:
            self.tune[key].value = TUNING_RANGES[key][2]

    def _stop(self):
        if self.state in ('armed', 'running'):
            self.node.emergency_stop()      # tambien llega como evento 'estop'
        else:
            self._abort('Detenido.', release=False)

    # ================================================================
    #   Dead-man (lo llama app.py)
    # ================================================================
    def on_deadman_start(self):
        if self.state == 'armed':
            self.state = 'running'
            self.step_wall = time.monotonic()
            self.console.append('Dead-man presionado: arranca el ciclo.')
        # El tramo en curso lo conserva el ESP32 (su perfil se congela al soltar)

    def set_held(self, held):
        # app.py la llama con CADA latido (cada 100 ms): solo actuar en los cambios
        if held == self.held:
            return
        self.held = held
        if self.state == 'running' and not held:
            self.console.append('Pausado (se solto el dead-man). Mantenlo para seguir.',
                                t.CONNECTED)
        self._render_state()

    def on_tab_change(self, name):
        if name != 'traj' and self.state in ('armed', 'running'):
            self._abort('Cambiaste de pestana: ciclo cancelado.', release=True)

    # ================================================================
    #   Ejecucion
    # ================================================================
    def _step(self):
        if self.plan is None or self.i >= len(self.plan.steps):
            return None
        return self.plan.steps[self.i]

    def _next(self):
        self.i += 1
        self.step_t = 0.0
        self.step_wall = time.monotonic()
        self.retries = 0
        self.goal_id = None

    def _tick(self):
        now = time.monotonic()
        dt = 0.0 if self.last_tick is None else min(MAX_DT_S, now - self.last_tick)
        self.last_tick = now
        if self.state != 'running' or not self.held:
            return
        # Pasos instantaneos (inicio de bloque, comandos) se procesan seguidos
        for _ in range(50):
            step = self._step()
            if step is None:
                self._finish()
                return
            if isinstance(step, BlockStart):
                self._block_start(step)
                self._next()
            elif isinstance(step, Command):
                if step.action == 'vac':
                    self.node.vacuum(bool(step.value))
                    self.console.append('  Bomba ' + ('ON (pick)' if step.value else 'OFF (place)'))
                else:
                    self.node.rotate(step.value)
                    self.console.append(f'  Servo {step.value:+d}° (relativo)')
                self._next()
            elif isinstance(step, Goal):
                if 'GS' not in self.limits:
                    self._abort('El firmware no tiene el perfil en el ESP32 (falta GS en '
                                '/scara/limits): flashea el firmware nuevo.', release=True)
                    return
                if self.goal_id is None:
                    self._send_goal(step, new_id=True)
                    return
                self.step_t += dt
                fresh = self.limits_time > self.goal_sent_at
                if fresh and self.limits.get('GS') == self.goal_id:
                    need = 100 if step.final else self.tuning.blend_pct
                    if self.limits.get('PG', 0) >= need:
                        self._next()
                        continue
                    return
                if self.step_t > GOAL_RESEND_S * (self.retries + 1):
                    if self.retries >= GOAL_MAX_RETRIES:
                        self._abort(f'El ESP32 no confirmo el tramo "{step.label}" '
                                    f'(GS={self.limits.get("GS")}, esperado {self.goal_id}).',
                                    block=step.block_id)
                        return
                    self.retries += 1
                    self.console.append(f'  Tramo sin confirmar: reenvio ({self.retries})',
                                        t.CONNECTED)
                    self._send_goal(step, new_id=False)
                return
            elif isinstance(step, Arrive):
                self.step_t += dt
                if self._arrived(step.target):
                    ar = self.limits.get('AR')
                    if isinstance(ar, str):
                        for m, ch in enumerate(ar, start=1):
                            if ch == '2':
                                self.console.append(
                                    f'  M{m} llego al final de carrera (al tope): '
                                    f'{describe_error(m, self.limits.get(f"EC{m}", 0))} '
                                    f'de la meta', t.CONNECTED)
                    self._next()
                    continue
                waiting = self._waiting_text()
                if waiting and waiting != self._last_wait:
                    self._last_wait = waiting
                    self.progress.set_content(f'{self.block_info}<br>'
                                              f'<span class="muted">esperando {waiting}</span>')
                if self.step_t > SETTLE_WARN_S * (self.retries + 1):
                    self.retries += 1          # aqui cuenta los avisos (se reinicia en _next)
                    self.console.append(
                        f'  Aviso: {self.step_t:.0f} s sin llegar · {waiting or "sin diagnostico"}'
                        f' · el RST sigue corrigiendo (STOP para cancelar)', t.CONNECTED)
                return
            elif isinstance(step, FfSet):
                if 'FF' not in self.limits:
                    self.console.append('  Aviso: el firmware no reporta FF; FF_GAIN no se '
                                        'cambio (flashea el firmware nuevo).', t.CONNECTED)
                    self._next()
                    continue
                want = int(round(step.value * 100))
                if self.step_t == 0.0 and self.retries == 0:
                    self.node.set_ff_gain(step.value)
                self.step_t += dt
                fresh = self.limits_time > self.step_wall + 0.1
                if fresh and self.limits['FF'] == want:
                    self.console.append(f'  FF_GAIN = {step.value:.2f} confirmado en el ESP32')
                    self._next()
                    continue
                if self.step_t > FF_RETRY_S * (self.retries + 1):
                    if self.retries >= FF_MAX_RETRIES:
                        self._abort(f'El ESP32 no confirmo FF_GAIN = {step.value:.2f} '
                                    f'(FF={self.limits.get("FF")}).')
                        return
                    self.retries += 1
                    self.node.set_ff_gain(step.value)
                return
            elif isinstance(step, VacConfirm):
                self.step_t += dt
                if 'VAC' not in self.limits:          # firmware viejo: no se puede confirmar
                    self._next()
                    continue
                fresh = self.limits_time > self.step_wall + 0.1
                if fresh and bool(self.limits['VAC']) == step.on:
                    if self.retries:
                        self.console.append(f'  Bomba confirmada {"ON" if step.on else "OFF"} '
                                            f'tras {self.retries} reintento(s)', t.CONNECTED)
                    self._next()
                    continue
                if self.step_t > VAC_RETRY_S * (self.retries + 1):
                    if self.retries >= VAC_MAX_RETRIES:
                        self._abort(f'El ESP32 no confirmo la bomba {"ON" if step.on else "OFF"} '
                                    f'(VAC={int(bool(self.limits["VAC"]))} tras '
                                    f'{VAC_MAX_RETRIES} reenvios).', block=step.block_id)
                        return
                    self.retries += 1
                    self.node.vacuum(step.on)
                    self.console.append(f'  Bomba sin confirmar: reenvio '
                                        f'{"vac1" if step.on else "vac0"} ({self.retries})',
                                        t.CONNECTED)
                return
            elif isinstance(step, Dwell):
                self.step_t += dt
                if self.step_t >= step.seconds:
                    self._next()
                    continue
                return
        self._render_state()

    def _block_start(self, s):
        if self.active_block:
            self._js(f'scaraTraj.mark({json.dumps(self.active_block)}, "done")')
        self.active_block = s.block_id
        self._js(f'scaraTraj.setActive({json.dumps(s.block_id)})')
        n = len(self.plan.order)
        self.block_info = (f'Ciclo {s.cycle}/{self.repeats} · bloque {s.index}/{n}<br>'
                           f'<span class="muted">{s.text}</span>')
        self.console.append(f'[{s.cycle}/{self.repeats}] #{s.index} {s.text}')
        self._render_state()

    def _target_xy(self):
        """(x, y) del GO TO POSE activo (cruz en el gemelo)."""
        step = self._step()
        end = getattr(step, 'end', None) or getattr(step, 'target', None)
        if end is None:
            return None
        x, y, _, _ = cartesian(end, ROBOT['d1'], ROBOT['d2'])
        return (x, y)

    def _send_goal(self, step, new_id):
        """Manda el tramo al ESP32 (meta + duracion + id); el ESP32 interpola."""
        if new_id:
            self._id_counter = self._id_counter % 9999 + 1
            self.goal_id = self._id_counter
            kind = 'meta' if step.final else 'intermedio'
            self.console.append(f'  → {step.label} ({kind}, {step.duration:.1f} s)')
        x, y, z, codo = cartesian(step.end, ROBOT['d1'], ROBOT['d2'])
        self.node.send_goal(x, y, z, ROBOT, codo, step.duration, self.goal_id)
        self.goal_sent_at = time.monotonic()
        self.twin.set_ref((step.end.th1, step.end.th2), step.end.z, target=self._target_xy())

    def _waiting_text(self):
        """Motores que aun no llegan, con su error y voltaje (campos AR/EC/U del firmware)."""
        d = self.limits
        ar = d.get('AR')
        if not isinstance(ar, str) or len(ar) != 3:
            return ''
        parts = []
        for m in (1, 2, 3):
            if ar[m - 1] == '0' and f'EC{m}' in d:
                parts.append(f'{describe_error(m, d[f"EC{m}"])}, u = {d.get(f"U{m}", 0):+.1f} V')
        return ' · '.join(parts)

    def _arrived(self, target):
        if self.step_t < ARRIVE_GRACE_S or self.limits_time < self.step_wall + ARRIVE_GRACE_S:
            return False
        d = self.limits
        if 'ARR' in d:
            return bool(d['ARR'])
        q = real_pose(d)          # firmware sin ARR: por encoders
        return q is not None and (abs(q['th1'] - target.th1) < TOL_DEG and
                                  abs(q['th2'] - target.th2) < TOL_DEG and
                                  abs(q['z'] - target.z) < TOL_MM)

    def _finish(self):
        if self.active_block:
            self._js(f'scaraTraj.mark({json.dumps(self.active_block)}, "done")')
        self._js('scaraTraj.setActive(null); scaraTraj.setLocked(false)')
        self.console.append(f'Trayectoria terminada ({self.repeats} ciclo(s)).', t.ACCENT)
        self.state = 'done'
        self.plan = None
        self.block_info = ''
        self._render_state()

    def _abort(self, reason, block=None, release=True):
        if self.state not in ('armed', 'running'):
            return
        if release:
            self.node.release_control()      # borra la consigna en el firmware
        self.node.vacuum(False)              # ciclo cancelado: la bomba no se queda prendida
        self._js('scaraTraj.setActive(null); scaraTraj.setLocked(false)')
        bad = block or self.active_block
        if bad:
            self._js(f'scaraTraj.mark({json.dumps(bad)}, "error")')
        self.console.append(reason, t.DANGER)
        self.state = 'error'
        self.plan = None
        self._render_state()

    # ================================================================
    #   Eventos del nodo
    # ================================================================
    def _on_limits(self, d):
        self.limits = d or {}
        self.limits_time = time.monotonic()
        q = real_pose(self.limits)
        self.twin.set_real(None if q is None else (q['th1'], q['th2']),
                           None if q is None else q['z'])

    def _on_ik(self, r):
        if self.state != 'running' or r.get('ok'):
            return
        reason = REASONS.get(r.get('reason', ''), r.get('reason', 'error'))
        self._abort(f'El ESP32 rechazo una consigna: {reason}')

    def _on_connection(self, connected):
        self.btn_start.set_enabled(connected)
        if not connected:
            self._abort('Se perdio la conexion con el ESP32: ciclo cancelado.', release=False)

    def _on_select(self, e):
        """Tocar un bloque GO TO POSE muestra su pose en el gemelo (si no esta corriendo)."""
        if self.state in ('armed', 'running'):
            return
        a = e.args or {}
        if a.get('kind') != 'goto':
            return
        p = a.get('params') or {}
        try:
            x, y, z = float(p['x']), float(p['y']), float(p['z'])
        except (KeyError, TypeError, ValueError):
            self.twin.set_ref(None, None)
            return
        q = ik_xy(x, y, int(p.get('codo', 1)), ROBOT['d1'], ROBOT['d2'])
        if q is not None and not in_limits({'th1': q[0], 'th2': q[1], 'z': z}):
            q = None
        self.twin.set_ref(q, z if q is not None else None, target=(x, y))

    # ================================================================
    def _render_state(self):
        st = self.state
        if st == 'running' and not self.held:
            st = 'paused'
        text, color = STATE_STYLE[st]
        self.chip.set_text(text)
        self.chip.style(f'color:{color}')
        hint = {
            'idle': 'Arma el ciclo en el sandbox y presiona Iniciar.',
            'armed': 'Manten el dead-man (boton o Espacio) para empezar.',
            'paused': 'Manten el dead-man para continuar.',
            'done': 'Ciclo completo.',
            'error': 'Revisa la consola.',
        }.get(st, '')
        body = self.block_info if st in ('running', 'paused') and self.block_info else hint
        self.progress.set_content(body)
        busy = self.state in ('armed', 'running')
        self.btn_start.set_enabled(self.node.connected and not busy)
        self.btn_stop.set_enabled(busy)

    def _js(self, code):
        ui.run_javascript(f'if (window.scaraTraj) {{ {code} }}')
