"""
trajectory_tab.py - Pestana "Trayectoria" (web): ciclo pick & place con bloques.

  ┌─────────────────────────────────────────────────────────────────────────┐
  │ [GO TO POSE] [PICK] [PLACE] [ROTATE]    Z segura  Repet.  [Iniciar] [Detener]
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
  3. GO TO POSE: el perfil quintico se muestrea cada TICK_S y cada punto
     va a /scara/target (stream_target, sin log). Al final espera ARR=1 en
     /scara/limits (los 3 motores llegaron). PICK/PLACE: "vac1"/"vac0" +
     espera. ROTATE: "rot +90"/"rot -90" + espera.
  4. STOP (encabezado), Esc, "Detener", cambiar de pestana, perder el ESP32
     o un rechazo de la IK del firmware -> el ciclo se cancela.

Los parametros del robot (d1..av) son los de Punto a punto por defecto
(ROBOT_DEFAULTS de gui/point_to_point_tab.py).
"""
import json
import time

from nicegui import ui

from scara_app.gui import theme as t
from scara_app.gui.point_to_point_tab import REASONS, ROBOT_DEFAULTS
from scara_app.robot_model import JOINT_LIMITS, ik_xy, in_limits
from scara_app.trajectory import (
    KIND_LABEL, Z_SAFE_DEFAULT, Arrive, BlockStart, Command, Dwell, Move, PlanError, Pose,
    cartesian, plan_cycle,
)

from . import trajectory_sandbox as sandbox
from .dk_tab import real_pose
from .twin import Twin
from .widgets import LogView, button, caps_label, deadman_button, number_box, read_number

ROBOT = dict(ROBOT_DEFAULTS)

TICK_S = 0.05             # muestreo del perfil -> /scara/target (20 Hz)
MAX_DT_S = 0.2            # si la pagina se atrasa, no saltar mas que esto
ARRIVE_GRACE_S = 0.25     # tras el ultimo punto, ignorar ARR viejos
SETTLE_TIMEOUT_S = 8.0    # tiempo maximo (con dead-man) para que ARR=1
TOL_DEG, TOL_MM = 1.0, 1.0   # si el firmware no manda ARR: llegada por encoders

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
                with ui.element('div').classes('field traj-field'):
                    caps_label('Z segura [mm]')
                    self.z_safe = number_box(Z_SAFE_DEFAULT, 'param',
                                             minimum=JOINT_LIMITS['z'][0],
                                             maximum=JOINT_LIMITS['z'][1])
                with ui.element('div').classes('field traj-field'):
                    caps_label('Repeticiones')
                    self.reps = (ui.number(value=1, format='%.0f', step=1, min=1, max=99)
                                 .props('outlined dense hide-bottom-space')
                                 .classes('num num-param'))
                self.btn_start = button('Iniciar trayectoria', 'primary', self._start)
                self.btn_stop = button('Detener', 'danger', self._stop)

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
        z_safe = read_number(self.z_safe)
        reps = read_number(self.reps)
        if z_safe is None or reps is None:
            self.console.append('Llena Z segura y Repeticiones.', t.DANGER)
            return
        q = real_pose(self.limits)
        start = Pose(q['th1'], q['th2'], q['z']) if q else None
        try:
            plan = plan_cycle(program, start, self.limits.get('Q4', 0.0), int(round(reps)),
                              z_safe, ROBOT['d1'], ROBOT['d2'])
        except PlanError as e:
            self.console.append(f'No se puede iniciar: {e}', t.DANGER)
            if e.block_id:
                self._js(f'scaraTraj.mark({json.dumps(e.block_id)}, "error")')
            self.state = 'error'
            self._render_state()
            return

        for w in plan.warnings:
            self.console.append(f'Aviso: {w}', t.CONNECTED)
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
        # Reenviar la consigna actual (por si el firmware la perdio)
        step = self._step()
        if self.state == 'running' and isinstance(step, Move):
            self._send_pose(step.at(self.step_t))

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
                    self.console.append(f'  Servo {step.value:+d}°')
                self._next()
            elif isinstance(step, Move):
                self.step_t += dt
                done = self.step_t >= step.duration
                pose = step.end if done else step.at(self.step_t)
                self._send_pose(pose)
                self.twin.set_ref((pose.th1, pose.th2), pose.z, target=self._target_xy())
                if done:
                    self._next()
                return
            elif isinstance(step, Arrive):
                self.step_t += dt
                if self._arrived(step.target):
                    self._next()
                    continue
                if self.step_t > SETTLE_TIMEOUT_S:
                    self._abort(f'El robot no llego a la pose en {SETTLE_TIMEOUT_S:.0f} s '
                                f'(revisa zona muerta / finales).', block=step.block_id)
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

    def _send_pose(self, pose):
        x, y, z, codo = cartesian(pose, ROBOT['d1'], ROBOT['d2'])
        self.node.stream_target(x, y, z, ROBOT, codo)

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
