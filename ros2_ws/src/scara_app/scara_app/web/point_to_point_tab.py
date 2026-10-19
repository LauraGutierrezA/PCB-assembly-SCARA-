"""
point_to_point_tab.py - Pestana de punto a punto (web) + gemelo.

Flujo igual a gui/point_to_point_tab.py: parametros + consigna (x, y, z)
-> /scara/target -> IK en el ESP32 -> /scara/ik -> consola.

Gemelo (derecha): la consigna (cian) se calcula aqui mismo con la misma IK
del firmware (robot_model.ik_xy) apenas cambian X, Y, Z o el codo, antes de
mover; si no es alcanzable, solo se ve la cruz en rojo. El robot real (gris)
sigue a los encoders. Tocar la planta pone X y Y (y cambia de codo si con el
actual no se llega).

Los valores por defecto y los mensajes de error se importan de la version
de escritorio: si cambias los parametros del robot alla, cambian aca.

Dead-man: "Enviar consigna" solo calcula la IK y fija la consigna; el robot
se mueve mientras se mantiene "Mantener para mover" (o la barra
espaciadora). Al empezar a mantener se vuelve a mandar la consigna de los
campos (app.py llama on_deadman_start).
"""
import math
import time

from nicegui import ui

from scara_app.gui import theme as t
from scara_app.gui.point_to_point_tab import (
    REASONS, RESPONSE_TIMEOUT_MS, ROBOT_DEFAULTS, TARGET_DEFAULTS,
)
from scara_app.robot_model import ik_xy, in_limits

from .dk_tab import real_pose
from .twin import Twin
from .widgets import (
    LogView, SegmentedToggle, button, caps_label, deadman_button, number_box, read_number,
)

ROBOT = dict(ROBOT_DEFAULTS)


def local_solution(x, y, z, codo):
    """IK local (igual al firmware) -> (th1, th2) si es alcanzable y esta en rango, si no None."""
    q = ik_xy(x, y, codo, ROBOT['d1'], ROBOT['d2'])
    if q is None or not in_limits({'th1': q[0], 'th2': q[1], 'z': z}):
        return None
    return q


class PointToPointTab:
    def __init__(self, node, hub):
        self.node = node
        self._deadline = None   # hora limite para la respuesta del ESP32

        with ui.element('div').classes('tab-body compact'):
            # ---------- Parametros del robot ----------
            with ui.element('div').classes('p2p-header'):
                caps_label('Parametros del robot [mm]  ·  av [mm/vuelta]')
                button('Restaurar valores', 'text', self._restore_defaults)
            self.params = {}
            with ui.element('div').classes('p2p-params').style('margin-bottom:14px'):
                for name, value in ROBOT_DEFAULTS:
                    with ui.element('div').classes('field'):
                        caps_label(name)
                        box = number_box(value, 'param')
                        box.on('keydown.enter', self._send)
                        self.params[name] = box

            with ui.element('div').classes('split'):
                with ui.element('div').classes('left'):
                    # ---------- Consigna X, Y, Z ----------
                    self.fields = {}
                    with ui.element('div').classes('coords'):
                        for axis, value in TARGET_DEFAULTS:
                            with ui.element('div').classes('field'):
                                caps_label(f'{axis} [mm]')
                                box = number_box(value, 'coord')
                                box.on('keydown.enter', self._send)
                                box.on_value_change(lambda _e: self._update_twin())
                                self.fields[axis] = box
                    # ---------- Codo + enviar ----------
                    with ui.element('div').classes('p2p-row'):
                        with ui.element('div').classes('field'):
                            caps_label('Codo')
                            self.codo = SegmentedToggle([('+1', 1), ('−1', -1)], default=0,
                                                        on_change=lambda _v: self._update_twin())
                        self.btn_send = button('Enviar consigna', 'primary', self._send)

                    # ---------- Dead-man ----------
                    deadman_button('p2p')
                    ui.element('div').style('height:12px; flex:none')

                    # ---------- Consola ----------
                    self.console = LogView('Consola')

                with ui.element('div').classes('right'):
                    self.twin = Twin(ROBOT['d1'], ROBOT['d2'], on_plan_click=self._plan_click)

        ui.timer(0.2, self._check_timeout)
        hub.on('ik', self._on_ik_result)
        hub.on('connection', self._on_connection)
        hub.on('limits', self._on_limits)
        self._on_connection(node.connected)
        self._on_limits(node.last_limits)
        self._update_twin()

    # ---------------- Gemelo ----------------
    def _update_twin(self):
        x, y, z = (read_number(self.fields[a]) for a in ('X', 'Y', 'Z'))
        if None in (x, y, z):
            self.twin.set_ref(None, None)
            return
        q = local_solution(x, y, z, self.codo.value())
        self.twin.set_ref(q, z if q is not None else None, target=(x, y))

    def _plan_click(self, x, y):
        z = read_number(self.fields['Z'], 72.6)
        codo = self.codo.value()
        if local_solution(x, y, z, codo) is None and local_solution(x, y, z, -codo) is not None:
            self.codo.set_index(0 if -codo > 0 else 1)
        self.fields['X'].value = x
        self.fields['Y'].value = y
        self._update_twin()

    def _on_limits(self, d):
        q = real_pose(d)
        if q is None:
            self.twin.set_real(None, None)
        else:
            self.twin.set_real((q['th1'], q['th2']), q['z'])

    # ------------------------------------------------------------------
    def _restore_defaults(self):
        for name, value in ROBOT_DEFAULTS:
            self.params[name].value = value

    def _send(self):
        if not self.btn_send.enabled:
            return
        values = {a: read_number(self.fields[a]) for a in ('X', 'Y', 'Z')}
        params = {name: read_number(box) for name, box in self.params.items()}
        empty = [k for k, v in {**values, **params}.items() if v is None]
        if empty:
            self.console.append(f"Campo vacio: {', '.join(empty)}", t.DANGER)
            return
        self._deadline = time.monotonic() + RESPONSE_TIMEOUT_MS / 1000.0
        self.node.send_target(values['X'], values['Y'], values['Z'], params, self.codo.value())

    def on_deadman_start(self):
        self._send()

    def _on_ik_result(self, r):
        if self._deadline is None:
            return          # respuesta a una consigna de otra pestana (DK)
        self._deadline = None
        target = f"({r.get('x', 0):.1f}, {r.get('y', 0):.1f}, {r.get('z', 0):.1f})"

        if r['ok']:
            th3 = r['th3']
            turns = th3 / (2 * math.pi)
            self.console.append(
                f"Llevando el robot a {target} con "
                f"th1 = {r['th1']:.2f}°, th2 = {r['th2']:.2f}°, "
                f"th3 = {th3:.2f} rad ({turns:.2f} vueltas)  [codo {r.get('codo', 1):+d}]")
            if r.get('homed') == 0:
                self.console.append('  Aviso: el robot no ha hecho homing', t.CONNECTED)
        else:
            reason = REASONS.get(r.get('reason', ''), r.get('reason', 'error'))
            if 'joint' in r:
                reason += f" [articulacion {int(r['joint'])}]"
            self.console.append(f'No se puede llegar a {target}: {reason}', t.DANGER)

    def _check_timeout(self):
        if self._deadline is not None and time.monotonic() > self._deadline:
            self._deadline = None
            self.console.append('Sin respuesta del ESP32', t.DANGER)

    def _on_connection(self, connected):
        self.btn_send.set_enabled(connected)
