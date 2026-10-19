"""
dk_tab.py - Pestana DK (web): un slider por articulacion + gemelo.

  - Izquierda: th1, th2 [grados] y z [mm sobre el piso] (slider + campo),
    consigna y posicion real, "Usar posicion actual", "Home", dead-man y consola.
  - Derecha: gemelo (web/twin.py). La consigna (cian) sigue a los sliders en
    vivo, antes de mover; el robot real (gris) sigue a los encoders.

Movimiento: dead-man ("Mantener para mover" o barra espaciadora). Al empezar
se manda la consigna; si se mueve un slider mientras se mantiene (dos dedos
en el iPad, o barra + mouse en el PC), la consigna nueva se manda sola cada
150 ms como maximo. Se envia como consigna cartesiana con codo = signo(th2)
(ver robot_model.py).
"""
import time

from nicegui import ui

from scara_app.gui import theme as t
from scara_app.gui.point_to_point_tab import REASONS, RESPONSE_TIMEOUT_MS, ROBOT_DEFAULTS
from scara_app.robot_model import JOINT_HOME, JOINT_LIMITS, codo_for, fk_xy

from .twin import Twin
from .widgets import LogView, button, deadman_button, number_box, read_number

RESEND_S = 0.15

JOINTS = [
    ('th1', 'θ1', '°'),
    ('th2', 'θ2', '°'),
    ('z',   'z',  'mm'),
]


def real_pose(d):
    """dict de /scara/limits -> {'th1','th2','z'} o None si no hay homing."""
    if d and d.get('CAL') and all(f'Q{i}' in d for i in (1, 2, 3)):
        return {'th1': d['Q1'], 'th2': d['Q2'], 'z': d['Q3']}
    return None


class DkTab:
    def __init__(self, node, hub):
        self.node = node
        self.params = dict(ROBOT_DEFAULTS)
        self._deadline = None
        self._held = False
        self._dirty = False
        self._last_actual = None
        self.sliders, self.boxes = {}, {}

        with ui.element('div').classes('tab-body compact'):
            with ui.element('div').classes('split'):
                with ui.element('div').classes('left'):
                    with ui.element('div').classes('dk-grid'):
                        for key, label, unit in JOINTS:
                            lo, hi = JOINT_LIMITS[key]
                            ui.label(label).classes('dk-name')
                            slider = (ui.slider(min=lo, max=hi, step=0.1, value=JOINT_HOME[key])
                                      .props('thumb-size=28px track-size=6px')
                                      .classes('slider'))
                            box = number_box(JOINT_HOME[key], 'dk', step=0.5, suffix=unit,
                                             minimum=lo, maximum=hi)
                            ui.label(f'{lo:g} … {hi:g} {unit}').classes('dk-limits')
                            slider.on_value_change(lambda e, k=key: self._from_slider(k, e.value))
                            box.on_value_change(lambda e, k=key: self._from_box(k, e.value))
                            self.sliders[key], self.boxes[key] = slider, box

                    with ui.element('div').classes('mini'):
                        self.target_label = ui.html('', sanitize=False).classes('mini-ref')
                        self.actual_label = ui.label().classes('mini-real')

                    with ui.element('div').classes('btn-row').style('margin-bottom:12px'):
                        button('Usar posición actual', 'secondary', self._use_actual)
                        button('Home', 'secondary', lambda: self._set_values(JOINT_HOME))

                    deadman_button('dk')
                    ui.element('div').style('height:12px; flex:none')
                    self.console = LogView('Consola')

                with ui.element('div').classes('right'):
                    self.twin = Twin(self.params['d1'], self.params['d2'])

        ui.timer(RESEND_S, self._tick)
        hub.on('ik', self._on_ik_result)
        hub.on('limits', self._on_limits)
        hub.on('connection', lambda _c: self._show_actual(None))
        self._changed()
        self._on_limits(node.last_limits)

    # ---------------- Sliders <-> campos ----------------
    def _from_slider(self, key, value):
        box = self.boxes[key]
        if value is not None and (box.value is None or abs(box.value - value) > 1e-6):
            box.value = round(value, 1)
        self._changed()

    def _from_box(self, key, value):
        if value is None:
            return
        lo, hi = JOINT_LIMITS[key]
        value = min(max(float(value), lo), hi)
        slider = self.sliders[key]
        if abs(slider.value - value) > 1e-6:
            slider.value = value
        self._changed()

    def _values(self):
        return {k: read_number(self.boxes[k], JOINT_HOME[k]) for k, _, _ in JOINTS}

    def _set_values(self, values):
        for key, value in values.items():
            self.boxes[key].value = round(value, 1)
            self.sliders[key].value = value
        self._changed()

    def _changed(self):
        v = self._values()
        x, y = fk_xy(v['th1'], v['th2'], self.params['d1'], self.params['d2'])
        self.target_label.set_content(
            f'<b>consigna</b> x {x:.1f} · y {y:.1f} · z {v["z"]:.1f} mm  '
            f'[codo {codo_for(v["th2"]):+d}]')
        self.twin.set_ref((v['th1'], v['th2']), v['z'])
        if self._held:
            self._dirty = True

    # ---------------- Dead-man (lo llama app.py) ----------------
    def on_deadman_start(self):
        self._send()

    def set_held(self, held):
        self._held = held
        if not held:
            self._dirty = False

    # ---------------- Envio ----------------
    def _send(self):
        v = self._values()
        self._deadline = time.monotonic() + RESPONSE_TIMEOUT_MS / 1000.0
        self.node.send_joints(v['th1'], v['th2'], v['z'], self.params)
        self._dirty = False

    def _tick(self):
        if self._held and self._dirty:
            self._send()
        if self._deadline is not None and time.monotonic() > self._deadline:
            self._deadline = None
            self.console.append('Sin respuesta del ESP32', t.DANGER)

    def _on_ik_result(self, r):
        if self._deadline is None:
            return          # respuesta a una consigna de Punto a punto
        self._deadline = None
        if r['ok']:
            self.console.append(
                f"Consigna: th1 = {r['th1']:.2f}°, th2 = {r['th2']:.2f}°, "
                f"z = {r.get('z', 0):.1f} mm  ->  ({r.get('x', 0):.1f}, {r.get('y', 0):.1f})")
            if r.get('homed') == 0:
                self.console.append('  Aviso: el robot no ha hecho homing', t.CONNECTED)
        else:
            reason = REASONS.get(r.get('reason', ''), r.get('reason', 'error'))
            if 'joint' in r:
                reason += f" [articulacion {int(r['joint'])}]"
            self.console.append(f'Consigna rechazada: {reason}', t.DANGER)

    # ---------------- Posicion actual ----------------
    def _on_limits(self, d):
        self._show_actual(real_pose(d))

    def _show_actual(self, q):
        self._last_actual = q
        if q is None:
            self.actual_label.set_text('real      —  (sin homing o sin conexión)')
            self.twin.set_real(None, None)
        else:
            x, y = fk_xy(q['th1'], q['th2'], self.params['d1'], self.params['d2'])
            self.actual_label.set_text(
                f"real      x {x:.1f} · y {y:.1f} · z {q['z']:.1f} mm   "
                f"(θ1 {q['th1']:.1f}°, θ2 {q['th2']:.1f}°)")
            self.twin.set_real((q['th1'], q['th2']), q['z'])

    def _use_actual(self):
        if self._last_actual is None:
            self.console.append('No hay posición actual: haz el homing primero', t.DANGER)
            return
        self._set_values({k: min(max(v, JOINT_LIMITS[k][0]), JOINT_LIMITS[k][1])
                          for k, v in self._last_actual.items()})
