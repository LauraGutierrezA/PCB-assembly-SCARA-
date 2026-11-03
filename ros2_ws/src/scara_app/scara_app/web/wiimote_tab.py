"""
wiimote_tab.py - Pestana "Wiimote" (web). Equivalente de gui/wiimote_tab.py.

La logica NO esta aqui sino en el nodo (web/ros_node.py -> node.wiimote, ver
wiimote_bridge.py y wiimote_control.py): hay UNA instancia por nodo, aunque
haya varias pantallas abiertas. Esta pagina solo activa / desactiva el control,
muestra el estado que manda el nodo (evento 'wiimote') y cambia los parametros.

El Wiimote esta conectado por Bluetooth al PC del robot, no al iPad ni al
celular: el navegador es solo la pantalla.

Seguridad: arranca DESACTIVADO y en SIMULACION; se desactiva al salir de la
pestana, al cerrarse la pagina (app.py), con STOP / Esc y con HOME en el Wiimote.
"""
import time

from nicegui import ui

from scara_app.gui import theme as t
from scara_app.gui.point_to_point_tab import ROBOT_DEFAULTS
from scara_app.wiimote_control import JOINTS, MODE_IK

from .twin import Twin
from .widgets import LogView, SegmentedToggle, button, caps_label, number_box

JOINT_LABELS = {'th1': 'θ1', 'th2': 'θ2', 'z': 'z'}
LEVEL_COLORS = {'info': None, 'warn': t.CONNECTED, 'error': t.DANGER}
REFRESH_S = 0.12          # la pagina se repinta como maximo ~8 veces por segundo

# (clave de WiimoteParams, etiqueta, unidad, minimo, maximo, paso)
PARAM_FIELDS = [
    ('dead_angle_deg', 'Zona muerta', '°', 1.0, 90.0, 1.0),
    ('full_angle_deg', 'Velocidad máx. a', '°', 5.0, 170.0, 5.0),
    ('speed_th_dps', 'Velocidad θ1, θ2', '°/s', 0.0, 60.0, 1.0),
    ('speed_z_mmps', 'Velocidad z', 'mm/s', 0.0, 40.0, 1.0),
    ('speed_xy_mmps', 'Velocidad X, Y', 'mm/s', 0.0, 80.0, 1.0),
    ('rate_gate_dps', 'Ignorar giros <', '°/s', 0.0, 20.0, 0.5),
    ('lead_deg', 'Correa θ', '°', 1.0, 45.0, 1.0),
    ('lead_mm', 'Correa z', 'mm', 1.0, 50.0, 1.0),
    ('ramp_s', 'Suavizado D-pad (rampa)', 's', 0.0, 2.0, 0.1),
    ('send_period_ms', 'Periodo de envío', 'ms', 40.0, 500.0, 10.0),
]


def bar_html(angle, dead, full, active):
    """Barra del angulo relativo: zona muerta al centro, relleno al pasarla (CSS .wm-bar)."""
    def pct(a):
        return 50.0 + max(-full, min(full, a)) / full * 50.0
    dead_l, dead_r = pct(-dead), pct(dead)
    pos = pct(angle)
    on = ' on' if (abs(angle) > dead and active) else ''
    left, right = (50.0, pos) if pos > 50.0 else (pos, 50.0)
    return (f'<div class="wm-bar"><div class="track"></div>'
            f'<div class="dead" style="left:{dead_l:.2f}%;width:{dead_r - dead_l:.2f}%"></div>'
            f'<div class="fill{on}" style="left:{left:.2f}%;width:{right - left:.2f}%"></div>'
            f'<div class="center"></div>'
            f'<div class="mark{on}" style="left:{pos:.2f}%"></div></div>')


class WiimoteTab:
    def __init__(self, node, hub):
        self.node = node
        self.bridge = node.wiimote
        self.params = dict(ROBOT_DEFAULTS)
        self._last_key = None
        self._last_ui = 0.0
        self.param_boxes, self.axis_boxes, self.bars, self.rates = {}, {}, {}, {}

        with ui.element('div').classes('tab-body compact'):
            with ui.element('div').classes('split'):
                with ui.element('div').classes('left wm-left'):
                    self._build_left()
                with ui.element('div').classes('right'):
                    self.twin = Twin(self.params['d1'], self.params['d2'])

        hub.on('wiimote', self._on_state)
        hub.on('wiimote_log', self._on_log)
        self._on_state(self.bridge.snapshot(), force=True)

    # ---------------- Construccion ----------------
    def _build_left(self):
        with ui.element('div').classes('wm-switches'):
            with ui.element('div').classes('field'):
                caps_label('Control con Wiimote')
                self.enable_toggle = SegmentedToggle(
                    [('Desactivado', False), ('Activado', True)], default=0,
                    on_change=self.bridge.set_enabled)
            with ui.element('div').classes('field'):
                caps_label('Modo')
                self.sim_toggle = SegmentedToggle(
                    [('Simulación', True), ('Robot real', False)], default=0,
                    on_change=self.bridge.set_dry_run)

        self.link_label = ui.html('', sanitize=False).classes('wm-link')
        self.mode_label = ui.label().classes('wm-mode')
        self.hint_label = ui.label().classes('wm-hint')
        self.held_label = ui.label().classes('wm-held')
        with ui.element('div').classes('mini'):
            self.target_label = ui.html('', sanitize=False).classes('mini-ref')
            self.actual_label = ui.label().classes('mini-real')

        caps_label('Giro relativo desde que presionas B').classes('wm-section')
        with ui.element('div').classes('wm-bars'):
            for joint in JOINTS:
                ui.label(JOINT_LABELS[joint]).classes('wm-name')
                self.bars[joint] = ui.html('', sanitize=False)
                self.rates[joint] = ui.label('+0.0 °/s').classes('wm-rate')

        caps_label('Parámetros').classes('wm-section')
        with ui.element('div').classes('wm-params'):
            for key, label, unit, lo, hi, step in PARAM_FIELDS:
                ui.label(label).classes('wm-label')
                box = number_box(0.0, 'param', step=step, suffix=unit, minimum=lo, maximum=hi)
                box.on_value_change(lambda e, k=key: self._param_changed(k, e.value))
                self.param_boxes[key] = box

        caps_label('Eje del giroscopio').classes('wm-section')
        with ui.element('div').classes('wm-axes'):
            for joint in JOINTS:
                with ui.element('div').classes('wm-axis'):
                    ui.label(JOINT_LABELS[joint]).classes('wm-name')
                    combo = ui.select(['x', 'y', 'z'], value='x').props('outlined dense')
                    invert = ui.checkbox('invertir')
                    combo.on_value_change(lambda _e, j=joint: self._axis_changed(j))
                    invert.on_value_change(lambda _e, j=joint: self._axis_changed(j))
                    self.axis_boxes[joint] = (combo, invert)

        with ui.element('div').classes('btn-row').style('margin:16px 0'):
            button('Recalibrar giroscopio', 'secondary', self.bridge.calibrate_gyro)
            button('Consigna = posición actual', 'secondary', self.bridge.go_to_actual)
        self.console = LogView('Consola')

    # ---------------- Cambios del usuario ----------------
    def _param_changed(self, key, value):
        if value is None:
            return
        self.bridge.update_params(**{key: float(value)})

    def _axis_changed(self, joint):
        axes = {j: (c.value, -1.0 if inv.value else 1.0)
                for j, (c, inv) in self.axis_boxes.items()}
        self.bridge.update_params(gyro_axes=axes)

    # ---------------- Eventos del nodo ----------------
    def _on_log(self, payload):
        text, level = payload
        self.console.append(text, LEVEL_COLORS.get(level))

    def _on_state(self, s, force=False):
        now = time.monotonic()
        key = (s['enabled'], s['dry_run'], s['link'], s['mode'], s['held'],
               s['blocked'], s['homed'], s['pump'])
        if not force and key == self._last_key and now - self._last_ui < REFRESH_S:
            return
        self._last_key, self._last_ui = key, now
        p = s['params']
        ik = s['mode'] == MODE_IK

        self.enable_toggle.set_index(1 if s['enabled'] else 0)
        self.sim_toggle.set_index(0 if s['dry_run'] else 1)

        color = t.CONNECTED if s['link'] else t.DISCONNECTED
        self.link_label.set_content(
            f'<span style="color:{color}">●</span>&nbsp; '
            + ('Wiimote conectado' if s['link'] else 'Wiimote sin señal'))
        self.mode_label.set_text('IK · cinemática inversa  (botón 1)' if ik
                                 else 'DK · cinemática directa  (botón 2)')
        if ik:
            self.hint_label.set_text('D-pad: ↑↓ = X, ←→ = Y · +/− = z · mantén B para mover · '
                                     'A = bomba ON/OFF · botón 2 = modo DK · HOME = paro')
        else:
            ax = p['gyro_axes']
            self.hint_label.set_text(
                'Gira el Wiimote más allá de la zona muerta y mantenlo: la articulación se '
                'mueve hasta que vuelvas al centro. '
                + ' · '.join(f"{JOINT_LABELS[j]} ← eje {ax[j][0]}{'(−)' if ax[j][1] < 0 else ''}"
                             for j in JOINTS)
                + ' · mantén B · A = bomba ON/OFF · botón 1 = modo IK · HOME = paro')

        if not s['enabled']:
            held, held_color = 'B: desactivado', t.TEXT_MUTED
        elif s['held']:
            held = 'B: MANTENIDO' + (' (simulación)' if s['dry_run'] else ' — moviendo')
            held_color = t.ACCENT
        else:
            held, held_color = 'B: suelto', t.TEXT_MUTED
        if s['blocked'] and s['held']:
            held, held_color = held + '  ·  LÍMITE', t.CONNECTED
        if s['enabled']:
            pump = 'Bomba: ' + ('ON' if s['pump'] else 'OFF') + (' (simulación)' if s['dry_run'] else '')
            held += '  ·  ' + pump
        self.held_label.set_text(held)
        self.held_label.style(f'color:{held_color}')

        q, (x, y), a = s['q'], s['xy'], s['actual']
        self.target_label.set_content(
            f"<b>consigna</b> θ1 {q['th1']:.1f}° · θ2 {q['th2']:.1f}° · z {q['z']:.1f} mm<br>"
            f"x {x:.1f} · y {y:.1f} mm  [codo {s['codo']:+d}]")
        self.actual_label.set_text(
            'real      —  (sin homing o sin conexión)' if a is None else
            f"real      θ1 {a['th1']:.1f}° · θ2 {a['th2']:.1f}° · z {a['z']:.1f} mm")

        gyro = s['gyro']
        for joint in JOINTS:
            axis, sign = p['gyro_axes'][joint]
            self.bars[joint].set_content(bar_html(
                s['angles'][joint], p['dead_angle_deg'], p['full_angle_deg'],
                s['held'] and not ik))
            self.rates[joint].set_text(f"{sign * gyro[axis]:+.1f} °/s")

        self.twin.set_ref((q['th1'], q['th2']), q['z'], target=(x, y) if ik else None)
        self.twin.set_real(None if a is None else (a['th1'], a['th2']),
                           None if a is None else a['z'])

        for k, box in self.param_boxes.items():
            if box.value is None or abs(float(box.value) - p[k]) > 1e-9:
                box.value = p[k]
        for joint, (combo, invert) in self.axis_boxes.items():
            axis, sign = p['gyro_axes'][joint]
            if combo.value != axis:
                combo.value = axis
            if bool(invert.value) != (sign < 0):
                invert.value = sign < 0
