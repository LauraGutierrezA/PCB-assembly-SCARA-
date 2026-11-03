"""
wiimote_tab.py - Pestana "Wiimote": mover el robot con un control Wiimote.

La logica NO esta aqui sino en el nodo (ros_node.py -> self.node.wiimote, ver
wiimote_bridge.py y wiimote_control.py). Esta pestana solo:
  - activa / desactiva el control y elige simulacion o robot real,
  - muestra el modo, la consigna, la posicion real y los angulos del giroscopio,
  - cambia los parametros (zona muerta, velocidades, ejes del giroscopio),
  - muestra la consola y el gemelo.

Wiimote:
  1  modo cinematica inversa (D-pad = X / Y, + / - = z)
  2  modo cinematica directa (el giro mueve th1 / th2 / z, estilo joystick)
  B  dead-man: el robot solo se mueve mientras se mantiene B
  HOME  paro de emergencia

Seguridad: arranca DESACTIVADO y en SIMULACION. Se desactiva solo al salir de
la pestana (main_window.py), con STOP / Esc o con HOME en el Wiimote.
"""
from PyQt5.QtCore import QRectF, Qt
from PyQt5.QtGui import QColor, QPainter, QPen
from PyQt5.QtWidgets import (
    QAbstractSpinBox, QCheckBox, QComboBox, QDoubleSpinBox, QGridLayout, QHBoxLayout, QLabel,
    QScrollArea, QVBoxLayout, QWidget,
)

from scara_app.gui.point_to_point_tab import ROBOT_DEFAULTS
from scara_app.wiimote_control import JOINTS, MODE_IK

from . import theme
from .twin_widget import TwinWidget
from .widgets import LogView, SegmentedToggle, button, caps_label

JOINT_LABELS = {'th1': 'θ1', 'th2': 'θ2', 'z': 'z'}
LEVEL_COLORS = {'info': None, 'warn': theme.CONNECTED, 'error': theme.DANGER}

# (clave de WiimoteParams, etiqueta, unidad, minimo, maximo, paso)
PARAM_FIELDS = [
    ('dead_angle_deg', 'Zona muerta', '°', 1.0, 90.0, 1.0),
    ('full_angle_deg', 'Velocidad máx. a', '°', 5.0, 170.0, 5.0),
    ('speed_th_dps', 'Velocidad θ1, θ2', '°/s', 0.0, 60.0, 1.0),
    ('speed_z_mmps', 'Velocidad z', 'mm/s', 0.0, 40.0, 1.0),
    ('speed_xy_mmps', 'Velocidad X, Y (D-pad)', 'mm/s', 0.0, 80.0, 1.0),
    ('rate_gate_dps', 'Ignorar giros < ', '°/s', 0.0, 20.0, 0.5),
    ('lead_deg', 'Correa θ', '°', 1.0, 45.0, 1.0),
    ('lead_mm', 'Correa z', 'mm', 1.0, 50.0, 1.0),
    ('ramp_s', 'Suavizado D-pad (rampa)', 's', 0.0, 2.0, 0.1),
    ('send_period_ms', 'Periodo de envío', 'ms', 40.0, 500.0, 10.0),
]


class AngleBar(QWidget):
    """Barra del angulo relativo del giro: zona muerta al centro, relleno al pasarla."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(26)
        self.angle, self.dead, self.full = 0.0, 25.0, 70.0
        self.active = False

    def set_values(self, angle, dead, full, active):
        self.angle, self.dead, self.full, self.active = angle, dead, full, active
        self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        cx = w / 2.0
        half = w / 2.0 - 2.0
        scale = half / max(self.full, 1e-6)

        p.setPen(Qt.NoPen)
        p.setBrush(QColor(theme.DIVIDER))
        p.drawRoundedRect(QRectF(0, h / 2 - 5, w, 10), 5, 5)
        p.setBrush(QColor(theme.BG_PAGE))                        # zona muerta
        p.drawRect(QRectF(cx - self.dead * scale, h / 2 - 5, 2 * self.dead * scale, 10))

        beyond = abs(self.angle) > self.dead
        color = QColor(theme.ACCENT if (beyond and self.active) else theme.TEXT_MUTED)
        x = cx + max(-self.full, min(self.full, self.angle)) * scale
        p.setBrush(color)
        p.drawRect(QRectF(min(cx, x), h / 2 - 5, abs(x - cx), 10))
        p.setPen(QPen(QColor(theme.TEXT_STRONG), 2))
        p.drawLine(int(cx), 3, int(cx), h - 3)                   # centro
        p.setPen(QPen(color, 3))
        p.drawLine(int(x), 2, int(x), h - 2)                     # angulo actual
        p.end()


class WiimoteTab(QWidget):
    def __init__(self, ros_node, parent=None):
        super().__init__(parent)
        self.node = ros_node
        self.bridge = ros_node.wiimote
        self.params = dict(ROBOT_DEFAULTS)
        self._state = None
        self._axis_boxes = {}

        # ---------- Activacion ----------
        self.enable_toggle = SegmentedToggle([('Desactivado', False), ('Activado', True)])
        self.enable_toggle.connect_changed(self.bridge.set_enabled)
        self.sim_toggle = SegmentedToggle([('Simulación', True), ('Robot real', False)])
        self.sim_toggle.connect_changed(self.bridge.set_dry_run)
        switches = QHBoxLayout()
        switches.setSpacing(16)
        for title, widget in (('Control con Wiimote', self.enable_toggle),
                              ('Modo', self.sim_toggle)):
            col = QVBoxLayout()
            col.setSpacing(6)
            col.addWidget(caps_label(title))
            col.addWidget(widget)
            switches.addLayout(col)
        switches.addStretch()

        # ---------- Estado ----------
        self.link_label = QLabel()
        self.link_label.setFont(theme.mono(10))
        self.mode_label = QLabel()
        self.mode_label.setFont(theme.mono(15, bold=True))
        self.mode_label.setStyleSheet(f'color:{theme.TEXT_STRONG};')
        self.hint_label = QLabel()
        self.hint_label.setFont(theme.sans(9))
        self.hint_label.setWordWrap(True)
        self.hint_label.setStyleSheet(f'color:{theme.TEXT_MUTED};')
        self.held_label = QLabel()
        self.held_label.setFont(theme.mono(11))
        self.target_label = QLabel()
        self.target_label.setFont(theme.mono(11))
        self.actual_label = QLabel()
        self.actual_label.setFont(theme.mono(11))
        self.actual_label.setStyleSheet(f'color:{theme.TEXT_MUTED};')

        status = QVBoxLayout()
        status.setSpacing(6)
        status.addWidget(self.link_label)
        status.addWidget(self.mode_label)
        status.addWidget(self.hint_label)
        status.addWidget(self.held_label)
        status.addSpacing(6)
        status.addWidget(caps_label('Consigna'))
        status.addWidget(self.target_label)
        status.addWidget(caps_label('Posición actual (encoders)'))
        status.addWidget(self.actual_label)

        # ---------- Barras del giro ----------
        self.bars, self.rate_labels = {}, {}
        bars = QGridLayout()
        bars.setHorizontalSpacing(10)
        bars.setVerticalSpacing(8)
        for row, joint in enumerate(JOINTS):
            name = QLabel(JOINT_LABELS[joint])
            name.setFont(theme.mono(13, bold=True))
            name.setStyleSheet(f'color:{theme.TEXT_STRONG};')
            name.setFixedWidth(30)
            bar = AngleBar()
            rate = QLabel('0.0 °/s')
            rate.setFont(theme.mono(9))
            rate.setStyleSheet(f'color:{theme.TEXT_MUTED};')
            rate.setFixedWidth(90)
            rate.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            bars.addWidget(name, row, 0)
            bars.addWidget(bar, row, 1)
            bars.addWidget(rate, row, 2)
            self.bars[joint], self.rate_labels[joint] = bar, rate
        bars.setColumnStretch(1, 1)

        # ---------- Parametros ----------
        self.param_boxes = {}
        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(8)
        for i, (key, label, unit, lo, hi, step) in enumerate(PARAM_FIELDS):
            box = QDoubleSpinBox()
            box.setObjectName('paramBox')
            box.setRange(lo, hi)
            box.setDecimals(1)
            box.setSingleStep(step)
            box.setSuffix(f' {unit}')
            box.setButtonSymbols(QAbstractSpinBox.NoButtons)
            box.setFont(theme.mono(10))
            box.setAlignment(Qt.AlignRight)
            box.setFixedWidth(100)
            box.editingFinished.connect(lambda k=key: self._param_edited(k))
            text = QLabel(label)
            text.setFont(theme.sans(9))
            text.setStyleSheet(f'color:{theme.TEXT_MUTED};')
            r, c = divmod(i, 2)
            grid.addWidget(text, r, c * 2)
            grid.addWidget(box, r, c * 2 + 1)
            self.param_boxes[key] = box

        axes = QGridLayout()
        axes.setHorizontalSpacing(10)
        axes.setVerticalSpacing(6)
        axes.addWidget(caps_label('Eje del giroscopio'), 0, 0, 1, 3)
        for row, joint in enumerate(JOINTS, start=1):
            name = QLabel(JOINT_LABELS[joint])
            name.setFont(theme.mono(11, bold=True))
            combo = QComboBox()
            combo.addItems(['x', 'y', 'z'])
            combo.setFixedWidth(60)
            invert = QCheckBox('invertir')
            invert.setFont(theme.sans(9))
            combo.activated.connect(lambda _i, j=joint: self._axis_edited(j))
            invert.clicked.connect(lambda _c, j=joint: self._axis_edited(j))
            axes.addWidget(name, row, 0)
            axes.addWidget(combo, row, 1)
            axes.addWidget(invert, row, 2)
            self._axis_boxes[joint] = (combo, invert)
        axes.setColumnStretch(3, 1)

        btn_cal = button('Recalibrar giroscopio', 'secondary',
                         'Deja el Wiimote quieto sobre la mesa unos 2 segundos')
        btn_cal.clicked.connect(self.bridge.calibrate_gyro)
        btn_actual = button('Consigna = posición actual', 'secondary',
                            'En simulación: empezar desde donde esta el robot')
        btn_actual.clicked.connect(self.bridge.go_to_actual)
        tools = QHBoxLayout()
        tools.setSpacing(12)
        tools.addWidget(btn_cal)
        tools.addWidget(btn_actual)
        tools.addStretch()

        self.console = LogView('Consola')
        self.console.setMinimumHeight(170)

        # ---------- Gemelo ----------
        self.twin = TwinWidget(self.params['d1'], self.params['d2'])

        # ---------- Layout ----------
        left = QVBoxLayout()
        left.setContentsMargins(0, 0, 8, 0)
        left.setSpacing(0)
        left.addLayout(switches)
        left.addSpacing(14)
        left.addLayout(status)
        left.addSpacing(14)
        left.addWidget(caps_label('Giro relativo desde que presionas B'))
        left.addSpacing(6)
        left.addLayout(bars)
        left.addSpacing(14)
        left.addWidget(caps_label('Parámetros'))
        left.addSpacing(6)
        left.addLayout(grid)
        left.addSpacing(10)
        left.addLayout(axes)
        left.addSpacing(14)
        left.addLayout(tools)
        left.addSpacing(14)
        left.addWidget(self.console)
        left_box = QWidget()
        left_box.setLayout(left)
        scroll = QScrollArea()
        scroll.setWidget(left_box)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setFixedWidth(540)
        scroll.setStyleSheet(
            'QScrollArea, QScrollArea > QWidget > QWidget {background: transparent;}')

        layout = QHBoxLayout(self)
        layout.setContentsMargins(24, 20, 24, 24)
        layout.setSpacing(20)
        layout.addWidget(scroll)
        layout.addWidget(self.twin, stretch=1)

        self.node.signals.wiimote_state.connect(self._on_state)
        self.node.signals.wiimote_log.connect(self._on_log)
        self._on_state(self.bridge.snapshot())

    # ---------------- Cambios del usuario ----------------
    def _param_edited(self, key):
        self.bridge.update_params(**{key: self.param_boxes[key].value()})

    def _axis_edited(self, joint):
        axes = {j: (c.currentText(), -1.0 if inv.isChecked() else 1.0)
                for j, (c, inv) in self._axis_boxes.items()}
        self.bridge.update_params(gyro_axes=axes)

    # ---------------- Mensajes del nodo ----------------
    def _on_log(self, text, level):
        self.console.append(text, LEVEL_COLORS.get(level))

    def _on_state(self, s):
        self._state = s
        p = s['params']

        # activacion (cambia tambien por STOP / HOME / salir de la pestana)
        self.enable_toggle.set_index(1 if s['enabled'] else 0)
        self.sim_toggle.set_index(0 if s['dry_run'] else 1)

        color = theme.CONNECTED if s['link'] else theme.DISCONNECTED
        self.link_label.setText(f'<span style="color:{color}">●</span>&nbsp;'
                                + ('Wiimote conectado' if s['link'] else 'Wiimote sin señal'))

        ik = s['mode'] == MODE_IK
        self.mode_label.setText('IK · cinemática inversa  (botón 1)' if ik
                                else 'DK · cinemática directa  (botón 2)')
        if ik:
            self.hint_label.setText('D-pad: ↑↓ = X, ←→ = Y · +/− = z · mantén B para mover · '
                                    'A = bomba ON/OFF · botón 2 = modo DK · HOME = paro')
        else:
            ax = p['gyro_axes']
            self.hint_label.setText(
                'Gira el Wiimote más allá de la zona muerta y mantenlo: la articulación se '
                'mueve hasta que vuelvas al centro. '
                + ' · '.join(f"{JOINT_LABELS[j]} ← eje {ax[j][0]}{'(−)' if ax[j][1] < 0 else ''}"
                             for j in JOINTS)
                + ' · mantén B · A = bomba ON/OFF · botón 1 = modo IK · HOME = paro')

        if not s['enabled']:
            held, held_color = 'B: desactivado', theme.TEXT_MUTED
        elif s['held']:
            held = 'B: MANTENIDO' + (' (simulación)' if s['dry_run'] else ' — moviendo')
            held_color = theme.ACCENT
        else:
            held, held_color = 'B: suelto', theme.TEXT_MUTED
        if s['blocked'] and s['held']:
            held += '  ·  LÍMITE'
            held_color = theme.CONNECTED
        if s['enabled']:
            held += ('  ·  Bomba: ' + ('ON' if s['pump'] else 'OFF')
                     + (' (simulación)' if s['dry_run'] else ''))
        self.held_label.setText(held)
        self.held_label.setStyleSheet(f'color:{held_color};')

        q = s['q']
        x, y = s['xy']
        self.target_label.setText(
            f"θ1 {q['th1']:.1f}° · θ2 {q['th2']:.1f}° · z {q['z']:.1f} mm\n"
            f"x {x:.1f} · y {y:.1f} mm   [codo {s['codo']:+d}]")
        a = s['actual']
        self.actual_label.setText(
            '—  (sin homing o sin conexión)' if a is None else
            f"θ1 {a['th1']:.1f}° · θ2 {a['th2']:.1f}° · z {a['z']:.1f} mm")

        gyro = s['gyro']
        for joint in JOINTS:
            axis, sign = p['gyro_axes'][joint]
            self.bars[joint].set_values(s['angles'][joint], p['dead_angle_deg'],
                                        p['full_angle_deg'], s['held'] and not ik)
            self.rate_labels[joint].setText(f"{sign * gyro[axis]:+.1f} °/s")

        self.twin.set_ref((q['th1'], q['th2']), q['z'], target=(x, y) if ik else None)
        self.twin.set_real(None if a is None else (a['th1'], a['th2']),
                           None if a is None else a['z'])

        for key, box in self.param_boxes.items():
            if not box.hasFocus() and abs(box.value() - p[key]) > 1e-9:
                box.blockSignals(True)
                box.setValue(p[key])
                box.blockSignals(False)
        for joint, (combo, invert) in self._axis_boxes.items():
            axis, sign = p['gyro_axes'][joint]
            if not combo.hasFocus() and combo.currentText() != axis:
                combo.setCurrentText(axis)
            if invert.isChecked() != (sign < 0):
                invert.setChecked(sign < 0)
