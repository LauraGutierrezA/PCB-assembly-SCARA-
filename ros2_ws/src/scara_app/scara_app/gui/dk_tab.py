"""
dk_tab.py - Pestana DK (cinematica directa): un slider por articulacion + gemelo.

A la derecha, el gemelo en planta (twin_widget.py): la consigna (cian) sigue a
los sliders en vivo, antes de mover; el robot real (gris) sigue a los encoders.

  th1 [grados]   th2 [grados]   z [mm sobre el piso de la base]

Con los sliders se arma la consigna articular; abajo se ve a que (x, y)
corresponde (cinematica directa, robot_model.fk_xy) y donde esta el robot
ahora (Q1, Q2, Q3 de /scara/limits, despues del homingPRO).

Para mover: mantener "Mantener para mover" o la barra espaciadora
(dead-man). Al empezar se manda la consigna de los sliders; si se mueve un
slider mientras se mantiene, la consigna nueva se manda sola (cada 150 ms
como maximo).

Se envia como consigna cartesiana normal (/scara/target) con
codo = signo(th2): la IK del firmware devuelve los mismos angulos y aplica
los mismos chequeos de rango. Ver robot_model.py.
"""
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QAbstractSpinBox, QDoubleSpinBox, QGridLayout, QHBoxLayout, QLabel, QSlider,
    QVBoxLayout, QWidget,
)

from scara_app.robot_model import JOINT_HOME, JOINT_LIMITS, codo_for, fk_xy

from . import theme
from .point_to_point_tab import REASONS, RESPONSE_TIMEOUT_MS, ROBOT_DEFAULTS
from .twin_widget import TwinWidget
from .widgets import DeadmanButton, LogView, button, caps_label

SLIDER_SCALE = 10          # el QSlider es entero: 1 paso = 0.1 grados / 0.1 mm
RESEND_MS = 150            # consigna nueva como maximo cada 150 ms mientras se mueve

JOINTS = [
    # clave, etiqueta, unidad
    ('th1', 'θ1', '°'),
    ('th2', 'θ2', '°'),
    ('z',   'z',  ' mm'),
]


class DkTab(QWidget):
    def __init__(self, ros_node, deadman, parent=None):
        super().__init__(parent)
        self.node = ros_node
        self.deadman = deadman
        self.params = dict(ROBOT_DEFAULTS)
        self._waiting = False
        self._dirty = False          # se movio un slider mientras se mantiene el dead-man

        # ---------- Sliders ----------
        grid = QGridLayout()
        grid.setHorizontalSpacing(16)
        grid.setVerticalSpacing(14)
        self.sliders, self.boxes = {}, {}
        for row, (key, label, unit) in enumerate(JOINTS):
            lo, hi = JOINT_LIMITS[key]
            name = QLabel(label)
            name.setFont(theme.mono(15, bold=True))
            name.setStyleSheet(f'color:{theme.TEXT_STRONG};')
            name.setFixedWidth(36)

            slider = QSlider(Qt.Horizontal)
            slider.setRange(round(lo * SLIDER_SCALE), round(hi * SLIDER_SCALE))
            slider.setMinimumHeight(36)

            box = QDoubleSpinBox()
            box.setObjectName('paramBox')
            box.setRange(lo, hi)
            box.setDecimals(1)
            box.setSingleStep(0.5)
            box.setSuffix(unit)
            box.setButtonSymbols(QAbstractSpinBox.NoButtons)
            box.setFont(theme.mono(12))
            box.setAlignment(Qt.AlignRight)
            box.setFixedWidth(110)

            limits = QLabel(f'{lo:g} … {hi:g}{unit}')
            limits.setFont(theme.mono(9))
            limits.setStyleSheet(f'color:{theme.TEXT_MUTED};')

            slider.valueChanged.connect(lambda v, k=key: self._from_slider(k, v))
            box.valueChanged.connect(lambda v, k=key: self._from_box(k, v))

            grid.addWidget(name, row, 0)
            grid.addWidget(slider, row, 1)
            grid.addWidget(box, row, 2)
            grid.addWidget(limits, row, 3)
            self.sliders[key], self.boxes[key] = slider, box
        grid.setColumnStretch(1, 1)

        # ---------- Lecturas ----------
        self.target_label = QLabel()
        self.target_label.setFont(theme.mono(11))
        self.actual_label = QLabel()
        self.actual_label.setFont(theme.mono(11))
        self.actual_label.setStyleSheet(f'color:{theme.TEXT_MUTED};')
        readout = QVBoxLayout()
        readout.setSpacing(6)
        readout.addWidget(caps_label('Consigna'))
        readout.addWidget(self.target_label)
        readout.addSpacing(6)
        readout.addWidget(caps_label('Posicion actual (encoders)'))
        readout.addWidget(self.actual_label)

        btn_actual = button('Usar posicion actual', 'secondary',
                            'Poner los sliders donde esta el robot (sin saltos)')
        btn_actual.clicked.connect(self._use_actual)
        btn_home = button('Home', 'secondary', 'th1 = 0, th2 = 0, z en la mitad')
        btn_home.clicked.connect(self._go_home_values)
        tools = QHBoxLayout()
        tools.setSpacing(12)
        tools.addWidget(btn_actual)
        tools.addWidget(btn_home)
        tools.addStretch()

        self.deadman_button = DeadmanButton(deadman, self._send)
        self.console = LogView('Consola')

        # ---------- Gemelo (derecha) ----------
        self.twin = TwinWidget(self.params['d1'], self.params['d2'])

        left = QVBoxLayout()
        left.setSpacing(0)
        left.addLayout(grid)
        left.addSpacing(18)
        left.addLayout(readout)
        left.addSpacing(16)
        left.addLayout(tools)
        left.addSpacing(16)
        left.addWidget(self.deadman_button)
        left.addSpacing(18)
        left.addWidget(self.console, stretch=1)
        left_box = QWidget()
        left_box.setLayout(left)
        left_box.setFixedWidth(520)
        left.setContentsMargins(0, 0, 0, 0)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(24, 20, 24, 24)
        layout.setSpacing(20)
        layout.addWidget(left_box)
        layout.addWidget(self.twin, stretch=1)

        self._last_actual = None
        self._resend = QTimer(self)
        self._resend.setInterval(RESEND_MS)
        self._resend.timeout.connect(self._resend_if_dirty)
        self._timeout = QTimer(self)
        self._timeout.setSingleShot(True)
        self._timeout.timeout.connect(self._on_timeout)

        self.node.signals.ik_result.connect(self._on_ik_result)
        self.node.signals.limits_received.connect(self._on_limits)
        self.node.signals.connection_changed.connect(lambda _c: self._show_actual(None))
        self.deadman.held_changed.connect(self._on_held)

        self._set_values(JOINT_HOME)
        self._show_actual(None)

    # ---------------- Sliders <-> cajas ----------------
    def _from_slider(self, key, v):
        box = self.boxes[key]
        value = v / SLIDER_SCALE
        if abs(box.value() - value) > 1e-6:
            box.blockSignals(True)
            box.setValue(value)
            box.blockSignals(False)
        self._changed()

    def _from_box(self, key, v):
        slider = self.sliders[key]
        slider.blockSignals(True)
        slider.setValue(round(v * SLIDER_SCALE))
        slider.blockSignals(False)
        self._changed()

    def _values(self):
        return {k: self.boxes[k].value() for k, _, _ in JOINTS}

    def _set_values(self, values):
        for key, value in values.items():
            self.boxes[key].setValue(value)    # mueve tambien el slider
        self._changed()

    def _changed(self):
        v = self._values()
        x, y = fk_xy(v['th1'], v['th2'], self.params['d1'], self.params['d2'])
        self.target_label.setText(
            f"x {x:.1f} · y {y:.1f} · z {v['z']:.1f} mm   [codo {codo_for(v['th2']):+d}]")
        self.twin.set_ref((v['th1'], v['th2']), v['z'])
        if self.deadman.held:
            self._dirty = True

    # ---------------- Envio ----------------
    def _send(self):
        v = self._values()
        self._waiting = True
        self._timeout.start(RESPONSE_TIMEOUT_MS)
        self.node.send_joints(v['th1'], v['th2'], v['z'], self.params)
        self._dirty = False

    def _on_held(self, held):
        if held:
            self._resend.start()
        else:
            self._resend.stop()
            self._dirty = False

    def _resend_if_dirty(self):
        if self._dirty:
            self._send()

    def _on_ik_result(self, r):
        if not self._waiting:
            return          # respuesta a una consigna de Punto a punto
        self._waiting = False
        self._timeout.stop()
        if r['ok']:
            self.console.append(
                f"Consigna: th1 = {r['th1']:.2f}°, th2 = {r['th2']:.2f}°, "
                f"z = {r.get('z', 0):.1f} mm"
                f"  ->  ({r.get('x', 0):.1f}, {r.get('y', 0):.1f})")
            if r.get('homed') == 0:
                self.console.append('  Aviso: el robot no ha hecho homing', theme.CONNECTED)
        else:
            reason = REASONS.get(r.get('reason', ''), r.get('reason', 'error'))
            if 'joint' in r:
                reason += f" [articulacion {int(r['joint'])}]"
            self.console.append(f'Consigna rechazada: {reason}', theme.DANGER)

    def _on_timeout(self):
        if self._waiting:
            self._waiting = False
            self.console.append('Sin respuesta del ESP32', theme.DANGER)

    # ---------------- Posicion actual ----------------
    def _on_limits(self, d):
        if d.get('CAL') and all(f'Q{i}' in d for i in (1, 2, 3)):
            self._show_actual({'th1': d['Q1'], 'th2': d['Q2'], 'z': d['Q3']})
        else:
            self._show_actual(None)

    def _show_actual(self, q):
        self._last_actual = q
        if q is None:
            self.actual_label.setText('—  (sin homing o sin conexion)')
            self.twin.set_real(None, None)
        else:
            self.actual_label.setText(
                f"θ1 {q['th1']:.1f}° · θ2 {q['th2']:.1f}° · z {q['z']:.1f} mm")
            self.twin.set_real((q['th1'], q['th2']), q['z'])

    def _use_actual(self):
        if self._last_actual is None:
            self.console.append('No hay posicion actual: haz el homing primero', theme.DANGER)
            return
        clamped = {k: min(max(v, JOINT_LIMITS[k][0]), JOINT_LIMITS[k][1])
                   for k, v in self._last_actual.items()}
        self._set_values(clamped)

    def _go_home_values(self):
        self._set_values(JOINT_HOME)
