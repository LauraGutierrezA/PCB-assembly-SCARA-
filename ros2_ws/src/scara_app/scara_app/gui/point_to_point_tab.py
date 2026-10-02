"""
point_to_point_tab.py - Pestana de punto a punto.

Flujo: parametros del robot + consigna (x, y, z) -> /scara/target
       -> IK en el ESP32 -> /scara/ik -> consola:
       "Llevando el robot a (x, y, z) con th1 = a, th2 = b, th3 = c"

El calculo de la IK lo hace el FIRMWARE (requisito del Reporte 3). Esta
pestana solo envia los datos y muestra lo que responde el ESP32. Los
angulos son las referencias del control (todavia vacio en el firmware).
"""
import math

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QAbstractSpinBox, QDoubleSpinBox, QHBoxLayout, QVBoxLayout, QWidget,
)

from . import theme
from .widgets import LogView, SegmentedToggle, button, caps_label

RESPONSE_TIMEOUT_MS = 1500

# Parametros del robot (modelo DH de MATLAB), en mm; av en mm/vuelta.
# PROVISIONALES: el robot se esta modificando; actualizar con los definitivos.
ROBOT_DEFAULTS = [
    ('d1', 225.0), ('d2', 175.0), ('d3', 163.6), ('d4', 0.0),
    ('d5', -50.0), ('d6', 73.0), ('av', 20.0),
]
TARGET_DEFAULTS = [('X', 300.0), ('Y', 150.0), ('Z', 0.0)]

REASONS = {
    'OUT_OF_REACH': 'fuera del alcance en XY',
    'BAD_PARAMS':   'parametros invalidos (d1, d2 deben ser > 0 y av distinto de 0)',
    'BAD_MESSAGE':  'mensaje con formato incorrecto',
}


def _spinbox(value, width, font_size, name):
    s = QDoubleSpinBox()
    s.setObjectName(name)
    s.setRange(-10000.0, 10000.0)
    s.setDecimals(1)
    s.setSingleStep(1.0)
    s.setValue(value)
    s.setButtonSymbols(QAbstractSpinBox.NoButtons)
    s.setFont(theme.mono(font_size))
    s.setAlignment(Qt.AlignRight)
    s.setFixedWidth(width)
    return s


class PointToPointTab(QWidget):
    def __init__(self, ros_node, parent=None):
        super().__init__(parent)
        self.node = ros_node
        self._waiting = False

        # ---------- Parametros del robot (cajas pequenas) ----------
        self.params = {}
        params_row = QHBoxLayout()
        params_row.setSpacing(8)
        for name, value in ROBOT_DEFAULTS:
            col = QVBoxLayout()
            col.setSpacing(4)
            col.addWidget(caps_label(name))
            box = _spinbox(value, 72, 10, 'paramBox')
            box.lineEdit().returnPressed.connect(self._send)
            self.params[name] = box
            col.addWidget(box)
            params_row.addLayout(col)
        params_row.addStretch()

        btn_defaults = button('Restaurar valores', 'textButton')
        btn_defaults.clicked.connect(self._restore_defaults)
        params_header = QHBoxLayout()
        params_header.addWidget(caps_label('Parametros del robot [mm]  ·  av [mm/vuelta]'))
        params_header.addStretch()
        params_header.addWidget(btn_defaults)

        # ---------- Consigna X, Y, Z (cajas grandes) ----------
        self.fields = {}
        target_row = QHBoxLayout()
        target_row.setSpacing(12)
        for axis, value in TARGET_DEFAULTS:
            col = QVBoxLayout()
            col.setSpacing(6)
            col.addWidget(caps_label(f'{axis} [mm]'))
            box = _spinbox(value, 130, 13, 'coord')
            box.lineEdit().returnPressed.connect(self._send)
            self.fields[axis] = box
            col.addWidget(box)
            target_row.addLayout(col)

        codo_col = QVBoxLayout()
        codo_col.setSpacing(6)
        codo_col.addWidget(caps_label('Codo'))
        self.codo = SegmentedToggle([('+1', 1), ('−1', -1)], default=0)
        self.codo.setToolTip('Configuracion del codo (signo de sin th2)')
        codo_col.addWidget(self.codo)
        target_row.addSpacing(8)
        target_row.addLayout(codo_col)
        target_row.addSpacing(8)

        self.btn_send = button('Enviar consigna', 'primary', 'Calcular IK en el ESP32 (Enter)')
        self.btn_send.clicked.connect(self._send)
        send_col = QVBoxLayout()
        send_col.addStretch()
        send_col.addWidget(self.btn_send)
        target_row.addLayout(send_col)
        target_row.addStretch()

        # ---------- Consola ----------
        self.console = LogView('Consola')

        self._timeout = QTimer(self)
        self._timeout.setSingleShot(True)
        self._timeout.timeout.connect(self._on_timeout)

        # ---------- Layout ----------
        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 20, 24, 24)
        layout.setSpacing(0)
        layout.addLayout(params_header)
        layout.addSpacing(8)
        layout.addLayout(params_row)
        layout.addSpacing(22)
        layout.addLayout(target_row)
        layout.addSpacing(22)
        layout.addWidget(self.console, stretch=1)

        self.node.signals.ik_result.connect(self._on_ik_result)
        self.node.signals.connection_changed.connect(self._on_connection)
        self._on_connection(self.node.connected)

    # ------------------------------------------------------------------
    def _restore_defaults(self):
        for name, value in ROBOT_DEFAULTS:
            self.params[name].setValue(value)

    def _send(self):
        if not self.btn_send.isEnabled():
            return
        x, y, z = (self.fields[a].value() for a in ('X', 'Y', 'Z'))
        params = {name: box.value() for name, box in self.params.items()}
        self._waiting = True
        self._timeout.start(RESPONSE_TIMEOUT_MS)
        self.node.send_target(x, y, z, params, self.codo.value())

    def _on_ik_result(self, r):
        self._waiting = False
        self._timeout.stop()
        target = f"({r.get('x', 0):.1f}, {r.get('y', 0):.1f}, {r.get('z', 0):.1f})"

        if r['ok']:
            th3 = r['th3']
            turns = th3 / (2 * math.pi)
            self.console.append(
                f"Llevando el robot a {target} con "
                f"th1 = {r['th1']:.2f}°, th2 = {r['th2']:.2f}°, "
                f"th3 = {th3:.2f} rad ({turns:.2f} vueltas)  [codo {r.get('codo', 1):+d}]")
            if r.get('homed') == 0:
                self.console.append('  Aviso: el robot no ha hecho homing', theme.CONNECTED)
        else:
            reason = REASONS.get(r.get('reason', ''), r.get('reason', 'error'))
            self.console.append(f'No se puede llegar a {target}: {reason}', theme.DANGER)

    def _on_timeout(self):
        if self._waiting:
            self._waiting = False
            self.console.append('Sin respuesta del ESP32', theme.DANGER)

    def _on_connection(self, connected):
        self.btn_send.setEnabled(connected)
