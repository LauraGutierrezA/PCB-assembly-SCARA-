"""
circuit_test_tab.py - Pestana "Prueba de circuito".

Un modulo por motor, para probar cada parte del circuito por separado:
  1. End-stops: dos circulos (L y R) que se encienden al presionar el final
     de carrera. Funcionan aunque el motor este apagado.
  2. Accionamiento: sentido (-> L / -> R), velocidad (%), duracion (s),
     Start y Stop. El motor gira el tiempo indicado y se detiene solo.
     NO se detiene al tocar un end-stop (la prueba es aislada).

Mientras el homing esta corriendo, los Start se deshabilitan (el homing
tiene prioridad sobre los motores). Stop siempre funciona.

Topicos: /scara/test_cmd (ordenes) y /scara/limits (end-stops y motores).
"""
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QAbstractSpinBox, QDoubleSpinBox, QFrame, QHBoxLayout, QLabel, QSlider,
    QVBoxLayout, QWidget,
)

from . import theme
from .widgets import SegmentedToggle, button, caps_label

NUM_MOTORS = 3
# Velocidad inicial de cada motor = la del homing (200, 512, 430 de 1023)
DEFAULT_SPEED_PCT = {1: 20, 2: 50, 3: 42}
DEFAULT_DURATION_S = 1.0

LED_SIZE = 38


class EndStopLed(QLabel):
    """Circulo con una letra (L o R) que se enciende cuando el end-stop esta presionado."""

    def __init__(self, letter, parent=None):
        super().__init__(letter, parent)
        self.setFixedSize(LED_SIZE, LED_SIZE)
        self.setAlignment(Qt.AlignCenter)
        self.setFont(theme.mono(11, bold=True))
        self.set_state(None)

    def set_state(self, pressed):
        """pressed: True, False o None (sin datos)."""
        if pressed is None:
            bg, border, fg = 'transparent', theme.DIVIDER, theme.TEXT_DISABLED
        elif pressed:
            bg, border, fg = theme.ACCENT, theme.ACCENT, theme.BG_PAGE
        else:
            bg, border, fg = theme.BG_PAGE, theme.BORDER_CARD, theme.TEXT_MUTED
        self.setStyleSheet(
            f'background:{bg}; color:{fg}; border:2px solid {border};'
            f' border-radius:{LED_SIZE // 2}px;')


class MotorModule(QFrame):
    """Modulo de prueba de un motor: end-stops + accionamiento."""

    def __init__(self, motor, ros_node, parent=None):
        super().__init__(parent)
        self.motor = motor
        self.node = ros_node
        self.setObjectName('panel')

        # ---------- Encabezado ----------
        title = QLabel(f'MOTOR {motor}')
        title.setFont(theme.mono(12, bold=True))
        title.setStyleSheet(f'color:{theme.TEXT_STRONG};')
        self.running_label = QLabel('')
        self.running_label.setFont(theme.caps_label_font(7))
        header = QHBoxLayout()
        header.addWidget(title)
        header.addStretch()
        header.addWidget(self.running_label)

        # ---------- 1. End-stops ----------
        self.led_l = EndStopLed('L')
        self.led_r = EndStopLed('R')
        leds = QHBoxLayout()
        leds.setSpacing(14)
        leds.addWidget(self.led_l)
        leds.addWidget(self.led_r)
        leds.addStretch()

        # ---------- 2. Accionamiento ----------
        self.direction = SegmentedToggle([('→ L', 'L'), ('→ R', 'R')], default=0)

        self.speed = QSlider(Qt.Horizontal)
        self.speed.setRange(0, 100)
        self.speed.setValue(DEFAULT_SPEED_PCT.get(motor, 30))
        self.speed.setStyleSheet(
            f'QSlider::groove:horizontal {{ height:4px; background:{theme.DIVIDER};'
            f' border-radius:2px; }}'
            f'QSlider::sub-page:horizontal {{ background:{theme.PRIMARY}; border-radius:2px; }}'
            f'QSlider::handle:horizontal {{ background:{theme.ACCENT}; width:14px;'
            f' height:14px; margin:-5px 0; border-radius:7px; }}')
        self.speed_value = QLabel()
        self.speed_value.setFont(theme.mono(10))
        self.speed_value.setFixedWidth(44)
        self.speed_value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.speed.valueChanged.connect(lambda v: self.speed_value.setText(f'{v} %'))
        self.speed_value.setText(f'{self.speed.value()} %')
        speed_row = QHBoxLayout()
        speed_row.addWidget(self.speed, stretch=1)
        speed_row.addWidget(self.speed_value)

        self.duration = QDoubleSpinBox()
        self.duration.setObjectName('paramBox')
        self.duration.setRange(0.1, 60.0)
        self.duration.setDecimals(1)
        self.duration.setSingleStep(0.1)
        self.duration.setValue(DEFAULT_DURATION_S)
        self.duration.setSuffix(' s')
        self.duration.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.duration.setFont(theme.mono(10))
        self.duration.setAlignment(Qt.AlignRight)
        self.duration.setFixedWidth(90)

        self.btn_start = button('Start', 'primary', 'Mover el motor el tiempo indicado')
        self.btn_stop = button('Stop', 'danger', 'Detener este motor')
        for b in (self.btn_start, self.btn_stop):
            b.setMinimumWidth(0)
        self.btn_start.clicked.connect(self._start)
        self.btn_stop.clicked.connect(lambda: self.node.test_stop(self.motor))
        buttons = QHBoxLayout()
        buttons.setSpacing(8)
        buttons.addWidget(self.btn_start)
        buttons.addWidget(self.btn_stop)

        # ---------- Layout ----------
        lay = QVBoxLayout(self)
        lay.setContentsMargins(16, 14, 16, 16)
        lay.setSpacing(0)
        lay.addLayout(header)
        lay.addSpacing(14)
        lay.addWidget(caps_label('End-stops'))
        lay.addSpacing(8)
        lay.addLayout(leds)
        lay.addSpacing(18)
        lay.addWidget(self._divider())
        lay.addSpacing(14)
        lay.addWidget(caps_label('Accionamiento'))
        lay.addSpacing(10)
        lay.addWidget(self._small('Sentido'))
        lay.addSpacing(4)
        lay.addWidget(self.direction)
        lay.addSpacing(12)
        lay.addWidget(self._small('Velocidad'))
        lay.addSpacing(4)
        lay.addLayout(speed_row)
        lay.addSpacing(12)
        lay.addWidget(self._small('Duracion'))
        lay.addSpacing(4)
        lay.addWidget(self.duration)
        lay.addSpacing(16)
        lay.addLayout(buttons)
        lay.addStretch()

        self.set_running(False)

    # ------------------------------------------------------------------
    @staticmethod
    def _small(text):
        lbl = QLabel(text)
        lbl.setFont(theme.sans(9))
        lbl.setStyleSheet(f'color:{theme.TEXT_MUTED};')
        return lbl

    @staticmethod
    def _divider():
        line = QFrame()
        line.setFixedHeight(1)
        line.setStyleSheet(f'background:{theme.DIVIDER}; border:none;')
        return line

    def _start(self):
        self.node.test_run(self.motor, self.direction.value(),
                           self.speed.value(), self.duration.value())

    def set_end_stops(self, left, right):
        self.led_l.set_state(left)
        self.led_r.set_state(right)

    def set_running(self, running):
        if running:
            self.running_label.setText('EN MARCHA')
            self.running_label.setStyleSheet(f'color:{theme.ACCENT};')
        else:
            self.running_label.setText('DETENIDO')
            self.running_label.setStyleSheet(f'color:{theme.TEXT_MUTED};')

    def set_start_enabled(self, enabled, reason=''):
        self.btn_start.setEnabled(enabled)
        self.btn_start.setToolTip('Mover el motor el tiempo indicado' if enabled else reason)


class CircuitTestTab(QWidget):
    def __init__(self, ros_node, parent=None):
        super().__init__(parent)
        self.node = ros_node

        self.modules = {m: MotorModule(m, ros_node) for m in range(1, NUM_MOTORS + 1)}
        row = QHBoxLayout()
        row.setSpacing(14)
        for m in range(1, NUM_MOTORS + 1):
            row.addWidget(self.modules[m], stretch=1)

        self.btn_stop_all = button('Detener todos', 'danger', 'Detener los 3 motores')
        self.btn_stop_all.clicked.connect(lambda: self.node.test_stop(None))
        self.hint = QLabel('')
        self.hint.setFont(theme.sans(9))
        self.hint.setStyleSheet(f'color:{theme.TEXT_MUTED};')
        bottom = QHBoxLayout()
        bottom.addWidget(self.hint)
        bottom.addStretch()
        bottom.addWidget(self.btn_stop_all)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 20, 24, 20)
        layout.setSpacing(0)
        layout.addLayout(row, stretch=1)
        layout.addSpacing(16)
        layout.addLayout(bottom)

        self.node.signals.limits_received.connect(self._on_limits)
        self.node.signals.status_changed.connect(lambda _s: self._update_enabled())
        self.node.signals.connection_changed.connect(self._on_connection)
        self._on_connection(self.node.connected)

    # ------------------------------------------------------------------
    def _on_limits(self, d):
        for m, module in self.modules.items():
            module.set_end_stops(d.get(f'L{m}'), d.get(f'R{m}'))
            module.set_running(bool(d.get(f'M{m}')))

    def _on_connection(self, connected):
        if not connected:
            for module in self.modules.values():
                module.set_end_stops(None, None)
                module.set_running(False)
        self._update_enabled()

    def _update_enabled(self):
        connected = self.node.connected
        homing = (self.node.last_status or '').startswith('HOMING')
        if not connected:
            reason = 'ESP32 sin conexion'
        elif homing:
            reason = 'Homing en curso: espera a que termine o detenlo'
        else:
            reason = ''
        for module in self.modules.values():
            module.set_start_enabled(connected and not homing, reason)
        self.hint.setText(reason)
