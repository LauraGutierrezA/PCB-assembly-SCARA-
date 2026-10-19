"""
main_window.py - Ventana principal de la interfaz del SCARA.

Estructura:
  ┌───────────────────────────────────────────────┐
  │ ● ESP32 conectado              ESTADO ACTUAL  │  <- encabezado (siempre visible)
  │                           ABORTED M1 SEEK_L   │
  ├───────────────────────────────────────────────┤
  │  Homing   [ ... futuras pestanas ... ]        │  <- una pestana por funcion
  │                                               │
  └───────────────────────────────────────────────┘

Dead-man switch: un solo objeto Deadman para toda la ventana (widgets.py).
Las pestanas DK y Punto a punto tienen su boton "Mantener para mover"; la
barra espaciadora hace lo mismo en esas pestanas (eventFilter). Al cambiar
de pestana o al salir de la ventana, el dead-man se suelta.
Al entrar a "Prueba de circuito" se borra la consigna del firmware
("release"), para que el controlador no devuelva el motor a la consigna
vieja despues de cada prueba.

Para agregar una pestana nueva (p. ej. punto a punto):
  1. Crear gui/point_to_point_tab.py con una clase QWidget que reciba ros_node.
  2. Agregarla en _build_tabs() con self.tabs.addTab(...).
Los estilos salen de theme.py (objectName de cada widget).
"""
from PyQt5.QtCore import QEvent, Qt
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import (
    QApplication, QFrame, QHBoxLayout, QLabel, QMainWindow, QShortcut, QTabWidget,
    QVBoxLayout, QWidget,
)

from . import theme
from .homing_tab import HomingTab
from .point_to_point_tab import PointToPointTab
from .dk_tab import DkTab
from .circuit_test_tab import CircuitTestTab
from .widgets import Deadman


class MainWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.node = ros_node
        self.deadman = Deadman(ros_node, self)
        self.setWindowTitle('SCARA RRP - Estacion 3')
        self.resize(1240, 820)
        self.setFont(theme.sans(10))
        self.setStyleSheet(theme.stylesheet())

        page = QWidget()
        page.setObjectName('page')
        page_layout = QVBoxLayout(page)
        page_layout.setContentsMargins(4, 4, 4, 4)

        card = QFrame()
        card.setObjectName('card')
        card_layout = QVBoxLayout(card)
        card_layout.setContentsMargins(0, 0, 0, 0)
        card_layout.setSpacing(0)

        card_layout.addWidget(self._build_header())
        divider = QFrame()
        divider.setObjectName('divider')
        card_layout.addWidget(divider)

        self.tabs = QTabWidget()
        self._build_tabs()
        card_layout.addWidget(self.tabs)

        page_layout.addWidget(card)
        self.setCentralWidget(page)

        # Esc = STOP desde cualquier parte de la ventana (suelta tambien el dead-man)
        QShortcut(QKeySequence(Qt.Key_Escape), self, activated=self._emergency_stop)

        # Barra espaciadora = dead-man en DK y Punto a punto
        QApplication.instance().installEventFilter(self)
        # Si la ventana pierde el foco (otra app, minimizar), se suelta
        QApplication.instance().applicationStateChanged.connect(
            lambda state: state == Qt.ApplicationActive or self.deadman.release_all())
        self.tabs.currentChanged.connect(self._on_tab_changed)

        self.node.signals.status_received.connect(self._on_status)
        self.node.signals.connection_changed.connect(self._on_connection)
        self._on_connection(False)

    # ------------------------------------------------------------------
    def _build_tabs(self):
        self.tabs.addTab(HomingTab(self.node), 'Homing')
        self.tabs.addTab(PointToPointTab(self.node, self.deadman), 'Punto a punto')
        self.tabs.addTab(DkTab(self.node, self.deadman), 'DK')
        self.circuit_tab = CircuitTestTab(self.node)
        self.tabs.addTab(self.circuit_tab, 'Prueba de circuito')
        # Aqui iran: 'Jog', 'Caracterizacion', ...

    def _build_header(self):
        header = QWidget()
        header.setObjectName('header')
        row = QHBoxLayout(header)
        row.setContentsMargins(24, 18, 24, 18)

        # Izquierda: punto + texto de conexion
        self.conn_dot = QLabel('●')
        self.conn_dot.setFont(theme.sans(9))
        self.conn_label = QLabel()
        self.conn_label.setFont(theme.mono(10))
        row.addWidget(self.conn_dot)
        row.addSpacing(4)
        row.addWidget(self.conn_label)
        row.addStretch()

        # Derecha: etiqueta + estado
        right = QVBoxLayout()
        right.setSpacing(2)
        caps = QLabel('ESTADO ACTUAL')
        caps.setObjectName('capsLabel')
        caps.setFont(theme.caps_label_font(7))
        caps.setAlignment(Qt.AlignRight)
        self.status_label = QLabel('—')
        self.status_label.setObjectName('statusValue')
        self.status_label.setFont(theme.mono(17))
        self.status_label.setAlignment(Qt.AlignRight)
        right.addWidget(caps)
        right.addWidget(self.status_label)
        row.addLayout(right)
        return header

    def _emergency_stop(self):
        self.deadman.release_all()
        self.node.send_command('stop')

    def _on_tab_changed(self, _index):
        self.deadman.release_all()
        if self.tabs.currentWidget() is self.circuit_tab:
            self.node.release_control()

    def eventFilter(self, obj, event):
        """Barra espaciadora como dead-man en las pestanas que tienen boton."""
        if event.type() in (QEvent.KeyPress, QEvent.KeyRelease) and event.key() == Qt.Key_Space:
            button = getattr(self.tabs.currentWidget(), 'deadman_button', None)
            if button is None or not self.isActiveWindow():
                return False
            if not event.isAutoRepeat():
                if event.type() == QEvent.KeyPress:
                    button.press_space()
                else:
                    button.release_space()
            return True     # que la barra no "haga clic" en el boton con foco
        return super().eventFilter(obj, event)

    def _on_status(self, status):
        self.status_label.setText(status)

    def _on_connection(self, connected):
        color = theme.CONNECTED if connected else theme.DISCONNECTED
        self.conn_dot.setStyleSheet(f'color:{color};')
        self.conn_label.setText('ESP32 conectado' if connected else 'ESP32 sin conexion')
        if not connected:
            self.status_label.setText('—')
