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

Para agregar una pestana nueva (p. ej. punto a punto):
  1. Crear gui/point_to_point_tab.py con una clase QWidget que reciba ros_node.
  2. Agregarla en _build_tabs() con self.tabs.addTab(...).
Los estilos salen de theme.py (objectName de cada widget).
"""
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import (
    QFrame, QHBoxLayout, QLabel, QMainWindow, QShortcut, QTabWidget,
    QVBoxLayout, QWidget,
)

from . import theme
from .homing_tab import HomingTab


class MainWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.node = ros_node
        self.setWindowTitle('SCARA RRP - Estacion 3')
        self.resize(700, 560)
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

        # Esc = STOP desde cualquier parte de la ventana
        QShortcut(QKeySequence(Qt.Key_Escape), self,
                  activated=lambda: self.node.send_command('stop'))

        self.node.signals.status_received.connect(self._on_status)
        self.node.signals.connection_changed.connect(self._on_connection)
        self._on_connection(False)

    # ------------------------------------------------------------------
    def _build_tabs(self):
        self.tabs.addTab(HomingTab(self.node), 'Homing')
        # Aqui iran: 'Punto a punto', 'Jog', 'Caracterizacion', ...

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

    def _on_status(self, status):
        self.status_label.setText(status)

    def _on_connection(self, connected):
        color = theme.CONNECTED if connected else theme.DISCONNECTED
        self.conn_dot.setStyleSheet(f'color:{color};')
        self.conn_label.setText('ESP32 conectado' if connected else 'ESP32 sin conexion')
        if not connected:
            self.status_label.setText('—')
