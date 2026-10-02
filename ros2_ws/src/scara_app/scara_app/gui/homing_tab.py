"""
homing_tab.py - Pestana de homing: botones Start / Resume / Stop e historial.

Reglas de los botones:
  - STOP siempre esta habilitado (seguridad). Atajo: tecla Esc desde
    cualquier parte de la ventana (definido en main_window.py).
  - START solo con el ESP32 conectado.
  - RESUME solo con el ESP32 conectado y si el homing esta detenido
    (ABORTED) o nunca se ha corrido (IDLE).
"""
from PyQt5.QtWidgets import QHBoxLayout, QVBoxLayout, QWidget

from . import theme
from .widgets import LogView, button


class HomingTab(QWidget):
    def __init__(self, ros_node, parent=None):
        super().__init__(parent)
        self.node = ros_node

        # ---------- Botones ----------
        self.btn_start = button('Start', 'primary', 'Homing desde el motor 1')
        self.btn_resume = button('Resume', 'secondary', 'Continuar donde quedo')
        self.btn_stop = button('Stop', 'danger', 'Detener todo (Esc)')

        self.btn_start.clicked.connect(lambda: self.node.send_command('start'))
        self.btn_resume.clicked.connect(lambda: self.node.send_command('resume'))
        self.btn_stop.clicked.connect(lambda: self.node.send_command('stop'))

        buttons = QHBoxLayout()
        buttons.setSpacing(12)
        buttons.addWidget(self.btn_start)
        buttons.addWidget(self.btn_resume)
        buttons.addWidget(self.btn_stop)
        buttons.addStretch()

        # ---------- Historial ----------
        self.log = LogView('Historial')

        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 24, 24, 24)
        layout.setSpacing(0)
        layout.addLayout(buttons)
        layout.addSpacing(28)
        layout.addWidget(self.log)

        # ---------- Senales del nodo ----------
        self.node.signals.status_changed.connect(self._on_status_changed)
        self.node.signals.connection_changed.connect(self._on_connection_changed)
        self._update_buttons()

    # ------------------------------------------------------------------
    def _on_status_changed(self, status):
        color = theme.DANGER if status.startswith('ABORTED') else theme.TEXT
        self.log.append(status, color)
        self._update_buttons()

    def _on_connection_changed(self, connected):
        text = '--- ESP32 conectado ---' if connected else '--- ESP32 sin conexion ---'
        self.log.append(text, theme.TEXT_MUTED)
        self._update_buttons()

    def _update_buttons(self):
        connected = self.node.connected
        status = self.node.last_status or ''
        can_resume = status.startswith('ABORTED') or status == 'IDLE'

        self.btn_start.setEnabled(connected)
        self.btn_resume.setEnabled(connected and can_resume)
        self.btn_stop.setEnabled(True)
