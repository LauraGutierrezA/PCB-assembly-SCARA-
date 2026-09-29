"""
homing_tab.py - Pestana de homing: botones Start / Resume / Stop e historial.

Reglas de los botones:
  - STOP siempre esta habilitado (seguridad). Atajo: tecla Esc desde
    cualquier parte de la ventana (definido en main_window.py).
  - START solo con el ESP32 conectado.
  - RESUME solo con el ESP32 conectado y si el homing esta detenido
    (ABORTED) o nunca se ha corrido (IDLE).
"""
import html
from datetime import datetime

from PyQt5.QtGui import QTextBlockFormat, QTextCursor
from PyQt5.QtWidgets import (
    QHBoxLayout, QLabel, QPushButton, QTextEdit, QVBoxLayout, QWidget,
)

from . import theme

MAX_LOG_LINES = 500


class HomingTab(QWidget):
    def __init__(self, ros_node, parent=None):
        super().__init__(parent)
        self.node = ros_node

        # ---------- Botones ----------
        self.btn_start = self._make_button('Start', 'primary', 'Homing desde el motor 1')
        self.btn_resume = self._make_button('Resume', 'secondary', 'Continuar donde quedo')
        self.btn_stop = self._make_button('Stop', 'danger', 'Detener todo (Esc)')

        self.btn_start.clicked.connect(lambda: self.node.send_command('start'))
        self.btn_resume.clicked.connect(lambda: self.node.send_command('resume'))
        self.btn_stop.clicked.connect(lambda: self.node.send_command('stop'))

        buttons = QHBoxLayout()
        buttons.setSpacing(12)
        buttons.addWidget(self.btn_start)
        buttons.addWidget(self.btn_resume)
        buttons.addWidget(self.btn_stop)
        buttons.addStretch()

        # ---------- Encabezado del historial ----------
        caps = QLabel('HISTORIAL')
        caps.setObjectName('capsLabel')
        caps.setFont(theme.caps_label_font(7))
        btn_clear = QPushButton('Limpiar historial')
        btn_clear.setObjectName('textButton')
        btn_clear.clicked.connect(self._clear_log)

        log_header = QHBoxLayout()
        log_header.addWidget(caps)
        log_header.addStretch()
        log_header.addWidget(btn_clear)

        # ---------- Historial ----------
        self.log = QTextEdit()
        self.log.setObjectName('log')
        self.log.setReadOnly(True)
        self.log.setFont(theme.mono(10))
        self.log.document().setMaximumBlockCount(MAX_LOG_LINES)
        self._log_empty = True

        # ---------- Layout ----------
        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 24, 24, 24)
        layout.setSpacing(0)
        layout.addLayout(buttons)
        layout.addSpacing(28)
        layout.addLayout(log_header)
        layout.addSpacing(10)
        layout.addWidget(self.log)

        # ---------- Senales del nodo ----------
        self.node.signals.status_changed.connect(self._on_status_changed)
        self.node.signals.connection_changed.connect(self._on_connection_changed)
        self._update_buttons()

    # ------------------------------------------------------------------
    @staticmethod
    def _make_button(text, role, tooltip):
        b = QPushButton(text)
        b.setObjectName(role)
        b.setToolTip(tooltip)
        b.setMinimumWidth(94)
        return b

    def _append_log(self, text, color):
        stamp = datetime.now().strftime('%H:%M:%S')
        line = (f'<span style="color:{theme.TEXT_MUTED};">{stamp}</span>'
                f'&nbsp;&nbsp;<span style="color:{color};">{html.escape(text)}</span>')

        cursor = self.log.textCursor()
        cursor.movePosition(QTextCursor.End)
        if not self._log_empty:
            cursor.insertBlock()
        cursor.insertHtml(line)
        fmt = QTextBlockFormat()
        fmt.setLineHeight(160, QTextBlockFormat.ProportionalHeight)
        cursor.mergeBlockFormat(fmt)
        self._log_empty = False
        self.log.setTextCursor(cursor)
        self.log.ensureCursorVisible()

    def _clear_log(self):
        self.log.clear()
        self._log_empty = True

    def _on_status_changed(self, status):
        color = theme.DANGER if status.startswith('ABORTED') else theme.TEXT
        self._append_log(status, color)
        self._update_buttons()

    def _on_connection_changed(self, connected):
        text = '--- ESP32 conectado ---' if connected else '--- ESP32 sin conexion ---'
        self._append_log(text, theme.TEXT_MUTED)
        self._update_buttons()

    def _update_buttons(self):
        connected = self.node.connected
        status = self.node.last_status or ''
        can_resume = status.startswith('ABORTED') or status == 'IDLE'

        self.btn_start.setEnabled(connected)
        self.btn_resume.setEnabled(connected and can_resume)
        self.btn_stop.setEnabled(True)
