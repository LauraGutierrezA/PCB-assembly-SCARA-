"""
widgets.py - Piezas reutilizables de la interfaz (con el estilo de theme.py).
"""
import html
from datetime import datetime

from PyQt5.QtGui import QTextBlockFormat, QTextCursor
from PyQt5.QtWidgets import (
    QButtonGroup, QHBoxLayout, QLabel, QPushButton, QTextEdit, QVBoxLayout, QWidget,
)

from . import theme

MAX_LOG_LINES = 500


def caps_label(text):
    """Etiqueta pequena en mayusculas (HISTORIAL, RESULTADO, ...)."""
    lbl = QLabel(text.upper())
    lbl.setObjectName('capsLabel')
    lbl.setFont(theme.caps_label_font(7))
    return lbl


def button(text, role, tooltip=''):
    """role: 'primary' | 'secondary' | 'danger' | 'textButton'."""
    b = QPushButton(text)
    b.setObjectName(role)
    if tooltip:
        b.setToolTip(tooltip)
    if role != 'textButton':
        b.setMinimumWidth(94)
    return b


class LogView(QWidget):
    """Historial con hora, encabezado 'HISTORIAL' y boton 'Limpiar historial'."""

    def __init__(self, title='Historial', parent=None):
        super().__init__(parent)
        btn_clear = button('Limpiar historial', 'textButton')
        btn_clear.clicked.connect(self.clear)

        header = QHBoxLayout()
        header.addWidget(caps_label(title))
        header.addStretch()
        header.addWidget(btn_clear)

        self.text = QTextEdit()
        self.text.setObjectName('log')
        self.text.setReadOnly(True)
        self.text.setFont(theme.mono(10))
        self.text.document().setMaximumBlockCount(MAX_LOG_LINES)
        self._empty = True

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)
        layout.addLayout(header)
        layout.addWidget(self.text)

    def append(self, text, color=None):
        color = color or theme.TEXT
        stamp = datetime.now().strftime('%H:%M:%S')
        line = (f'<span style="color:{theme.TEXT_MUTED};">{stamp}</span>'
                f'&nbsp;&nbsp;<span style="color:{color};">{html.escape(text)}</span>')
        cursor = self.text.textCursor()
        cursor.movePosition(QTextCursor.End)
        if not self._empty:
            cursor.insertBlock()
        cursor.insertHtml(line)
        fmt = QTextBlockFormat()
        fmt.setLineHeight(160, QTextBlockFormat.ProportionalHeight)
        cursor.mergeBlockFormat(fmt)
        self._empty = False
        self.text.setTextCursor(cursor)
        self.text.ensureCursorVisible()

    def clear(self):
        self.text.clear()
        self._empty = True

    def to_plain_text(self):
        return self.text.toPlainText()


class SegmentedToggle(QWidget):
    """Selector de opciones excluyentes (p. ej. Codo +1 / -1)."""

    def __init__(self, options, default=0, parent=None):
        """options: lista de (texto, valor)."""
        super().__init__(parent)
        self._group = QButtonGroup(self)
        self._group.setExclusive(True)
        self._values = {}
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(0)
        for i, (text, value) in enumerate(options):
            b = QPushButton(text)
            b.setCheckable(True)
            b.setObjectName('segment')
            b.setProperty('position', 'first' if i == 0 else
                          ('last' if i == len(options) - 1 else 'middle'))
            self._group.addButton(b, i)
            self._values[i] = value
            row.addWidget(b)
        self._group.button(default).setChecked(True)

    def value(self):
        return self._values[self._group.checkedId()]

    def set_index(self, index):
        self._group.button(index).setChecked(True)
