"""
widgets.py - Piezas reutilizables de la interfaz (con el estilo de theme.py).
"""
import html
from datetime import datetime

from PyQt5.QtCore import QObject, QTimer, pyqtSignal
from PyQt5.QtGui import QTextBlockFormat, QTextCursor
from PyQt5.QtWidgets import (
    QButtonGroup, QHBoxLayout, QLabel, QPushButton, QTextEdit, QVBoxLayout, QWidget,
)

from scara_app.robot_model import DEADMAN_PERIOD_S

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

    def connect_changed(self, fn):
        """fn(valor): se llama cuando el usuario toca una opcion."""
        self._group.buttonClicked.connect(lambda _b: fn(self.value()))

    def set_index(self, index):
        self._group.button(index).setChecked(True)


class Deadman(QObject):
    """Dead-man switch de la ventana (uno solo para todas las pestanas).

    Varias "fuentes" pueden mantenerlo: el boton de cada pestana y la barra
    espaciadora. Mientras haya al menos una, manda un latido ("dm1") cada
    DEADMAN_PERIOD_S; al soltar la ultima manda "dm0" y el firmware detiene
    los motores. Si la interfaz se congela, el QTimer deja de latir y el
    firmware tambien se detiene (300 ms sin latido).
    """
    held_changed = pyqtSignal(bool)

    def __init__(self, ros_node, parent=None):
        super().__init__(parent)
        self.node = ros_node
        self._sources = set()
        self._timer = QTimer(self)
        self._timer.setInterval(int(DEADMAN_PERIOD_S * 1000))
        self._timer.timeout.connect(self.node.deadman_beat)

    @property
    def held(self):
        return bool(self._sources)

    def press(self, source, on_start=None):
        """on_start: se llama al empezar a mantener (p. ej. para mandar la consigna)."""
        was_held = self.held
        self._sources.add(source)
        if not was_held:
            if on_start is not None:
                on_start()
            self.node.deadman_beat()
            self._timer.start()
            self.held_changed.emit(True)

    def release(self, source):
        if source not in self._sources:
            return
        self._sources.discard(source)
        if not self._sources:
            self._stop()

    def release_all(self):
        if self._sources:
            self._sources.clear()
            self._stop()

    def _stop(self):
        self._timer.stop()
        self.node.deadman_release()
        self.held_changed.emit(False)


class DeadmanButton(QPushButton):
    """Boton "mantener para mover". Al presionarlo llama on_start (mandar la
    consigna) y mantiene el dead-man hasta soltarlo. La barra espaciadora
    hace lo mismo (main_window.py la redirige a press_space/release_space)."""

    IDLE_TEXT = 'Mantener para mover  (Espacio)'
    HELD_TEXT = 'Moviendo... suelta para detener'

    def __init__(self, deadman, on_start, parent=None):
        super().__init__(self.IDLE_TEXT, parent)
        self.setObjectName('deadman')
        self.setProperty('held', False)
        self.setMinimumHeight(64)
        self.deadman = deadman
        self.on_start = on_start
        self.pressed.connect(lambda: self.deadman.press(self, self.on_start))
        self.released.connect(lambda: self.deadman.release(self))
        self.deadman.held_changed.connect(self._show_held)

    def press_space(self):
        self.deadman.press('space', self.on_start)

    def release_space(self):
        self.deadman.release('space')

    def _show_held(self, held):
        self.setText(self.HELD_TEXT if held else self.IDLE_TEXT)
        self.setProperty('held', held)
        self.style().unpolish(self)
        self.style().polish(self)
