"""
widgets.py - Piezas reutilizables de la interfaz web (estilo en web/theme.py).

Equivalentes de gui/widgets.py: caps_label, button, LogView, SegmentedToggle.
"""
from collections import defaultdict
from datetime import datetime

from nicegui import ui

from scara_app.gui import theme as t

MAX_LOG_LINES = 500


def caps_label(text):
    """Etiqueta pequena en mayusculas (HISTORIAL, CODO, ...)."""
    return ui.label(text.upper()).classes('caps')


def button(text, role, on_click=None):
    """role: 'primary' | 'secondary' | 'danger' | 'text'."""
    return (ui.button(text, on_click=on_click, color=None)
            .props('unelevated no-caps')
            .classes(f'btn btn-{role}'))


def number_box(value, kind, step=1.0, suffix=None, minimum=None, maximum=None):
    """Campo numerico. kind: 'coord' (grande) | 'param' (pequeno)."""
    box = ui.number(value=value, format='%.1f', step=step, suffix=suffix,
                    min=minimum, max=maximum)
    return box.props('outlined dense hide-bottom-space').classes(f'num num-{kind}')


def deadman_button(source):
    """Boton "mantener para mover". source: nombre de la pestana ('p2p', 'dk').
    Toda la logica de presionar/soltar esta en JavaScript (web/theme.py);
    el servidor recibe los eventos 'deadman' (web/app.py)."""
    idle = 'Mantener para mover  (Espacio)'
    with ui.element('div').classes('deadman') as el:
        ui.label(idle).classes('deadman-text')
    el._props.update({'data-dm': source, 'data-idle': idle,
                      'data-held': 'Moviendo... suelta para detener'})
    el.update()
    return el


def read_number(box, default=None):
    """Valor de un ui.number; default si esta vacio."""
    return default if box.value is None else float(box.value)


class Hub:
    """Reparte los eventos del nodo dentro de UNA pagina (una por pantalla abierta)."""

    def __init__(self):
        self._handlers = defaultdict(list)

    def on(self, kind, handler):
        self._handlers[kind].append(handler)

    def emit(self, kind, payload):
        for handler in self._handlers[kind]:
            handler(payload)


class LogView:
    """Historial con hora, titulo y boton 'Limpiar historial'.

    Las lineas nuevas se insertan al principio de un contenedor con
    flex-direction: column-reverse: se ven abajo y el scroll se queda pegado
    al final sin JavaScript.
    """

    def __init__(self, title='Historial'):
        with ui.element('div').classes('log-wrap'):
            with ui.element('div').classes('log-header'):
                caps_label(title)
                button('Limpiar historial', 'text', on_click=self.clear)
            self._box = ui.element('div').classes('log')
        self._count = 0

    def append(self, text, color=None):
        stamp = datetime.now().strftime('%H:%M:%S')
        with self._box:
            with ui.element('div').classes('log-line') as line:
                ui.label(stamp).classes('log-stamp')
                ui.label(text).classes('log-text').style(f'color:{color or t.TEXT}')
        line.move(target_index=0)
        self._count += 1
        if self._count > MAX_LOG_LINES:
            self._box.remove(self._box.default_slot.children[-1])
            self._count -= 1

    def clear(self):
        self._box.clear()
        self._count = 0


class SegmentedToggle:
    """Selector de opciones excluyentes (p. ej. Codo +1 / -1)."""

    def __init__(self, options, default=0, on_change=None):
        """options: lista de (texto, valor). on_change(valor): al tocar una opcion."""
        self._values = [value for _, value in options]
        self._on_change = on_change
        self._index = default
        self._buttons = []
        with ui.element('div').classes('segmented'):
            for i, (text, _) in enumerate(options):
                b = (ui.button(text, on_click=self._selector(i), color=None)
                     .props('unelevated no-caps').classes('segment'))
                self._buttons.append(b)
        self._render()

    def _selector(self, index):
        def select():
            self.set_index(index)
            if self._on_change is not None:
                self._on_change(self.value())
        return select

    def value(self):
        return self._values[self._index]

    def set_index(self, index):
        self._index = index
        self._render()

    def _render(self):
        for i, b in enumerate(self._buttons):
            if i == self._index:
                b.classes(add='on')
            else:
                b.classes(remove='on')
