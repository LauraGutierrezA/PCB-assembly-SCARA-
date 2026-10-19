"""
app.py - Pagina principal de la interfaz web del SCARA.

  ┌──────────────────────────────────────────────────────────┐
  │ ● ESP32 conectado              ESTADO ACTUAL     [ STOP ]│  <- siempre visible
  │                                HOMED                     │
  ├──────────────────────────────────────────────────────────┤
  │  Homing   Punto a punto   Prueba de circuito             │
  │                                                          │
  └──────────────────────────────────────────────────────────┘

El boton STOP del encabezado reemplaza la tecla Esc (el iPad no tiene).
Con teclado (PC o iPad con teclado), Esc sigue funcionando.

Dead-man: el navegador manda eventos 'deadman' (JavaScript en web/theme.py)
mientras se mantiene el boton de DK / Punto a punto o la barra espaciadora.
Aqui se pasan al ESP32 ("dm1" / "dm0"). Al empezar a mantener, la pestana
manda primero su consigna (on_deadman_start). Al cambiar de pestana se
suelta. Al entrar a "Prueba de circuito" se borra la consigna del firmware.

Cada pantalla que abre la pagina (iPad, PC, celular) tiene su propia copia
de la interfaz; todas mandan los comandos al mismo nodo ROS 2.

Para agregar una pestana nueva:
  1. Crear web/<nombre>_tab.py con una clase que reciba (node, hub) y
     escuche los eventos con hub.on('status' | 'connection' | 'ik' | 'limits', fn).
  2. Agregarla en TABS.
"""
from nicegui import ui

from scara_app.gui import theme as t

from . import theme
from .circuit_test_tab import CircuitTestTab
from .dk_tab import DkTab
from .homing_tab import HomingTab
from .point_to_point_tab import PointToPointTab
from .widgets import Hub, button, caps_label

POLL_PERIOD_S = 0.05   # cada cuanto la pagina revisa eventos nuevos del nodo

TABS = [
    ('homing', 'Homing', HomingTab),
    ('p2p', 'Punto a punto', PointToPointTab),
    ('dk', 'DK', DkTab),
    ('circuit', 'Prueba de circuito', CircuitTestTab),
    # Aqui iran: 'Jog', 'Caracterizacion', ...
]


def build_ui(node):
    @ui.page('/')
    def index():
        ui.add_head_html(theme.HEAD_HTML + theme.css())
        ui.colors(primary=t.PRIMARY, accent=t.ACCENT, negative=t.DANGER, dark=t.BG_CARD)
        hub = Hub()

        with ui.element('div').classes('card'):
            _header(node, hub)

            with ui.tabs().props('no-caps align=left narrow-indicator').classes('tabs') as tabs:
                tab_items = [ui.tab(name, label=label) for name, label, _ in TABS]
            pages = {}
            with ui.tab_panels(tabs, value=tab_items[0], animated=False).classes('panels'):
                for tab_item, (name, _, tab_class) in zip(tab_items, TABS):
                    with ui.tab_panel(tab_item):
                        pages[name] = tab_class(node, hub)

        # ---------- Dead-man ----------
        def release_deadman():
            ui.run_javascript('window.scaraDeadman && window.scaraDeadman.releaseAll()')

        def on_deadman(e):
            args = e.args or {}
            held = bool(args.get('held'))
            if held and args.get('start'):
                page = pages.get(args.get('source'))
                if page is not None and hasattr(page, 'on_deadman_start'):
                    page.on_deadman_start()      # primero la consigna
            if held:
                node.deadman_beat()
            else:
                node.deadman_release()
            for page in pages.values():
                if hasattr(page, 'set_held'):
                    page.set_held(held)
        ui.on('deadman', on_deadman)

        def on_tab_change(e):
            release_deadman()
            node.deadman_release()
            name = getattr(e.value, '_props', {}).get('name', e.value)
            if name == 'circuit':
                node.release_control()
        tabs.on_value_change(on_tab_change)

        # Esc = STOP (si hay teclado), tambien escribiendo en un campo
        def on_key(e):
            if e.key == 'Escape' and e.action.keydown and not e.action.repeat:
                release_deadman()
                node.emergency_stop()
        ui.keyboard(on_key=on_key, ignore=[])

        # Pasar los eventos nuevos del nodo a esta pagina
        last_seq = node.latest_seq

        def poll():
            nonlocal last_seq
            last_seq, events = node.events_since(last_seq)
            for kind, payload in events:
                hub.emit(kind, payload)
        ui.timer(POLL_PERIOD_S, poll)


def _header(node, hub):
    with ui.element('div').classes('header'):
        with ui.element('div').classes('conn'):
            dot = ui.label('●').classes('conn-dot')
            conn_text = ui.label()
        ui.element('div').classes('grow')
        with ui.element('div').classes('status-box'):
            caps_label('Estado actual')
            status = ui.label('—').classes('status-value')

        def stop():
            ui.run_javascript('window.scaraDeadman && window.scaraDeadman.releaseAll()')
            node.emergency_stop()
        button('STOP', 'danger', stop).classes('btn-estop')

    def on_connection(connected):
        dot.style(f'color:{t.CONNECTED if connected else t.DISCONNECTED}')
        conn_text.set_text('ESP32 conectado' if connected else 'ESP32 sin conexion')
        if not connected:
            status.set_text('—')

    hub.on('connection', on_connection)
    hub.on('status', lambda payload: status.set_text(payload[0]))
    on_connection(node.connected)
    if node.connected and node.last_status:
        status.set_text(node.last_status)
