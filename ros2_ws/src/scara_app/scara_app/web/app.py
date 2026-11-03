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
La pestana "Wiimote" tiene su propio dead-man (el boton B del Wiimote, que esta
en el PC, no en el navegador; ver wiimote_bridge.py): el control con Wiimote se
desactiva al salir de esa pestana, al cerrarse la pagina y con STOP / Esc.

Cada pantalla que abre la pagina (iPad, PC, celular) tiene su propia copia
de la interfaz; todas mandan los comandos al mismo nodo ROS 2.

OBSERVADORES (bypass): entrar a la pestana "Observadores" NO hace nada de lo
anterior (no suelta el dead-man, no cancela la Trayectoria, no desactiva el
Wiimote). La pestana desde la que se llego queda como "duena" (owner): al
salir de Observadores hacia otra pestana se aplica la regla normal como si se
saliera de la duena; si se vuelve a la duena, todo sigue igual. El dead-man
de Observadores ('obs') se reenvia a la pestana duena.

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
from .observers_tab import ObserversTab     # OBSERVADORES
from .point_to_point_tab import PointToPointTab
from .trajectory_tab import TrajectoryTab   # TRAYECTORIA
from .widgets import Hub, button, caps_label
from .wiimote_tab import WiimoteTab

POLL_PERIOD_S = 0.05   # cada cuanto la pagina revisa eventos nuevos del nodo
OBS_TAB = 'obs'        # OBSERVADORES: pestana con bypass (no detiene nada al entrar)

TABS = [
    ('homing', 'Homing', HomingTab),
    ('p2p', 'Punto a punto', PointToPointTab),
    ('dk', 'DK', DkTab),
    ('traj', 'Trayectoria', TrajectoryTab),   # TRAYECTORIA
    ('circuit', 'Prueba de circuito', CircuitTestTab),
    ('wiimote', 'Wiimote', WiimoteTab),
    ('obs', 'Observadores', ObserversTab),    # OBSERVADORES
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
            source = args.get('source')
            if source == OBS_TAB:                # OBSERVADORES: es el de la duena
                source = current['owner']
            if held and args.get('start'):
                page = pages.get(source)
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

        # tab: pestana que esta viendo ESTA pagina
        # owner: OBSERVADORES, ultima pestana que no es Observadores (la que
        #        sigue "mandando" mientras se ven las graficas)
        current = {'tab': TABS[0][0], 'owner': TABS[0][0]}

        def disarm_wiimote_if_viewing():
            if current['owner'] == 'wiimote':
                node.wiimote.set_enabled(False)

        def on_tab_change(e):
            name = getattr(e.value, '_props', {}).get('name', e.value)
            prev = current['tab']
            current['tab'] = name
            for page in pages.values():          # OBSERVADORES: que pestana se ve
                if hasattr(page, 'on_view_change'):
                    page.on_view_change(name)
            # OBSERVADORES (bypass): entrar a Observadores no detiene nada
            if name == OBS_TAB:
                pages[OBS_TAB].set_owner(current['owner'])
                return
            # Volver de Observadores a la duena: todo sigue igual
            if prev == OBS_TAB and name == current['owner']:
                return
            release_deadman()
            node.deadman_release()
            for page in pages.values():          # TRAYECTORIA: aborta el ciclo
                if hasattr(page, 'on_tab_change'):
                    page.on_tab_change(name)
            if name != 'wiimote':
                disarm_wiimote_if_viewing()
            current['owner'] = name
            if name == 'circuit':
                node.release_control()
        tabs.on_value_change(on_tab_change)

        # Si se cierra o se duerme esta pantalla en la pestana Wiimote, se desactiva.
        try:
            ui.context.client.on_disconnect(disarm_wiimote_if_viewing)
        except AttributeError:
            pass

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
