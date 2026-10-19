"""
homing_tab.py - Pestana de homing (web): Start / Resume / Stop e historial.

Mismas reglas que gui/homing_tab.py:
  - STOP siempre habilitado.
  - START solo con el ESP32 conectado.
  - RESUME solo conectado y con el homing detenido (ABORTED) o sin correr (IDLE).
"""
from nicegui import ui

from scara_app.gui import theme as t

from .widgets import LogView, button


class HomingTab:
    def __init__(self, node, hub):
        self.node = node

        with ui.element('div').classes('tab-body'):
            with ui.element('div').classes('btn-row'):
                self.btn_start = button('Start', 'primary', lambda: node.send_command('start'))
                self.btn_resume = button('Resume', 'secondary',
                                         lambda: node.send_command('resume'))
                self.btn_stop = button('Stop', 'danger', lambda: node.send_command('stop'))
            ui.element('div').style('height:24px; flex:none')
            self.log = LogView('Historial')

        hub.on('status', self._on_status)
        hub.on('connection', self._on_connection)

        # Pagina recien abierta: mostrar lo que ya se sabe
        self.log.append('--- ESP32 conectado ---' if node.connected
                        else '--- ESP32 sin conexion ---', t.TEXT_MUTED)
        if node.last_status:
            self._log_status(node.last_status)
        self._update_buttons()

    # ------------------------------------------------------------------
    def _on_status(self, payload):
        status, changed = payload
        if changed:
            self._log_status(status)
            self._update_buttons()

    def _log_status(self, status):
        color = t.DANGER if status.startswith(('ABORTED', 'ERROR')) else t.TEXT
        self.log.append(status, color)

    def _on_connection(self, connected):
        self.log.append('--- ESP32 conectado ---' if connected
                        else '--- ESP32 sin conexion ---', t.TEXT_MUTED)
        self._update_buttons()

    def _update_buttons(self):
        connected = self.node.connected
        status = self.node.last_status or ''
        can_resume = status.startswith('ABORTED') or status == 'IDLE'
        self.btn_start.set_enabled(connected)
        self.btn_resume.set_enabled(connected and can_resume)
        self.btn_stop.set_enabled(True)
