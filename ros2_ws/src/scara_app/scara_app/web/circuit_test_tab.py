"""
circuit_test_tab.py - Pestana "Prueba de circuito" (web).

Igual que gui/circuit_test_tab.py: un modulo por motor con end-stops,
encoder y accionamiento. Para que quepa en el iPad, cada modulo es mas
compacto: end-stops y encoder van en la misma fila, y sentido y duracion
tambien.

Constantes (velocidades, nombres de los finales, unidades) importadas de la
version de escritorio.
"""
from nicegui import ui

from scara_app.gui.circuit_test_tab import (
    DEFAULT_DURATION_S, DEFAULT_SPEED_PCT, END_STOP_NAMES, NUM_MOTORS, POSITION_UNITS,
)

from .widgets import SegmentedToggle, button, caps_label, number_box, read_number


class EndStopLed:
    def __init__(self, letter):
        self.label = ui.label(letter).classes('led')

    def set_state(self, pressed):
        """pressed: True, False o None (sin datos)."""
        self.label.classes(remove='on off')
        if pressed is True:
            self.label.classes(add='on')
        elif pressed is False:
            self.label.classes(add='off')


class MotorModule:
    def __init__(self, motor, node):
        self.motor = motor
        self.node = node
        name_l, name_r = END_STOP_NAMES.get(motor, ('L', 'R'))

        with ui.element('div').classes('panel'):
            with ui.element('div').classes('panel-head'):
                ui.label(f'MOTOR {motor}').classes('panel-title')
                self.running_label = ui.label('DETENIDO').classes('caps')

            # ---------- End-stops | Encoder ----------
            with ui.element('div').classes('panel-row'):
                with ui.element('div').classes('field'):
                    caps_label('End-stops')
                    with ui.element('div').classes('leds'):
                        self.led_l = EndStopLed(name_l)
                        self.led_r = EndStopLed(name_r)
                with ui.element('div').classes('field').style('align-items:flex-end'):
                    caps_label('Encoder')
                    self.enc_counts = ui.label('—').classes('enc-counts')
                    self.enc_pos = ui.label('—').classes('enc-pos')

            ui.element('div').classes('panel-divider')

            # ---------- Accionamiento ----------
            caps_label('Accionamiento')
            with ui.element('div').classes('panel-row'):
                with ui.element('div').classes('field'):
                    ui.label('Sentido').classes('small')
                    self.direction = SegmentedToggle(
                        [(f'→ {name_l}', 'L'), (f'→ {name_r}', 'R')], default=0)
                with ui.element('div').classes('field'):
                    ui.label('Duracion').classes('small')
                    self.duration = number_box(DEFAULT_DURATION_S, 'param', step=0.1,
                                               suffix='s', minimum=0.1, maximum=60.0)
            with ui.element('div').classes('field'):
                ui.label('Velocidad').classes('small')
                with ui.element('div').classes('speed-row'):
                    self.speed = (ui.slider(min=0, max=100, step=1,
                                            value=DEFAULT_SPEED_PCT.get(motor, 30))
                                  .props('thumb-size=28px track-size=6px')
                                  .classes('slider grow'))
                    ui.label().classes('speed-value').bind_text_from(
                        self.speed, 'value', lambda v: f'{int(v)} %')

            with ui.element('div').classes('btn-row'):
                self.btn_start = button('Start', 'primary', self._start)
                button('Stop', 'danger', lambda: self.node.test_stop(self.motor))

    # ------------------------------------------------------------------
    def _start(self):
        seconds = read_number(self.duration, 0.0)
        if seconds <= 0:
            return
        seconds = min(seconds, 60.0)
        self.node.test_run(self.motor, self.direction.value(), self.speed.value, seconds)

    def set_end_stops(self, left, right):
        self.led_l.set_state(left)
        self.led_r.set_state(right)

    def set_encoder(self, counts, position):
        unit = POSITION_UNITS.get(self.motor, '°')
        self.enc_counts.set_text('—' if counts is None else f'{counts} cts')
        self.enc_pos.set_text('—' if position is None else f'{position:.1f}{unit}')

    def set_running(self, running):
        self.running_label.set_text('EN MARCHA' if running else 'DETENIDO')
        self.running_label.style('color: var(--scara-accent)' if running else '')

    def set_start_enabled(self, enabled):
        self.btn_start.set_enabled(enabled)


class CircuitTestTab:
    def __init__(self, node, hub):
        self.node = node

        with ui.element('div').classes('tab-body'):
            with ui.element('div').classes('motors'):
                self.modules = {m: MotorModule(m, node) for m in range(1, NUM_MOTORS + 1)}
            with ui.element('div').classes('circuit-bottom'):
                self.hint = ui.label('').classes('hint')
                button('Detener todos', 'danger', lambda: node.test_stop(None))

        hub.on('limits', self._on_limits)
        hub.on('status', lambda _payload: self._update_enabled())
        hub.on('connection', self._on_connection)
        self._on_connection(node.connected)
        if node.last_limits:
            self._on_limits(node.last_limits)

    # ------------------------------------------------------------------
    def _on_limits(self, d):
        for m, module in self.modules.items():
            module.set_end_stops(d.get(f'L{m}'), d.get(f'R{m}'))
            module.set_running(bool(d.get(f'M{m}')))
            position = d.get(f'Q{m}') if d.get('CAL') else None
            module.set_encoder(d.get(f'E{m}'), position)

    def _on_connection(self, connected):
        if not connected:
            for module in self.modules.values():
                module.set_end_stops(None, None)
                module.set_running(False)
                module.set_encoder(None, None)
        self._update_enabled()

    def _update_enabled(self):
        connected = self.node.connected
        homing = (self.node.last_status or '').startswith('HOMING')
        if not connected:
            reason = 'ESP32 sin conexion'
        elif homing:
            reason = 'Homing en curso: espera a que termine o detenlo'
        else:
            reason = ''
        for module in self.modules.values():
            module.set_start_enabled(connected and not homing)
        self.hint.set_text(reason)
