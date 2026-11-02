"""
circuit_test_tab.py - Pestana "Prueba de circuito" (web).

Igual que gui/circuit_test_tab.py: un modulo por motor con end-stops,
encoder y accionamiento. Para que quepa en el iPad, cada modulo es mas
compacto: end-stops y encoder van en la misma fila, y sentido y duracion
tambien.

Mas el servo (GPIO 2) y la bomba de vacio (GPIO 16), como en gui/circuit_test_tab.py.

Constantes (velocidades, nombres de los finales, unidades) importadas de la
version de escritorio.
"""
from nicegui import ui

from scara_app.gui.circuit_test_tab import (
    DEFAULT_DURATION_S, DEFAULT_SPEED_PCT, END_STOP_NAMES, NUM_MOTORS, POSITION_UNITS,
    SERVO_DEFAULT_DEG, SERVO_MAX_DEG, SERVO_MIN_DEG, SERVO_PRESETS,
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


class ServoModule:
    """Servo en GPIO 2: slider -90..+90 (se manda al soltar), atajos y Enviar."""

    def __init__(self, node):
        self.node = node
        with ui.element('div').classes('panel'):
            with ui.element('div').classes('panel-head'):
                ui.label('SERVO  (GPIO 2)').classes('panel-title')
                self.state_label = ui.label('—').classes('caps')
            caps_label('Angulo')
            with ui.element('div').classes('speed-row'):
                self.slider = (ui.slider(min=SERVO_MIN_DEG, max=SERVO_MAX_DEG, step=1, value=SERVO_DEFAULT_DEG)
                               .props('thumb-size=28px track-size=6px')
                               .classes('slider grow')
                               .on('change', lambda _e: self._send()))
                ui.label().classes('speed-value').bind_text_from(
                    self.slider, 'value', lambda v: f'{int(v)}°')
            with ui.element('div').classes('btn-row'):
                for deg in SERVO_PRESETS:
                    button(f'{deg}°', 'secondary', lambda _e=None, d=deg: self._go(d))
                self.btn_send = button('Enviar', 'primary', self._send)

    def _go(self, deg):
        self.slider.set_value(deg)
        self._send()

    def _send(self):
        self.node.servo_set(float(self.slider.value))

    def set_state(self, angle):
        if angle is None:
            self.state_label.set_text('—')
            self.state_label.style('')
        else:
            self.state_label.set_text(f'EN {int(round(angle))}°')
            self.state_label.style('color: var(--scara-accent)')

    def set_enabled(self, enabled):
        self.btn_send.set_enabled(enabled)


class PumpModule:
    """Bomba de vacio en GPIO 16: Encender / Apagar."""

    def __init__(self, node):
        self.node = node
        with ui.element('div').classes('panel'):
            with ui.element('div').classes('panel-head'):
                ui.label('BOMBA DE VACIO  (GPIO 16)').classes('panel-title')
                self.state_label = ui.label('—').classes('caps')
            caps_label('Bomba')
            with ui.element('div').classes('btn-row'):
                self.btn_on = button('Encender', 'primary', lambda: self.node.pump_set(True))
                self.btn_off = button('Apagar', 'danger', lambda: self.node.pump_set(False))

    def set_state(self, on):
        if on is None:
            self.state_label.set_text('—')
            self.state_label.style('')
        elif on:
            self.state_label.set_text('ENCENDIDA')
            self.state_label.style('color: var(--scara-accent)')
        else:
            self.state_label.set_text('APAGADA')
            self.state_label.style('')

    def set_enabled(self, enabled):
        self.btn_on.set_enabled(enabled)
        self.btn_off.set_enabled(enabled)


class CircuitTestTab:
    def __init__(self, node, hub):
        self.node = node

        with ui.element('div').classes('tab-body'):
            with ui.element('div').classes('motors'):
                self.modules = {m: MotorModule(m, node) for m in range(1, NUM_MOTORS + 1)}
                self.servo = ServoModule(node)
                self.pump = PumpModule(node)
            with ui.element('div').classes('circuit-bottom'):
                self.hint = ui.label('').classes('hint')
                button('Detener todos', 'danger', self._stop_all)

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
        self.servo.set_state(d.get('Q4'))
        self.pump.set_state(d.get('VAC'))

    def _stop_all(self):
        self.node.test_stop(None)
        self.node.pump_set(False)          # tambien apaga la bomba

    def _on_connection(self, connected):
        if not connected:
            for module in self.modules.values():
                module.set_end_stops(None, None)
                module.set_running(False)
                module.set_encoder(None, None)
            self.servo.set_state(None)
            self.pump.set_state(None)
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
        self.servo.set_enabled(connected)       # no dependen del homing
        self.pump.set_enabled(connected)
        self.hint.set_text(reason)
