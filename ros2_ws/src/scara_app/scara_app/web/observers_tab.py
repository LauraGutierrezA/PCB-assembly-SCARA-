"""
observers_tab.py - Pestana "Observadores" (web).

Grafica en vivo, para cada motor:
  - Corriente de armadura estimada [A]: Kalman y Luenberger.
  - Velocidad [RPM del eje de salida]: medida (encoder), Luenberger y Kalman.

Los observadores corren en el ESP32 (firmware/components/observers, cada
8 ms con el voltaje aplicado y la velocidad del encoder). El ESP32 publica en
/scara/obs el promedio de cada 40 ms (25 Hz), pero SOLO mientras alguien ve
esta pestana: la pagina manda el latido "obs1" cada 1 s.

Esta pestana no mueve el robot. Para moverlo se usa otra pestana y se vuelve
aqui. Entrar a esta pestana NO cancela lo que estaba corriendo (bypass en
web/app.py): la Trayectoria sigue y el Wiimote sigue activado. Si se venia de
Trayectoria, Punto a punto o DK, aqui aparece el mismo dead-man (boton o
barra Espacio) para seguir moviendo el robot sin salir de las graficas.
"""
import math
import time
from collections import deque

from nicegui import ui

from .widgets import button, caps_label, deadman_button

NUM_MOTORS = 3
SAMPLE_S = 0.040          # periodo de /scara/obs (OBS_DECIM * 8 ms en el firmware)
WINDOW_S = 5.0            # ventana deslizante de las graficas
REDRAW_S = 0.2            # cada cuanto se redibuja (5 Hz)
KEEPALIVE_S = 1.0         # latido "obs1" hacia el ESP32
STALE_S = 1.0             # sin muestras en este tiempo -> "sin datos"

# Velocidad: el firmware la manda en rad/s del EJE DEL MOTOR (unidades del
# modelo). Se grafica en RPM del eje de SALIDA, como la pagina del Lab 4.
GEAR_RATIO = 50.0
RAD_S_TO_RPM_OUT = 60.0 / (2.0 * math.pi * GEAR_RATIO)

MOTOR_NAMES = {1: 'MOTOR 1 · th1', 2: 'MOTOR 2 · th2', 3: 'MOTOR 3 · eje Z'}

# Pestanas que se quedan corriendo cuando se entra aqui
OWNER_LABELS = {'traj': 'Trayectoria', 'wiimote': 'Wiimote', 'p2p': 'Punto a punto',
                'dk': 'DK', 'homing': 'Homing', 'circuit': 'Prueba de circuito'}
DEADMAN_OWNERS = ('traj', 'p2p', 'dk')

# Colores de las series (validados contra el fondo de los paneles, modo oscuro).
# Medida: neutro (es la referencia). Mismo color = mismo estimador en todas las graficas.
C_MEAS = '#8b93ad'
C_LUEN = '#c3850f'
C_KAL = '#1ea2aa'
AXIS = '#8b93ad'
GRID = '#1c2554'
TEXT = '#d8ddec'
TOOLTIP_BG = '#0a0f2e'

# Indices dentro del bloque de 5 valores de cada motor en /scara/obs
W_MEAS, W_LUEN, W_KAL, I_LUEN, I_KAL = range(5)


def _series(name, color, width, z):
    return {
        'name': name, 'type': 'line', 'data': [], 'showSymbol': False,
        'lineStyle': {'width': width, 'color': color}, 'itemStyle': {'color': color},
        'emphasis': {'disabled': True}, 'z': z, 'animation': False,
    }


def _chart_options(title, unit, series):
    return {
        'animation': False,
        'backgroundColor': 'transparent',
        'title': {'text': f'{title}  [{unit}]', 'left': 0, 'top': 0,
                  'textStyle': {'color': TEXT, 'fontSize': 13, 'fontWeight': 500}},
        'legend': {'top': 0, 'right': 0, 'itemWidth': 18, 'itemHeight': 2,
                   'icon': 'rect', 'textStyle': {'color': AXIS, 'fontSize': 12}},
        'grid': {'left': 52, 'right': 12, 'top': 30, 'bottom': 26},
        'tooltip': {'trigger': 'axis', 'backgroundColor': TOOLTIP_BG,
                    'borderColor': GRID, 'textStyle': {'color': TEXT, 'fontSize': 12},
                    'axisPointer': {'type': 'line', 'lineStyle': {'color': AXIS}}},
        'xAxis': {'type': 'value', 'min': -WINDOW_S, 'max': 0,
                  'axisLabel': {'color': AXIS, 'fontSize': 11, 'formatter': '{value} s'},
                  'axisLine': {'lineStyle': {'color': GRID}},
                  'splitLine': {'show': False}},
        'yAxis': {'type': 'value', 'scale': True,
                  'axisLabel': {'color': AXIS, 'fontSize': 11},
                  'axisLine': {'show': False},
                  'splitLine': {'lineStyle': {'color': GRID, 'width': 1}}},
        'series': series,
    }


class MotorPlots:
    """Las dos graficas de un motor y sus buffers."""

    def __init__(self, motor):
        self.motor = motor
        n = int(WINDOW_S / SAMPLE_S) + 2
        self.t = deque(maxlen=n)
        self.vals = [deque(maxlen=n) for _ in range(5)]

        with ui.element('div').classes('panel obs-panel'):
            ui.label(MOTOR_NAMES[motor]).classes('panel-title')
            with ui.element('div').classes('obs-row'):
                self.current = ui.echart(_chart_options(
                    'Corriente estimada', 'A',
                    [_series('Kalman', C_KAL, 2, 3),
                     _series('Luenberger', C_LUEN, 2, 2)])).classes('obs-chart')
                self.speed = ui.echart(_chart_options(
                    'Velocidad eje de salida', 'RPM',
                    [_series('Medida', C_MEAS, 1.5, 1),
                     _series('Luenberger', C_LUEN, 2, 2),
                     _series('Kalman', C_KAL, 2, 3)])).classes('obs-chart')

    def add(self, t, block):
        self.t.append(t)
        for j in range(5):
            v = block[j]
            if j <= W_KAL:
                v *= RAD_S_TO_RPM_OUT
            self.vals[j].append(v)

    def clear(self):
        self.t.clear()
        for d in self.vals:
            d.clear()
        self.redraw()

    def redraw(self):
        t_last = self.t[-1] if self.t else 0.0
        xs = [round(t - t_last, 3) for t in self.t]

        def pts(j, nd):
            return [[x, round(v, nd)] for x, v in zip(xs, self.vals[j])]

        cs = self.current.options['series']
        cs[0]['data'] = pts(I_KAL, 4)
        cs[1]['data'] = pts(I_LUEN, 4)
        ss = self.speed.options['series']
        ss[0]['data'] = pts(W_MEAS, 2)
        ss[1]['data'] = pts(W_LUEN, 2)
        ss[2]['data'] = pts(W_KAL, 2)
        self.current.update()
        self.speed.update()


class ObserversTab:
    def __init__(self, node, hub):
        self.node = node
        self.visible = False
        self.paused = False
        self.dirty = False
        self.last_n = None
        self.last_rx = None      # time.monotonic de la ultima muestra
        self.t_rx = 0.0          # reloj de las muestras (n * 40 ms)

        with ui.element('div').classes('tab-body obs-body'):
            with ui.element('div').classes('obs-bar'):
                with ui.element('div').classes('obs-info'):
                    caps_label('Datos')
                    self.status = ui.label('—').classes('obs-status')
                with ui.element('div').classes('obs-info'):
                    caps_label('Sigue corriendo')
                    self.owner_label = ui.label('—').classes('obs-status')
                ui.element('div').classes('grow')
                self.pause_btn = button('Pausar', 'secondary', self._toggle_pause)
                button('Limpiar', 'secondary', self._clear)
            # Dead-man de la pestana de origen (Trayectoria, Punto a punto, DK)
            self.deadman = deadman_button('obs').classes('obs-deadman')
            self.deadman.set_visibility(False)

            with ui.element('div').classes('obs-motors'):
                self.motors = {m: MotorPlots(m) for m in range(1, NUM_MOTORS + 1)}

        hub.on('obs', self._on_obs)
        hub.on('connection', lambda _c: self._render_status())
        ui.timer(REDRAW_S, self._redraw)
        ui.timer(KEEPALIVE_S, self._keepalive)
        ui.timer(0.5, self._render_status)

    # ---------------- Lo llama app.py ----------------
    def on_view_change(self, name):
        self.visible = (name == 'obs')
        if self.visible:
            self.dirty = True
            self._keepalive()          # que el ESP32 empiece a publicar ya
            self._render_status()

    def set_owner(self, owner):
        """Pestana que quedo corriendo al entrar aqui (bypass de app.py)."""
        self.owner_label.set_text(OWNER_LABELS.get(owner, '—'))
        self.deadman.set_visibility(owner in DEADMAN_OWNERS)

    # ---------------- Datos ----------------
    def _on_obs(self, data):
        n = int(round(data[0]))
        if self.last_n is not None and n <= self.last_n - 25:
            self._clear()              # se reinicio el ESP32 (el contador volvio a empezar)
        if self.last_n is None or n > self.last_n:
            gap = 1 if self.last_n is None else n - self.last_n
            self.t_rx += gap * SAMPLE_S
            self.last_n = n
        self.last_rx = time.monotonic()
        if self.paused:
            return
        for m, plots in self.motors.items():
            k = 1 + (m - 1) * 5
            plots.add(self.t_rx, data[k:k + 5])
        self.dirty = True

    def _redraw(self):
        if not self.visible or self.paused or not self.dirty:
            return
        self.dirty = False
        for plots in self.motors.values():
            plots.redraw()

    def _keepalive(self):
        if self.visible and self.node.connected:
            self.node.observers_keepalive()

    def _render_status(self):
        if not self.node.connected:
            self.status.set_text('ESP32 sin conexion')
        elif self.paused:
            self.status.set_text('En pausa')
        elif self.last_rx is None or time.monotonic() - self.last_rx > STALE_S:
            self.status.set_text('Esperando datos…')
        else:
            self.status.set_text('En vivo · 25 Hz')

    # ---------------- Botones ----------------
    def _toggle_pause(self):
        self.paused = not self.paused
        self.pause_btn.set_text('Reanudar' if self.paused else 'Pausar')
        if not self.paused:
            self.dirty = True
        self._render_status()

    def _clear(self):
        self.last_n = None
        self.t_rx = 0.0
        for plots in self.motors.values():
            plots.clear()
