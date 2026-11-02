"""
ros_node.py - Nodo ROS 2 de la interfaz web.

Mismos topicos que gui/ros_node.py (ver ese archivo para el formato de cada
uno). La diferencia es COMO le avisa a la interfaz:

  - PyQt5: ROS y Qt corrian en el mismo hilo y el nodo emitia senales Qt.
  - Web:   ROS corre en su propio hilo (executor.spin) y NiceGUI en el hilo
           principal. Los callbacks de ROS NO tocan la interfaz: solo anotan
           "eventos" numerados en una lista. Cada pagina abierta (el iPad, el
           PC, ...) revisa cada 50 ms que eventos nuevos hay y actualiza su
           propia pantalla. Asi no hay problemas de hilos y pueden estar
           abiertas varias pantallas a la vez.

Eventos: ('status', (texto, cambio)), ('connection', bool),
         ('ik', dict), ('limits', dict),
         ('wiimote', dict)             estado del control con Wiimote (~15 Hz, solo mientras
                                       hay Wiimote o esta activado; ver wiimote_bridge.py)
         ('wiimote_log', (texto, nivel))

El control con Wiimote vive en self.wiimote (WiimoteBridge): UNA instancia en el
nodo, no una por pantalla, asi varias paginas abiertas no mandan consignas duplicadas.
"""
import threading
import time
from collections import deque

from rclpy.node import Node
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Float32MultiArray, String

from scara_app.gui.point_to_point_tab import ROBOT_DEFAULTS
from scara_app.robot_model import codo_for, fk_xy
from scara_app.wiimote_bridge import WiimoteBridge
from scara_app.wiimote_control import BUTTON_NAMES

# Se reutilizan los parsers de la version de escritorio (un solo lugar).
from scara_app.gui.ros_node import CONNECTION_TIMEOUT_S, parse_ik, parse_limits

MAX_EVENTS = 2000   # eventos guardados; una pagina que se atrase mas pierde los viejos


class ScaraWebNode(Node):
    def __init__(self):
        super().__init__('scara_web')

        self._cmd_pub = self.create_publisher(String, 'scara/cmd', 10)
        self._target_pub = self.create_publisher(Float32MultiArray, 'scara/target', 10)
        self._test_pub = self.create_publisher(String, 'scara/test_cmd', 10)
        self.create_subscription(String, 'scara/status', self._on_status, 10)
        self.create_subscription(String, 'scara/ik', self._on_ik, 10)
        self.create_subscription(String, 'scara/limits', self._on_limits, 10)

        self._lock = threading.Lock()
        self._events = deque(maxlen=MAX_EVENTS)
        self._seq = 0

        self._last_status = None
        self._last_status_time = 0.0
        self._connected = False
        self._last_limits = {}
        self.create_timer(0.5, self._check_connection)

        # Control con Wiimote (desactivado hasta activarlo en su pestana)
        robot = dict(ROBOT_DEFAULTS)
        self._wiimote_active = False
        self._wiimote_cmd_pub = self.create_publisher(String, 'wiimote/cmd', 10)
        self.wiimote = WiimoteBridge(
            robot,
            send_joints=lambda th1, th2, z: self.send_joints(th1, th2, z, robot),
            deadman_beat=self.deadman_beat,
            deadman_release=self.deadman_release,
            emergency_stop=self.emergency_stop,
            log=lambda text, level: self._push('wiimote_log', (text, level)),
            on_state=self._on_wiimote_state,
            calibrate=lambda: self._wiimote_cmd_pub.publish(String(data='calibrate')))
        self.create_subscription(Joy, 'wiimote/joy', self._on_wiimote_joy, 10)
        self.create_subscription(Imu, 'wiimote/imu', self._on_wiimote_imu, 10)
        self.create_timer(0.02, self.wiimote.tick)

    # ---------------- Eventos para las paginas ----------------
    def _push(self, kind, payload):
        with self._lock:
            self._seq += 1
            self._events.append((self._seq, kind, payload))

    def events_since(self, seq):
        """Devuelve (ultimo_seq, [(tipo, dato), ...]) con los eventos posteriores a seq."""
        with self._lock:
            if seq >= self._seq:
                return seq, []
            new = [(kind, payload) for s, kind, payload in self._events if s > seq]
            return self._seq, new

    @property
    def latest_seq(self):
        with self._lock:
            return self._seq

    # ---------------- Comandos (los llaman las pestanas) ----------------
    def send_command(self, cmd: str):
        msg = String()
        msg.data = cmd
        self._cmd_pub.publish(msg)
        self.get_logger().info(f'Comando enviado: {cmd}')

    # ---------------- Dead-man switch ----------------
    def deadman_beat(self):
        """Latido del dead-man (cada DEADMAN_PERIOD_S mientras se mantiene). Sin log."""
        self._publish_cmd_quiet('dm1')

    def deadman_release(self):
        """Dead-man suelto: el firmware detiene los motores de inmediato."""
        self._publish_cmd_quiet('dm0')

    def release_control(self):
        """Borra la consigna en el firmware (al entrar a Prueba de circuito)."""
        self.send_command('release')

    def _publish_cmd_quiet(self, cmd: str):
        msg = String()
        msg.data = cmd
        self._cmd_pub.publish(msg)

    def send_joints(self, th1: float, th2: float, z: float, params: dict):
        """Pestana DK: (th1, th2) en grados y z en mm -> consigna cartesiana.
        La IK del firmware devuelve los mismos angulos (ver robot_model.py)."""
        x, y = fk_xy(th1, th2, params['d1'], params['d2'])
        self.send_target(x, y, z, params, codo_for(th2))

    def send_target(self, x: float, y: float, z: float, params: dict, codo: int = 1):
        """params: {'d1','d2','d3','d4','d5','d6','av'} en mm (av en mm/vuelta)."""
        msg = Float32MultiArray()
        msg.data = [float(v) for v in (x, y, z,
                    params['d1'], params['d2'], params['d3'],
                    params['d4'], params['d5'], params['d6'], params['av'],
                    1 if codo >= 0 else -1)]
        self._target_pub.publish(msg)
        self.get_logger().info(f'Consigna enviada: x={x:.1f} y={y:.1f} z={z:.1f} codo={codo:+d}')

    def test_run(self, motor: int, direction: str, speed_pct: float, seconds: float):
        self._send_test(f'run {motor} {direction} {speed_pct:.0f} {seconds:.2f}')

    def test_stop(self, motor=None):
        self._send_test('stop all' if motor is None else f'stop {motor}')

    def servo_set(self, degrees: float):
        """Prueba de circuito: servo del efector (GPIO 2) al angulo dado, -90 a +90."""
        self.send_command(f'servo {int(round(max(-90.0, min(90.0, degrees))))}')

    def pump_set(self, on: bool):
        """Prueba de circuito: enciende o apaga la bomba de vacio (GPIO 16)."""
        self.send_command('vac1' if on else 'vac0')

    def emergency_stop(self):
        """Boton STOP del encabezado: detiene el homing, la prueba de circuito y
        el control (el firmware borra la consigna y suelta el dead-man). Tambien
        desactiva el control con Wiimote."""
        self.send_command('stop')
        self.test_stop(None)
        self.wiimote.set_enabled(False)
        self._push('estop', None)   # TRAYECTORIA: las pestanas abortan

    # ---------------- Trayectoria (TRAYECTORIA) ----------------
    def stream_target(self, x: float, y: float, z: float, params: dict, codo: int = 1):
        """Como send_target pero SIN log: la pestana Trayectoria manda ~20 por segundo."""
        msg = Float32MultiArray()
        msg.data = [float(v) for v in (x, y, z,
                    params['d1'], params['d2'], params['d3'],
                    params['d4'], params['d5'], params['d6'], params['av'],
                    1 if codo >= 0 else -1)]
        self._target_pub.publish(msg)

    def vacuum(self, on: bool):
        """Bomba de la ventosa: PICK -> True, PLACE -> False."""
        self.send_command('vac1' if on else 'vac0')

    def rotate(self, delta_deg: int):
        """Servo de la boquilla: giro relativo de +90 o -90 grados."""
        self.send_command(f'rot {int(delta_deg):+d}')

    def _send_test(self, text: str):
        msg = String()
        msg.data = text
        self._test_pub.publish(msg)
        self.get_logger().info(f'Prueba: {text}')

    # ---------------- Callbacks de ROS (hilo de ROS) ----------------
    def _on_status(self, msg: String):
        self._last_status_time = time.monotonic()
        if not self._connected:
            self._connected = True
            self._push('connection', True)
        changed = msg.data != self._last_status
        self._last_status = msg.data
        self._push('status', (msg.data, changed))

    def _on_ik(self, msg: String):
        result = parse_ik(msg.data)
        self.wiimote.on_ik_result(result)
        self._push('ik', result)

    def _on_limits(self, msg: String):
        data = parse_limits(msg.data)
        self._last_limits = data
        self.wiimote.on_limits(data)
        self._push('limits', data)

    def _on_wiimote_joy(self, msg: Joy):
        self.wiimote.on_joy([name for name, v in zip(BUTTON_NAMES, msg.buttons) if v])

    def _on_wiimote_imu(self, msg: Imu):
        w = msg.angular_velocity
        self.wiimote.on_imu(w.x, w.y, w.z)

    def _on_wiimote_state(self, state):
        """Solo se anuncia mientras hay Wiimote o esta activado (si no, la lista de
        eventos se llenaria de estados que nadie mira)."""
        active = state['enabled'] or state['link']
        if active or self._wiimote_active:
            self._push('wiimote', state)
        self._wiimote_active = active

    def _check_connection(self):
        alive = (time.monotonic() - self._last_status_time) < CONNECTION_TIMEOUT_S
        if alive != self._connected:
            self._connected = alive
            if not alive:
                self._last_status = None
                self._last_limits = {}
            self._push('connection', alive)

    # ---------------- Estado actual (para pintar una pagina recien abierta) ----------------
    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def last_status(self):
        return self._last_status

    @property
    def last_limits(self) -> dict:
        return self._last_limits
