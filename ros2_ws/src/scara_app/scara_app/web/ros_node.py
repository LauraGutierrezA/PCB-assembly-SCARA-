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
         ('ik', dict), ('limits', dict)
"""
import threading
import time
from collections import deque

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from scara_app.robot_model import codo_for, fk_xy

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

    def emergency_stop(self):
        """Boton STOP del encabezado: detiene el homing, la prueba de circuito y
        el control (el firmware borra la consigna y suelta el dead-man)."""
        self.send_command('stop')
        self.test_stop(None)

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
        self._push('ik', parse_ik(msg.data))

    def _on_limits(self, msg: String):
        data = parse_limits(msg.data)
        self._last_limits = data
        self._push('limits', data)

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
