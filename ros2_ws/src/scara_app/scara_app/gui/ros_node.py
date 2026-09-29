"""
ros_node.py - Nodo ROS 2 de la interfaz grafica.

Toda la comunicacion con el ESP32 pasa por aqui. Las pestanas de la
interfaz NO usan rclpy directamente: llaman metodos de este nodo y
escuchan sus senales de Qt. Asi, agregar una pestana nueva (punto a punto,
jog, caracterizacion) es solo agregar un metodo/senal aqui.

Topicos:
  /scara/cmd     (std_msgs/String)  PC -> ESP32   "start" | "stop" | "resume"
  /scara/status  (std_msgs/String)  ESP32 -> PC   "IDLE", "HOMING M1 SEEK_L", ...
"""
import time

from PyQt5.QtCore import QObject, pyqtSignal
from rclpy.node import Node
from std_msgs.msg import String

# Si no llega /scara/status en este tiempo, se considera desconectado.
# El ESP32 publica un "latido" cada 1 s.
CONNECTION_TIMEOUT_S = 2.5


class RosSignals(QObject):
    """Senales de Qt que emite el nodo hacia la interfaz."""
    status_received = pyqtSignal(str)        # cada mensaje de /scara/status
    status_changed = pyqtSignal(str)         # solo cuando el texto cambia
    connection_changed = pyqtSignal(bool)    # True = ESP32 respondiendo


class ScaraGuiNode(Node):
    def __init__(self):
        super().__init__('scara_gui')
        self.signals = RosSignals()

        self._cmd_pub = self.create_publisher(String, 'scara/cmd', 10)
        self.create_subscription(String, 'scara/status', self._on_status, 10)

        self._last_status = None
        self._last_status_time = 0.0
        self._connected = False
        # Revisa la conexion 2 veces por segundo
        self.create_timer(0.5, self._check_connection)

    # ---------------- Comandos (los llaman las pestanas) ----------------
    def send_command(self, cmd: str):
        msg = String()
        msg.data = cmd
        self._cmd_pub.publish(msg)
        self.get_logger().info(f'Comando enviado: {cmd}')

    # ---------------- Callbacks internos ----------------
    def _on_status(self, msg: String):
        self._last_status_time = time.monotonic()
        if not self._connected:
            self._connected = True
            self.signals.connection_changed.emit(True)

        self.signals.status_received.emit(msg.data)
        if msg.data != self._last_status:
            self._last_status = msg.data
            self.signals.status_changed.emit(msg.data)

    def _check_connection(self):
        alive = (time.monotonic() - self._last_status_time) < CONNECTION_TIMEOUT_S
        if alive != self._connected:
            self._connected = alive
            if not alive:
                self._last_status = None   # al reconectar, se vuelve a mostrar el estado
            self.signals.connection_changed.emit(alive)

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def last_status(self):
        return self._last_status
