"""
ros_node.py - Nodo ROS 2 de la interfaz grafica.

Toda la comunicacion con el ESP32 pasa por aqui. Las pestanas de la
interfaz NO usan rclpy directamente: llaman metodos de este nodo y
escuchan sus senales de Qt. Asi, agregar una pestana nueva es solo agregar
un metodo/senal aqui.

Topicos:
  /scara/cmd     (std_msgs/String)      PC -> ESP32   "start" | "stop" | "resume"
  /scara/status  (std_msgs/String)      ESP32 -> PC   "IDLE", "HOMING M1 SEEK_L", ...
  /scara/target  (std_msgs/Float32MultiArray)  PC -> ESP32
                 [x, y, z, d1, d2, d3, d4, d5, d6, av, codo]  (mm, av en mm/vuelta, codo +1/-1)
  /scara/ik      (std_msgs/String)  ESP32 -> PC   resultado de la IK (ver parse_ik)
"""
import time

from PyQt5.QtCore import QObject, pyqtSignal
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

# Si no llega /scara/status en este tiempo, se considera desconectado.
# El ESP32 publica un "latido" cada 1 s.
CONNECTION_TIMEOUT_S = 2.5


def parse_ik(text: str) -> dict:
    """
    Convierte el texto de /scara/ik en un diccionario.
      "OK x=300.0 y=150.0 z=0.0 th1=-2.049 th2=66.620 th3=44.171 codo=1 homed=1"
      "ERR x=500.0 y=0.0 z=0.0 reason=OUT_OF_REACH homed=1"
    th1, th2 en grados; th3 en radianes (motor del tornillo).
    Numeros -> float/int; 'ok' -> True/False; 'raw' -> texto original.
    """
    parts = text.split()
    result = {'raw': text, 'ok': bool(parts) and parts[0] == 'OK'}
    for token in parts[1:]:
        if '=' not in token:
            continue
        key, value = token.split('=', 1)
        if key in ('codo', 'homed', 'n'):
            result[key] = int(value)
        elif key == 'reason':
            result[key] = value
        else:
            try:
                result[key] = float(value)
            except ValueError:
                result[key] = value
    return result


class RosSignals(QObject):
    """Senales de Qt que emite el nodo hacia la interfaz."""
    status_received = pyqtSignal(str)        # cada mensaje de /scara/status
    status_changed = pyqtSignal(str)         # solo cuando el texto cambia
    connection_changed = pyqtSignal(bool)    # True = ESP32 respondiendo
    ik_result = pyqtSignal(dict)             # cada mensaje de /scara/ik (ya interpretado)


class ScaraGuiNode(Node):
    def __init__(self):
        super().__init__('scara_gui')
        self.signals = RosSignals()

        self._cmd_pub = self.create_publisher(String, 'scara/cmd', 10)
        self._target_pub = self.create_publisher(Float32MultiArray, 'scara/target', 10)
        self.create_subscription(String, 'scara/status', self._on_status, 10)
        self.create_subscription(String, 'scara/ik', self._on_ik, 10)

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

    def send_target(self, x: float, y: float, z: float, params: dict, codo: int = 1):
        """params: {'d1','d2','d3','d4','d5','d6','av'} en mm (av en mm/vuelta).
        codo: +1 o -1 (configuracion del codo)."""
        msg = Float32MultiArray()
        msg.data = [float(v) for v in (x, y, z,
                    params['d1'], params['d2'], params['d3'],
                    params['d4'], params['d5'], params['d6'], params['av'],
                    1 if codo >= 0 else -1)]
        self._target_pub.publish(msg)
        self.get_logger().info(f'Consigna enviada: x={x:.1f} y={y:.1f} z={z:.1f} codo={codo:+d}')

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

    def _on_ik(self, msg: String):
        self.signals.ik_result.emit(parse_ik(msg.data))

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
