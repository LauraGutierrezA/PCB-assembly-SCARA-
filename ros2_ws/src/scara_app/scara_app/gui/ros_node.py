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
  /scara/test_cmd (std_msgs/String) PC -> ESP32   prueba de circuito: "run 1 L 45 1.0" | "stop 1" | "stop all"
  /scara/limits  (std_msgs/String)  ESP32 -> PC   "L1=0 R1=1 ... M1=0 M2=1 M3=0 E1=1234 ... Q1=12.3 ..."
                 (ver parse_limits)
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
        if key in ('codo', 'homed', 'n', 'joint'):
            result[key] = int(value)
        elif key == 'reason':
            result[key] = value
        else:
            try:
                result[key] = float(value)
            except ValueError:
                result[key] = value
    return result


def parse_limits(text: str) -> dict:
    """
    "L1=0 R1=1 ... M1=0 M2=1 M3=0 E1=1234 E2=-56 E3=0 Q1=12.3 Q2=-0.5 Q3=0.0"
      -> {'L1': False, 'R1': True, ..., 'E1': 1234, ..., 'Q1': 12.3, ...}
    L/R/M -> True/False (end-stops y motores)
    E     -> int   (cuentas del encoder)
    Q     -> float (Q1, Q2 en grados; Q3 en mm desde el piso)
    CAL   -> True/False (el homingPRO ya calibro; si no, Q no vale)
    """
    out = {}
    for token in text.split():
        if '=' not in token:
            continue
        key, value = token.split('=', 1)
        value = value.strip()
        try:
            if key.startswith('E'):
                out[key] = int(value)
            elif key.startswith('Q'):
                out[key] = float(value)
            else:
                out[key] = value == '1'
        except ValueError:
            pass   # dato malo: se ignora ese campo
    return out


class RosSignals(QObject):
    """Senales de Qt que emite el nodo hacia la interfaz."""
    status_received = pyqtSignal(str)        # cada mensaje de /scara/status
    status_changed = pyqtSignal(str)         # solo cuando el texto cambia
    connection_changed = pyqtSignal(bool)    # True = ESP32 respondiendo
    ik_result = pyqtSignal(dict)             # cada mensaje de /scara/ik (ya interpretado)
    limits_received = pyqtSignal(dict)       # cada mensaje de /scara/limits (ya interpretado)


class ScaraGuiNode(Node):
    def __init__(self):
        super().__init__('scara_gui')
        self.signals = RosSignals()

        self._cmd_pub = self.create_publisher(String, 'scara/cmd', 10)
        self._target_pub = self.create_publisher(Float32MultiArray, 'scara/target', 10)
        self.create_subscription(String, 'scara/status', self._on_status, 10)
        self.create_subscription(String, 'scara/ik', self._on_ik, 10)
        # Prueba de circuito
        self._test_pub = self.create_publisher(String, 'scara/test_cmd', 10)
        self.create_subscription(String, 'scara/limits', self._on_limits, 10)

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

    def test_run(self, motor: int, direction: str, speed_pct: float, seconds: float):
        """Prueba de circuito: mueve 'motor' (1-3) hacia 'L' o 'R'."""
        self._send_test(f'run {motor} {direction} {speed_pct:.0f} {seconds:.2f}')

    def test_stop(self, motor=None):
        """Prueba de circuito: detiene un motor (1-3) o todos (None)."""
        self._send_test('stop all' if motor is None else f'stop {motor}')

    def _send_test(self, text: str):
        msg = String()
        msg.data = text
        self._test_pub.publish(msg)
        self.get_logger().info(f'Prueba: {text}')

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

    def _on_limits(self, msg: String):
        self.signals.limits_received.emit(parse_limits(msg.data))

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
