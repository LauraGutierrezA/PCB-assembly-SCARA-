"""
bridge_node.py - Puente robot real -> gemelo digital.

Lee el estado medido por los encoders del ESP32 y lo publica como
sensor_msgs/JointState en /joint_states (RViz) y, opcionalmente, como comandos
de posicion para Gazebo.

Dos fuentes, con prioridad:
  1. /scara/joints  (Float32MultiArray [Q1,Q2,Q3,Q4,CAL], 25 Hz, best effort):
     la que publica el firmware nuevo (fase 4). Es la preferida.
  2. /scara/limits  (String, ~10 Hz): la que ya existia. Solo se usa cuando
     /scara/joints no llega (firmware viejo): sirve de respaldo.

SOLO LEE: no publica nunca en /scara/cmd ni /scara/target, asi que el gemelo no
puede mover el robot real.

Parametros:
  joints_topic        'scara/joints'
  joints_fresh_s      0.5   (si /scara/joints no llega en este tiempo, se usa /scara/limits)
  limits_topic        'scara/limits'
  status_topic        'scara/status'   (latido del ESP32, para detectar conexion)
  joint_states_topic  'joint_states'
  publish_rate_hz     30.0
  smoothing           0.4   (0..1; 1.0 = sin suavizado)
  connection_timeout_s 2.5  (igual que CONNECTION_TIMEOUT_S de scara_app)
  require_cal         True  (ignora Q mientras CAL=0)
  publish_gz_commands False (True: ademas manda las mismas posiciones al gemelo de Gazebo)
  gz_commands_topic   'forward_position_controller/commands'
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Float64MultiArray, String

from scara_twin.conversions import (
    JOINT_NAMES, home_joints, joints_array_to_joints, limits_to_joints, parse_limits,
    smooth_step,
)


class ScaraTwinBridge(Node):
    def __init__(self):
        super().__init__('scara_twin_bridge')

        self.declare_parameter('joints_topic', 'scara/joints')
        self.declare_parameter('joints_fresh_s', 0.5)
        self.declare_parameter('limits_topic', 'scara/limits')
        self.declare_parameter('status_topic', 'scara/status')
        self.declare_parameter('joint_states_topic', 'joint_states')
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('smoothing', 0.4)
        self.declare_parameter('connection_timeout_s', 2.5)
        self.declare_parameter('require_cal', True)
        self.declare_parameter('publish_gz_commands', False)
        self.declare_parameter('gz_commands_topic', 'forward_position_controller/commands')

        p = self.get_parameter
        self._alpha = float(p('smoothing').value)
        self._timeout = float(p('connection_timeout_s').value)
        self._require_cal = bool(p('require_cal').value)
        self._joints_fresh = float(p('joints_fresh_s').value)
        self._last_joints_time = -1e9    # ultima vez que llego /scara/joints
        self._source = None              # 'joints' | 'limits': de donde salen los datos ahora

        self._target = home_joints()     # ultimo dato valido del robot real
        self._current = home_joints()    # lo que se publica (suavizado)
        self._have_data = False
        self._connected = False
        self._last_status_time = 0.0

        self._pub = self.create_publisher(JointState, p('joint_states_topic').value, 10)
        # Gazebo: el JointGroupPositionController recibe [joint1, joint2, joint3] (mismo orden
        # que JOINT_NAMES) con la misma pose suavizada que ve RViz, para que ambos vayan sincronizados.
        self._gz_pub = None
        if bool(p('publish_gz_commands').value):
            self._gz_pub = self.create_publisher(
                Float64MultiArray, p('gz_commands_topic').value, 10)
        # El firmware publica /scara/joints en best effort: un suscriptor "reliable" no la
        # recibiria. qos_profile_sensor_data (best effort) es compatible con ambos.
        self.create_subscription(Float32MultiArray, p('joints_topic').value,
                                 self._on_joints, qos_profile_sensor_data)
        self.create_subscription(String, p('limits_topic').value, self._on_limits, 10)
        self.create_subscription(String, p('status_topic').value, self._on_status, 10)

        period = 1.0 / float(p('publish_rate_hz').value)
        self.create_timer(period, self._tick)
        self.create_timer(0.5, self._check_connection)

        self.get_logger().info(
            'Puente listo: esperando /scara/joints o /scara/limits '
            '(gemelo en posicion de arranque)')

    # ---------------- Callbacks ----------------
    def _on_status(self, _msg: String):
        self._last_status_time = time.monotonic()
        if not self._connected:
            self._connected = True
            self.get_logger().info('ESP32 conectado')

    def _joints_fresh_now(self) -> bool:
        return (time.monotonic() - self._last_joints_time) < self._joints_fresh

    def _on_joints(self, msg: Float32MultiArray):
        self._last_joints_time = time.monotonic()
        self._apply(joints_array_to_joints(msg.data, self._require_cal), 'joints')

    def _on_limits(self, msg: String):
        if self._joints_fresh_now():
            return    # /scara/joints llega: es la fuente preferida, esta es solo respaldo
        limits = parse_limits(msg.data)
        self._apply(limits_to_joints(limits, self._require_cal), 'limits')

    def _apply(self, joints, source: str):
        """Acepta una pose valida de la fuente indicada (o avisa si no hay dato valido)."""
        if joints is None:
            self.get_logger().warn(
                'Sin dato valido (CAL=0 o faltan Q1/Q2/Q3): el gemelo se queda donde esta. '
                'Haz el homing desde la interfaz.', throttle_duration_sec=5.0)
            return
        if source != self._source:
            self._source = source
            self.get_logger().info(f'Fuente de datos del gemelo: /scara/{source}')
        if not self._have_data:
            # Primer dato: salta directo, sin recorrer el camino desde la posicion de arranque.
            self._current = joints
            self._have_data = True
            self.get_logger().info('Primer dato valido recibido: el gemelo sigue al robot')
        self._target = joints

    def _check_connection(self):
        if self._connected and (time.monotonic() - self._last_status_time) > self._timeout:
            self._connected = False
            self.get_logger().warn(
                'Se perdio la conexion con el ESP32: el gemelo conserva la ultima pose')

    # ---------------- Publicacion ----------------
    def _tick(self):
        self._current = smooth_step(self._current, self._target, self._alpha)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = list(self._current)
        self._pub.publish(msg)
        if self._gz_pub is not None:
            self._gz_pub.publish(Float64MultiArray(data=list(self._current)))


def main(args=None):
    rclpy.init(args=args)
    node = ScaraTwinBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
