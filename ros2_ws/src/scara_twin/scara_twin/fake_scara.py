"""
fake_scara.py - Robot falso para probar el gemelo SIN hardware.

Publica /scara/status ("HOMED", cada 1 s) y /scara/limits (10 Hz) con el mismo
formato que el ESP32, moviendo las tres articulaciones con senos. Sirve para
validar el puente y RViz antes de conectar el robot real.

  ros2 run scara_twin fake_scara                          # imita el firmware nuevo (fase 4)
  ros2 run scara_twin fake_scara --ros-args -p publish_joints:=false   # imita el firmware viejo

Con publish_joints:=true (por defecto) publica tambien /scara/joints a 25 Hz, igual
que el firmware nuevo: [Q1, Q2, Q3, Q4, CAL] en best effort.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray, String

from scara_twin.conversions import Z_HOME_MM


class FakeScara(Node):
    def __init__(self):
        super().__init__('fake_scara')
        self._limits_pub = self.create_publisher(String, 'scara/limits', 10)
        self._status_pub = self.create_publisher(String, 'scara/status', 10)
        self.declare_parameter('publish_joints', True)
        self._joints_pub = None
        if bool(self.get_parameter('publish_joints').value):
            self._joints_pub = self.create_publisher(
                Float32MultiArray, 'scara/joints', qos_profile_sensor_data)
            self.create_timer(0.04, self._publish_joints)   # 25 Hz, como el firmware nuevo
        self._t0 = self.get_clock().now()
        self.create_timer(0.1, self._publish_limits)    # el firmware revisa cada 100 ms
        self.create_timer(1.0, self._publish_status)    # latido cada 1 s
        self.get_logger().warn('ROBOT FALSO: no hay hardware, los datos son simulados')

    def _pose(self):
        t = (self.get_clock().now() - self._t0).nanoseconds * 1e-9
        return (60.0 * math.sin(0.5 * t),
                80.0 * math.sin(0.8 * t),
                Z_HOME_MM + 45.0 * math.sin(0.6 * t))

    def _publish_status(self):
        self._status_pub.publish(String(data='HOMED'))

    def _publish_limits(self):
        q1, q2, q3 = self._pose()
        text = (f'L1=0 R1=0 L2=0 R2=0 L3=0 R3=0 M1=0 M2=0 M3=0 '
                f'E1=0 E2=0 E3=0 Q1={q1:.1f} Q2={q2:.1f} Q3={q3:.1f} CAL=1')
        self._limits_pub.publish(String(data=text))

    def _publish_joints(self):
        q1, q2, q3 = self._pose()
        self._joints_pub.publish(Float32MultiArray(data=[q1, q2, q3, 0.0, 1.0]))


def main(args=None):
    rclpy.init(args=args)
    node = FakeScara()
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
