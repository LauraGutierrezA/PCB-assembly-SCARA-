"""
wiimote_node.py - Wiimote -> ROS 2 (Humble / Jazzy)

    ros2 run scara_app wiimote_node

Lee el Wiimote por el driver del kernel de Linux (hid-wiimote -> evdev) y publica:

  /wiimote/joy      sensor_msgs/Joy     ESTADO de los botones a 50 Hz (orden: BUTTON_NAMES de
                                        wiimote_control.py). Es el que usan las interfaces:
                                        al ser un estado (no eventos), un mensaje perdido
                                        nunca deja un boton "pegado".
  /wiimote/imu      sensor_msgs/Imu     angular_velocity (rad/s, sin sesgo) y
                                        linear_acceleration (m/s^2) a 50 Hz
  /wiimote/buttons  std_msgs/String     eventos "DPAD_UP pressed" / "PLUS released" (para ver
                                        con ros2 topic echo)
  /wiimote/cmd      std_msgs/String     entrada: "calibrate" = medir de nuevo el sesgo del giro

Si el Wiimote se desconecta (Bluetooth, pilas, se durmio) el nodo DEJA DE PUBLICAR joy e imu:
las interfaces lo detectan y sueltan el dead-man. Cuando vuelve (se presiona un boton, o 1+2),
el nodo lo reconecta solo y vuelve a calibrar el giroscopio. Se puede iniciar antes de
emparejar el Wiimote.

El giroscopio necesita Wii MotionPlus (el "Wiimote Plus", RVL-CNT-01-TR, ya lo trae). Un
Wiimote comun solo tiene acelerometro: el giroscopio queda en 0.

Calibracion: al conectar se promedia el giroscopio durante gyro_calibration_s segundos
(DEJA EL WIIMOTE QUIETO) y se resta ese promedio. Mientras tanto el giroscopio se publica en 0.

Parametros (--ros-args -p nombre:=valor):
  gyro_deg_per_count   0.0537  grados/s por cuenta del giroscopio (aproximado: calibrar)
  accel_counts_per_g   100.0   cuentas por g del acelerometro (aproximado: calibrar)
  gyro_calibration_s   2.0     0 desactiva la calibracion
  imu_rate_hz          50.0
  print_rate_hz        5.0     0 = no imprimir el giroscopio en la terminal
  frame_id             wiimote_link
"""
import math
import threading
import time

import evdev
from evdev import ecodes

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import String

from scara_app.wiimote_control import BUTTON_NAMES

# codigo de evdev -> (nombre para /wiimote/buttons, nombre para /wiimote/joy)
BUTTONS = {
    ecodes.KEY_UP: ('DPAD_UP', 'UP'),
    ecodes.KEY_DOWN: ('DPAD_DOWN', 'DOWN'),
    ecodes.KEY_LEFT: ('DPAD_LEFT', 'LEFT'),
    ecodes.KEY_RIGHT: ('DPAD_RIGHT', 'RIGHT'),
    ecodes.KEY_NEXT: ('PLUS', 'PLUS'),
    ecodes.KEY_PREVIOUS: ('MINUS', 'MINUS'),
    ecodes.BTN_A: ('A', 'A'),
    ecodes.BTN_B: ('B', 'B'),
    ecodes.BTN_1: ('ONE', 'ONE'),
    ecodes.BTN_2: ('TWO', 'TWO'),
    ecodes.BTN_MODE: ('HOME', 'HOME'),
}

AXIS_INDEX = {ecodes.ABS_RX: 0, ecodes.ABS_RY: 1, ecodes.ABS_RZ: 2}
G = 9.80665


def find_wiimote_devices():
    """Devuelve {'buttons': dev, 'accel': dev, 'gyro': dev} con lo que hay conectado."""
    found = {}
    for path in evdev.list_devices():
        try:
            dev = evdev.InputDevice(path)
        except OSError:
            continue
        name = dev.name
        if 'Nintendo Wii Remote' not in name:
            continue
        if name.endswith('Motion Plus'):
            found['gyro'] = dev
        elif name.endswith('Accelerometer'):
            found['accel'] = dev
        elif name.endswith(('Nunchuk', 'IR', 'Classic Controller')):
            continue  # este nodo no los usa
        elif name == 'Nintendo Wii Remote':
            found['buttons'] = dev
    return found


class WiimoteNode(Node):
    def __init__(self):
        super().__init__('wiimote')

        self.declare_parameter('gyro_deg_per_count', 0.0537)
        self.declare_parameter('accel_counts_per_g', 100.0)
        self.declare_parameter('gyro_calibration_s', 2.0)
        self.declare_parameter('imu_rate_hz', 50.0)
        self.declare_parameter('print_rate_hz', 5.0)
        self.declare_parameter('frame_id', 'wiimote_link')

        self.gyro_scale = self.get_parameter('gyro_deg_per_count').value
        self.accel_counts_per_g = self.get_parameter('accel_counts_per_g').value
        self.cal_duration = self.get_parameter('gyro_calibration_s').value
        self.frame_id = self.get_parameter('frame_id').value

        self.btn_pub = self.create_publisher(String, 'wiimote/buttons', 10)
        self.imu_pub = self.create_publisher(Imu, 'wiimote/imu', 10)
        self.joy_pub = self.create_publisher(Joy, 'wiimote/joy', 10)
        self.create_subscription(String, 'wiimote/cmd', self._on_cmd, 10)

        self.lock = threading.Lock()
        self.alive = False
        self._generation = 0              # cambia en cada reconexion (descarta hilos viejos)
        self.buttons = set()              # nombres de joy presionados
        self.gyro_raw = [0, 0, 0]
        self.accel_raw = [0, 0, 0]
        self.gyro_seen = False
        self.has_gyro = False
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.calibrating = False
        self.cal_end = None
        self.cal_samples = []
        self._warned_missing = False

        self._try_connect()
        hz = self.get_parameter('imu_rate_hz').value
        self.create_timer(1.0 / hz, self._publish)
        print_hz = self.get_parameter('print_rate_hz').value
        if print_hz > 0:
            self.create_timer(1.0 / print_hz, self._print_gyro)
        self.create_timer(1.0, self._reconnect_if_needed)

    # ---------- conexion ----------
    def _try_connect(self):
        devices = find_wiimote_devices()
        if 'buttons' not in devices:
            if not self._warned_missing:
                self._warned_missing = True
                self.get_logger().warn(
                    'No hay Wiimote. Empareja el Wiimote (bluetoothctl) y revisa que tu usuario '
                    'este en el grupo "input". Sigo buscando cada segundo...')
            return False
        self._warned_missing = False

        with self.lock:
            self._generation += 1
            gen = self._generation
            self.alive = True
            self.buttons = set()
            self.gyro_raw = [0, 0, 0]
            self.accel_raw = [0, 0, 0]
            self.gyro_seen = False
            self.has_gyro = 'gyro' in devices
            self.cal_samples = []
            self.cal_end = None
            self.calibrating = self.has_gyro and self.cal_duration > 0.0
            self.gyro_bias = [0.0, 0.0, 0.0]

        for role in ('buttons', 'accel', 'gyro'):
            if role in devices:
                dev = devices[role]
                self.get_logger().info(f'{role}: {dev.name} ({dev.path})')
                if role == 'buttons':
                    self._init_buttons(dev)
                threading.Thread(target=self._reader, args=(role, dev, gen), daemon=True).start()
            else:
                self.get_logger().warn(
                    f'{role} no esta presente'
                    + (' (sin MotionPlus: el giroscopio sera 0)' if role == 'gyro' else ''))
        self.get_logger().info('Wiimote conectado.')
        return True

    def _init_buttons(self, dev):
        """Botones que ya estaban presionados al conectar."""
        try:
            held = dev.active_keys()
        except OSError:
            return
        with self.lock:
            for code in held:
                if code in BUTTONS:
                    self.buttons.add(BUTTONS[code][1])

    def _reconnect_if_needed(self):
        if not self.alive:
            self._try_connect()

    def _on_lost(self, gen, role):
        with self.lock:
            if gen != self._generation or not self.alive:
                return
            self.alive = False
            self.buttons = set()
            self.gyro_raw = [0, 0, 0]
            self.accel_raw = [0, 0, 0]
            self.calibrating = False
        self.get_logger().error(
            f'Se perdio el Wiimote ({role}). Dejo de publicar; lo reconecto cuando vuelva.')

    def _on_cmd(self, msg):
        if msg.data.strip() == 'calibrate':
            with self.lock:
                if not (self.alive and self.has_gyro):
                    return
                self.calibrating = True
                self.cal_end = None
                self.cal_samples = []
            self.get_logger().info('Recalibrando el giroscopio: deja el Wiimote quieto.')

    # ---------- hilos lectores ----------
    def _reader(self, role, dev, gen):
        try:
            for ev in dev.read_loop():
                if gen != self._generation:
                    return
                if role == 'buttons' and ev.type == ecodes.EV_KEY:
                    self._on_key(ev)
                elif role in ('gyro', 'accel') and ev.type == ecodes.EV_ABS:
                    idx = AXIS_INDEX.get(ev.code)
                    if idx is not None:
                        with self.lock:
                            (self.gyro_raw if role == 'gyro' else self.accel_raw)[idx] = ev.value
                            if role == 'gyro':
                                self.gyro_seen = True
        except OSError:
            pass
        self._on_lost(gen, role)

    def _on_key(self, ev):
        names = BUTTONS.get(ev.code)
        if names is None or ev.value == 2:  # tecla desconocida o auto-repeticion
            return
        text_name, joy_name = names
        with self.lock:
            if ev.value == 1:
                self.buttons.add(joy_name)
            else:
                self.buttons.discard(joy_name)
        text = f'{text_name} {"pressed" if ev.value == 1 else "released"}'
        self.get_logger().info(text)
        self.btn_pub.publish(String(data=text))

    # ---------- calibracion del sesgo del giroscopio ----------
    def _calibration_step(self):
        """Se llama desde el timer de publicacion; promedia el giroscopio en reposo."""
        with self.lock:
            if not self.gyro_seen:
                return
            if self.cal_end is None:
                self.cal_end = time.monotonic() + self.cal_duration
                self.get_logger().info(
                    f'Calibrando el giroscopio {self.cal_duration:.1f} s: '
                    'deja el Wiimote quieto...')
            self.cal_samples.append(list(self.gyro_raw))
            if time.monotonic() < self.cal_end:
                return
            n = len(self.cal_samples)
            self.gyro_bias = [sum(s[i] for s in self.cal_samples) / n for i in range(3)]
            self.cal_samples = []
            self.calibrating = False
            bias = [round(b, 1) for b in self.gyro_bias]
        self.get_logger().info(f'Calibracion lista, sesgo (cuentas) = {bias}')

    # ---------- publicacion ----------
    def _gyro_rad_s(self):
        k = math.radians(self.gyro_scale)
        with self.lock:
            if self.calibrating:
                return [0.0, 0.0, 0.0]
            return [(v - b) * k for v, b in zip(self.gyro_raw, self.gyro_bias)]

    def _publish(self):
        if not self.alive:
            return          # silencio: las interfaces lo toman como "Wiimote perdido"
        if self.calibrating:
            self._calibration_step()
        with self.lock:
            accel = [v / self.accel_counts_per_g * G for v in self.accel_raw]
            pressed = set(self.buttons)
        gyro = self._gyro_rad_s()
        stamp = self.get_clock().now().to_msg()

        joy = Joy()
        joy.header.stamp = stamp
        joy.header.frame_id = self.frame_id
        joy.buttons = [1 if name in pressed else 0 for name in BUTTON_NAMES]
        self.joy_pub.publish(joy)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.frame_id
        imu.orientation_covariance[0] = -1.0  # no entrega orientacion
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = gyro
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = accel
        self.imu_pub.publish(imu)

    def _print_gyro(self):
        if not self.alive or self.calibrating:
            return
        gx, gy, gz = (math.degrees(v) for v in self._gyro_rad_s())
        with self.lock:
            raw = list(self.gyro_raw)
        self.get_logger().info(f'GYRO deg/s  x={gx:8.2f} y={gy:8.2f} z={gz:8.2f}   raw={raw}')


def main(args=None):
    rclpy.init(args=args)
    node = WiimoteNode()
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
