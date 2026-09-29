"""
scara_gui.py - Punto de entrada de la interfaz grafica.

    ros2 run scara_app scara_gui

ROS 2 y Qt corren en el mismo hilo: un QTimer llama rclpy.spin_once cada
20 ms. Asi los callbacks de ROS pueden tocar la interfaz sin problemas de
hilos.
"""
import signal
import sys

import rclpy
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication

from scara_app.gui.main_window import MainWindow
from scara_app.gui.ros_node import ScaraGuiNode

SPIN_PERIOD_MS = 20


def main(args=None):
    rclpy.init(args=args)
    node = ScaraGuiNode()

    app = QApplication(sys.argv)
    window = MainWindow(node)
    window.show()

    spin_timer = QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    spin_timer.start(SPIN_PERIOD_MS)

    # Permite cerrar con Ctrl+C desde la terminal
    signal.signal(signal.SIGINT, lambda *_: app.quit())

    try:
        exit_code = app.exec_()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
