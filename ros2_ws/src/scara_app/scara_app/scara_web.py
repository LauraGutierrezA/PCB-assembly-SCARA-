"""
scara_web.py - Punto de entrada de la interfaz WEB (para iPad, PC o celular).

    ros2 run scara_app scara_web

Luego, en Safari del iPad (misma red WiFi que el PC):
    http://<IP-del-PC>:8080          (la IP sale con: hostname -I)

Hilos:
  - Hilo principal: servidor web NiceGUI (uvicorn).
  - Hilo "ros":     executor.spin() con el nodo ScaraWebNode.
El nodo y la interfaz se comunican con la lista de eventos del nodo
(ver web/ros_node.py), asi ningun callback de ROS toca la interfaz.

La version de escritorio (ros2 run scara_app scara_gui) sigue funcionando
igual; incluso pueden correr las dos a la vez (nodos con nombres distintos).

Puerto: variable de entorno SCARA_WEB_PORT (por defecto 8080).
"""
import os
import threading

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from nicegui import app, ui

from scara_app.web.app import build_ui
from scara_app.web.ros_node import ScaraWebNode

HOST = '0.0.0.0'   # escuchar en todas las interfaces (WiFi incluida)
PORT = int(os.environ.get('SCARA_WEB_PORT', '8080'))


def main(args=None):
    # Ctrl+C lo maneja el servidor web, no rclpy
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ScaraWebNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    threading.Thread(target=spin, name='ros', daemon=True).start()

    def shutdown():
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    app.on_shutdown(shutdown)
    build_ui(node)
    ui.run(host=HOST, port=PORT, title='SCARA RRP - Estacion 3', dark=True,
           reload=False, show=False, favicon='🦾')


if __name__ == '__main__':
    main()
