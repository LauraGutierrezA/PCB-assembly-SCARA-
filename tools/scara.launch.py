"""
scara.launch.py - Lanza TODO el SCARA desde una sola terminal.

    source /opt/ros/jazzy/setup.bash
    ros2 launch ~/scara_ws/tools/scara.launch.py zip:=~/Downloads/Observadores_v1.zip

Reemplaza las 3 terminales:
  1. (antes Terminal 1 + inicio de la 3) tools/scara_prep.sh:
       descomprime el zip (si se pasa), touch, idf.py build, idf.py flash,
       colcon build scara_app.  Si algo falla, se detiene todo ahi.
  2. Cuando el paso 1 termina BIEN (el agente no puede estar abierto mientras
     se flashea: usan el mismo puerto):
       - agente micro-ROS:  micro_ros_agent serial --dev <puerto> -b <baud>
       - interfaz web:      ros2 run scara_app scara_web
  Ctrl+C cierra todo. Si el agente o la web se caen, se cierra lo demas.

Argumentos (todos opcionales):
  zip:=<ruta>          zip con cambios para ~/scara_ws (vacio = no descomprimir)
  flash:=true|false    compilar y flashear el ESP32 (false = solo agente + web)
  web:=true|false      colcon build de scara_app antes de abrir la web
  port:=/dev/ttyUSB0   puerto del ESP32 (flash y agente)
  baud:=115200
  idf:=<export.sh>     export.sh de ESP-IDF (vacio = $IDF_PATH o ~/esp/esp-idf)
  ws:=~/scara_ws
  twin:=none|rviz|gazebo   GEMELO digital (paquetes scara_description y scara_twin):
                       rviz   -> ros2 launch scara_twin twin.launch.py
                       gazebo -> ros2 launch scara_twin twin_gazebo.launch.py
                       (none = como antes; con rviz/gazebo, el paso 1 tambien
                        compila scara_description y scara_twin)

Este archivo NO esta dentro del paquete scara_app a proposito: asi puede
compilar scara_app antes de abrir la web (y no hay que compilar para usarlo).
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

HERE = os.path.dirname(os.path.realpath(__file__))


def _expand(path):
    return os.path.expanduser(path) if path else path


def _setup(context):
    arg = {k: LaunchConfiguration(k).perform(context)
           for k in ('zip', 'flash', 'web', 'port', 'baud', 'idf', 'ws', 'twin')}
    twin_mode = arg['twin'].strip().lower()
    if twin_mode not in ('none', 'rviz', 'gazebo'):
        return [LogInfo(msg=f'[scara] twin:={arg["twin"]} no vale (none, rviz o gazebo).'),
                EmitEvent(event=Shutdown(reason='twin invalido'))]
    ws = _expand(arg['ws'])
    # Por variables de entorno: ExecuteProcess descarta los argumentos vacios
    env = {'SCARA_WS': ws, 'SCARA_ZIP': _expand(arg['zip']), 'SCARA_FLASH': arg['flash'],
           'SCARA_WEB': arg['web'], 'SCARA_PORT': arg['port'], 'SCARA_IDF': _expand(arg['idf']),
           'SCARA_TWIN': twin_mode}

    # ---------- 1. firmware + colcon ----------
    prep = ExecuteProcess(
        name='prep', cmd=['bash', os.path.join(HERE, 'scara_prep.sh')],
        additional_env=env, output='screen', emulate_tty=True)

    # ---------- 2. agente micro-ROS ----------
    agent = ExecuteProcess(
        name='micro_ros_agent',
        cmd=['bash', '-c',
             f'source "{ws}/microros_ws/install/local_setup.bash" && '
             f'exec ros2 run micro_ros_agent micro_ros_agent serial '
             f'--dev {arg["port"]} -b {arg["baud"]}'],
        output='screen', emulate_tty=True)

    # ---------- 3. interfaz web ----------
    web = ExecuteProcess(
        name='scara_web',
        cmd=['bash', '-c',
             f'source "{ws}/ros2_ws/install/setup.bash" && exec ros2 run scara_app scara_web'],
        additional_env={'PYTHONUNBUFFERED': '1'}, output='screen', emulate_tty=True)

    # ---------- 4. GEMELO digital (opcional) ----------
    twin_launch = 'twin_gazebo.launch.py' if twin_mode == 'gazebo' else 'twin.launch.py'
    twin = ExecuteProcess(
        name='gemelo',
        cmd=['bash', '-c',
             f'source "{ws}/ros2_ws/install/setup.bash" && '
             f'exec ros2 launch scara_twin {twin_launch}'],
        output='screen', emulate_tty=True)

    def after_prep(event, _context):
        if event.returncode != 0:
            return [LogInfo(msg='[scara] Fallo la preparacion (ver arriba). No se abre nada.'),
                    EmitEvent(event=Shutdown(reason='prep fallo'))]
        return [LogInfo(msg='[scara] Abriendo agente micro-ROS...'),
                agent,
                # La web un poco despues, para que el agente ya tenga el puerto
                TimerAction(period=2.0, actions=[
                    LogInfo(msg='[scara] Interfaz web: http://<IP-del-PC>:8080 '
                                '(la IP sale con: hostname -I)'),
                    web]),
                *([TimerAction(period=4.0, actions=[
                    LogInfo(msg=f'[scara] Gemelo digital ({twin_mode})...'), twin])]
                  if twin_mode != 'none' else [])]

    def bye(name):
        return lambda _event, _context: [
            LogInfo(msg=f'[scara] Se cerro {name}: cerrando todo.'),
            EmitEvent(event=Shutdown(reason=f'{name} termino'))]

    return [
        LogInfo(msg=f'[scara] ws={ws}  zip={arg["zip"] or "(ninguno)"}  flash={arg["flash"]}  '
                    f'web={arg["web"]}  port={arg["port"]}  twin={twin_mode}'),
        RegisterEventHandler(OnProcessExit(target_action=prep, on_exit=after_prep)),
        RegisterEventHandler(OnProcessExit(target_action=agent, on_exit=bye('el agente'))),
        RegisterEventHandler(OnProcessExit(target_action=web, on_exit=bye('la web'))),
        prep,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('zip', default_value='',
                              description='zip con cambios a descomprimir en ws (vacio = ninguno)'),
        DeclareLaunchArgument('flash', default_value='true',
                              description='compilar y flashear el ESP32'),
        DeclareLaunchArgument('web', default_value='true',
                              description='colcon build de scara_app antes de abrir la web'),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('idf', default_value='',
                              description='ruta a export.sh de ESP-IDF'),
        DeclareLaunchArgument('ws', default_value='~/scara_ws'),
        DeclareLaunchArgument('twin', default_value='none',
                              description='gemelo digital: none, rviz o gazebo'),
        OpaqueFunction(function=_setup),
    ])
