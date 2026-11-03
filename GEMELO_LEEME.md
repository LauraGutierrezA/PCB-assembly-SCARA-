# Gemelo digital del SCARA (RViz y Gazebo) + arreglo de Trayectoria

Hecho sobre `main` en el commit `46369e2`. Los cambios a archivos que ya existían llevan la marca `GEMELO`.

## Qué trae el zip

| Archivo | Nuevo / cambio |
|---|---|
| `ros2_ws/src/scara_description/` | **Nuevo.** Modelo (`urdf/scara.urdf.xacro`), RViz, controladores de Gazebo y `display.launch.py` |
| `ros2_ws/src/scara_twin/` | **Nuevo.** Puente robot → gemelo (`bridge`), robot falso (`fake_scara`), `twin.launch.py` (RViz), `twin_gazebo.launch.py` (Gazebo + RViz) y pruebas |
| `ros2_ws/src/scara_app/scara_app/web/ros_node.py` | **Arreglo de Trayectoria:** agrega `send_goal()` y `set_ff_gain()` (ver abajo) |
| `tools/scara.launch.py`, `tools/scara_prep.sh` | Argumento nuevo `twin:=none\|rviz\|gazebo` |
| `tools/gemelo/fase4_scara_joints.patch` | Opcional: `/scara/joints` a 25 Hz desde el firmware |

## Por qué no funcionaba Trayectoria

Desde la versión 3 (perfil en el ESP32), `trajectory_tab.py` llama `node.send_goal(...)` y `node.set_ff_gain(...)`. Esos dos métodos **nunca quedaron en ningún commit** de `web/ros_node.py`. El zip de Observadores se hizo sobre `cdc61c7` y su `ros_node.py` reemplazó la copia local que sí los tenía. Como el primer paso de cada ciclo es fijar `FF_GAIN`, la trayectoria fallaba apenas se presionaba **Iniciar**.

El firmware no tenía nada que ver: lo único que cambió desde `cdc61c7` son los observadores, que no tocan el control.

> **Ojo:** no vuelvan a usar `zip:=…/Observadores_v1.zip`, porque volvería a pisar `ros_node.py`.

## Requisitos (una vez)

```bash
sudo apt install ros-jazzy-xacro ros-jazzy-robot-state-publisher \
                 ros-jazzy-joint-state-publisher-gui ros-jazzy-rviz2
# Solo para Gazebo:
sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers
```

## Cómo se corre

```bash
unzip -o ~/Downloads/gemelo_scara_v2.zip -d ~/scara_ws
source /opt/ros/jazzy/setup.bash

ros2 launch ~/scara_ws/tools/scara.launch.py                  # igual que antes (sin gemelo)
ros2 launch ~/scara_ws/tools/scara.launch.py twin:=rviz       # + gemelo en RViz
ros2 launch ~/scara_ws/tools/scara.launch.py twin:=gazebo     # + gemelo en Gazebo y RViz
```

- Se descomprime **a mano**, no con `zip:=`, porque el launch que se ejecuta tiene que ser el nuevo (el que entiende `twin:=`).
- Con `twin:=rviz` o `twin:=gazebo`, el paso 4 compila `scara_app scara_description scara_twin`. Con `web:=false` necesita que el gemelo ya esté compilado de una vez anterior.
- Si se cierra RViz o Gazebo, el agente y la web siguen corriendo.

### Solo el gemelo, sin robot

```bash
cd ~/scara_ws/ros2_ws
colcon build --packages-select scara_description scara_twin && source install/setup.bash
ros2 launch scara_description display.launch.py        # modelo con sliders
ros2 launch scara_twin twin.launch.py fake:=true        # robot falso que se mueve solo
ros2 launch scara_twin twin_gazebo.launch.py fake:=true
```

## Cómo funciona

```
encoders → ESP32 → /scara/limits (~10 Hz, texto)  ┐
                 → /scara/joints (25 Hz, fase 4)  ┴→ bridge (suaviza, 30 Hz)
                                                       ├→ /joint_states → RViz
                                                       └→ /forward_position_controller/commands → Gazebo
```

| `/scara/limits` | Gemelo |
|---|---|
| `Q1` [°] | `joint1` = Q1·π/180 (±85°) |
| `Q2` [°] | `joint2` = Q2·π/180 (±90°) |
| `Q3` [mm sobre el piso] | `joint3` = (Q3 − 202)/1000 m, con **Q3 entre 0 y 109.2 mm** |

- Los límites de Z son los de la versión 6 (`M3_BOTTOM_MM = 0`, `M3_TOP_MM = 109.2`, estacionamiento en 54.6 mm). Están en `scara_twin/conversions.py` y en el xacro (`z_min`, `z_max` y el `initial_value` de `joint3`). Si cambian en `homing_pro.c` y `robot_model.py`, también hay que cambiarlos aquí.
- Hasta que llegue `CAL=1`, el gemelo se queda en la posición de estacionamiento.
- El puente **solo lee**: no publica en `/scara/cmd` ni en `/scara/target`.

## Fase 4 (opcional): `/scara/joints` a 25 Hz

Sin la fase 4 el gemelo funciona igual, pero se actualiza a ~10 Hz. Para usarla:

```bash
cd ~/scara_ws
git apply tools/gemelo/fase4_scara_joints.patch      # main.c + app-colcon.meta
# En una terminal SIN ROS, con ESP-IDF cargado:
cd firmware && idf.py clean-microros && idf.py build && idf.py -p /dev/ttyUSB0 flash
```

- Es el **5.º publicador**, así que el parche sube `RMW_UXRCE_MAX_PUBLISHERS` a 5. Por eso `idf.py clean-microros` es obligatorio: si no se recompila la librería de micro-ROS, el ESP32 **no se conecta al agente**.
- El executor pasa a 6 handles (3 suscripciones y 3 timers: status, observadores y gemelo).
- Hay que hacerlo en una terminal aparte. El launch con `flash:=true` no hace `clean-microros`.
- Ancho de banda: el UART va a 115200 (~11 KB/s). `/scara/joints` usa ~1.3 KB/s, `/scara/limits` hasta ~3 KB/s y `/scara/obs` ~2.5 KB/s solo con esa pestaña abierta.
- El firmware con el parche **no se compiló** donde se hizo este zip, porque aquí no hay ESP-IDF.

## Qué se verificó y qué no

- Verificado:
  - Las 13 pruebas de `scara_twin`.
  - El xacro, con y sin `use_gz`: límites de `joint3` de −0.202 a −0.0928 m, arranque en −0.1474 m, y la punta a Q3/1000 m.
  - `send_goal` publica los 13 valores en el orden del firmware, y `ff 0.850` es lo que lee el `sscanf` del ESP32.
  - El planificador con el programa de ejemplo.
  - `scara.launch.py` y `scara_prep.sh` con `twin:=none/rviz/gazebo`, simulando `launch` y `colcon`.
  - El parche de la fase 4 aplica sobre `main`.
- **No verificado:** RViz, Gazebo, el firmware con la fase 4 y la trayectoria con el robot real.
