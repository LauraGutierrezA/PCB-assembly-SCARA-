# Pestaña "Observadores": interfaz web + firmware

Hecho sobre el commit `cdc61c7` ("Trayectorias listas").

## Qué trae el zip

| Archivo | Nuevo / cambio |
|---|---|
| `firmware/components/observers/` | **Nuevo.** Luenberger + Kalman de los 3 motores (cada 8 ms) |
| `firmware/components/motors/motors.c`, `include/motors.h` | `motor_get_volts(i)`: voltaje que se le está aplicando a cada motor, lo escriba quien lo escriba (control, homing o prueba de circuito) |
| `firmware/main/main.c` | Llama a los observadores en la tarea de control, publica `/scara/obs` y atiende el latido `obs1` |
| `ros2_ws/.../web/observers_tab.py` | **Nuevo.** La pestaña con las 6 gráficas |
| `ros2_ws/.../web/app.py` | Agrega la pestaña y el *bypass* |
| `ros2_ws/.../web/ros_node.py` | Suscripción a `/scara/obs` y el latido |
| `ros2_ws/.../web/theme.py` | CSS de la pestaña |

Cada cambio en un archivo existente lleva la marca `OBSERVADORES`.

## Cómo funciona

- **ESP32:** en cada ciclo de 8 ms, antes de que el control escriba el voltaje nuevo, cada motor actualiza los dos observadores con:
  - `u(k-1)`: el voltaje aplicado en el periodo anterior.
  - `y(k)`: la velocidad del encoder (derivada pura, con el signo del homingPRO, igual que `controlRST.c`).
- **Simplificación respecto al Lab 4:** L (Ackermann, polos `z^fac`) y K se calculan una sola vez al arrancar. El Kalman usa la ganancia de **estado estacionario**: con Q y R fijos, la K del Lab 4 converge a la misma en unas 20 muestras (160 ms). En tiempo real cada observador cuesta unas 10 multiplicaciones en float. Lo comparé contra `estimators.cpp` del Lab 4 en el PC:
  - Luenberger: diferencia de 1e-4 rad/s.
  - Kalman: diferencia menor a 1e-6 A después de los primeros 100 ms.
- **Publicación:** `/scara/obs` (Float32MultiArray, best effort) manda 16 valores cada 40 ms, que son el promedio de 5 muestras: `[n, M1: w_med w_luen w_kal i_luen i_kal, M2…, M3…]`. El UART va a 115200, así que no hay ancho de banda para mandar a 125 Hz.
- **Solo se publica mientras alguien ve la pestaña:** la página manda `obs1` cada 1 s y, sin latido durante 2.5 s, el ESP32 deja de publicar. Así no se le quita ancho de banda a Trayectoria cuando nadie mira las gráficas.
- **Gráficas:**
  - Corriente en A.
  - Velocidad en RPM del eje de **salida** (como la página del Lab 4). Para pasarla a rad/s del motor, cambia `RAD_S_TO_RPM_OUT` en `observers_tab.py`.
  - Ventana de 5 s y redibujo a 5 Hz, con botones de **Pausar** y **Limpiar**.

## Bypass

Entrar a **Observadores** no suelta el dead-man, no cancela la Trayectoria y no desactiva el Wiimote. La pestaña desde la que llegaste queda como "dueña" y se muestra en *Sigue corriendo*.

- Si llegaste desde Trayectoria, Punto a punto o DK, en Observadores aparece el mismo botón **dead-man** (o la barra Espacio). Así puedes seguir moviendo el robot mientras ves las gráficas.
- Con el Wiimote, el dead-man es el botón B, como siempre.
- Si vuelves a la pestaña dueña, todo sigue igual. Si te vas a otra pestaña, se aplica la regla normal: se cancela la Trayectoria o se desactiva el Wiimote.
- STOP y Esc funcionan igual en todas las pestañas.

## Dónde cambiar los parámetros de cada motor

En `firmware/components/observers/observers.c`, la tabla `OBS_PARAMS[]` tiene una fila por motor con `{A, B, fac, Q11, Q22, R}`. Hoy las tres filas usan `OBS_LAB4_MOTOR2`, que son los valores del motor 2 del Lab 4. Si identificas cada motor, cambias su fila y L y K se recalculan solas al arrancar.

## Notas

- Los observadores usan `homing_pro_dir_sign()`. Antes del homing ese signo vale +1 y, si el encoder cuenta al revés, las estimaciones divergen. Después del homing quedan bien.
- Al terminar el homing los encoders vuelven a cero. Ese salto se detecta y se reinician los observadores para que no aparezca un pico en la gráfica.
- En freno o libre (coast) se toma u = 0 V.
- El ESP32 queda con 4 publicadores (`RMW_UXRCE_MAX_PUBLISHERS=4`), así que no hay que recompilar la librería de micro-ROS.
