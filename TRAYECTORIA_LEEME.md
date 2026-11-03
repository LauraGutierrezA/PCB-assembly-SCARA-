# Pestaña "Trayectoria" — interfaz web + firmware

## Qué trae el zip

| Archivo | Nuevo / cambio |
|---|---|
| `ros2_ws/src/scara_app/scara_app/trajectory.py` | **Nuevo.** Ordena los bloques, valida con la IK del firmware y arma los perfiles quínticos |
| `ros2_ws/src/scara_app/scara_app/web/trajectory_tab.py` | **Nuevo.** La pestaña: barra de bloques, Iniciar/Detener, estado, gemelo, dead-man, ejecución del ciclo |
| `ros2_ws/src/scara_app/scara_app/web/trajectory_sandbox.py` | **Nuevo.** Sandbox de bloques (JavaScript + CSS, funciona con el dedo en el iPad) |
| `firmware/components/effector/` | **Nuevo.** Bomba de vacío (GPIO 12) + servo de giro (GPIO 16) |
| `tools/aplicar_trayectoria.py` | Script que agrega las líneas necesarias a 6 archivos que **ya existen** (sin reemplazarlos) |

El script toca: `pins.h`, `control.h`, `control.c`, `main.c`, `web/app.py` y `web/ros_node.py`.
Cada cambio lleva la marca `TRAYECTORIA`, se guarda una copia `.bak` y, si lo corres dos veces, no duplica nada.

## Protocolo nuevo (ESP32)

`/scara/cmd` (texto):
- `vac1` / `vac0`: bomba ON (PICK) / OFF (PLACE)
- `rot +90` / `rot -90`: giro **relativo** del servo, saturado en −90…+90° (arranca en 0°)
- `stop`: además de lo de siempre, **apaga la bomba**

`/scara/limits` agrega: `ARR=0/1` (los 3 motores llegaron a la consigna), `VAC=0/1` y `Q4=<ángulo del servo>`.

GO TO POSE no necesita tópicos nuevos: la pestaña muestrea el perfil a 20 Hz y manda cada punto por `/scara/target`, igual que Punto a punto.

## Perfil generado en el ESP32 (versión 3)

- **Una meta por tramo.** La pestaña ya no manda 20 consignas por segundo: manda una meta por tramo, con 13 valores en `/scara/target`: `[x, y, z, d1..d6, av, codo, T, id]`. El ESP32 interpola cada 8 ms en `components/control/profile.c`: quíntico en th1 y th2, trapezoide en Z.
- **Mezcla de tramos.** Los tramos se mezclan por superposición, así que no se frena en los puntos intermedios. La meta final de cada GO TO POSE se espera con `ARR` (estricto, `DEAD_TOL_COUNTS`).
- **Consignas de 11 valores.** Punto a punto, DK y Wiimote siguen mandando 11 valores, que funcionan como salto, igual que antes.
- **`/scara/limits` agrega** `RST` (controlador compilado), `PG` (avance del tramo, 0–100) y `GS` (id del tramo en curso).
- **Llegada y freno.** Mientras el perfil corre no se aplica la histéresis "llegó": no hay freno ni se borra la memoria del controlador.
- **Feedforward.** En el RST, a la salida del externo se le suma la velocidad de la referencia (`FF_GAIN`).

## Casillas de ajuste (pestaña Trayectoria)

Están debajo de los bloques, a la izquierda de "Despeje Z". Se aplican al presionar **Iniciar trayectoria**. NiceGUI recorta el valor al rango al salir de la casilla, y además se revisa al iniciar.

| Casilla | Rango | Por defecto |
|---|---|---|
| `V_MAX_DEG` [°/s] | 5–275 | 90 |
| `A_MAX_DEG` [°/s²] | 10–1000 | 200 |
| `V_MAX_Z` [mm/s] | 0.1–7.5 | 4.0 |
| `A_MAX_Z` [mm/s²] | 0.5–50 | 15 |
| `FF_GAIN` | 0–1.2 | 1.00 |
| `BLEND_PCT` [%] | 30–100 | 70 |
| `MIN_SEGMENT_S` [s] | 0.05–2 | 0.30 |

- **El tope de 275 °/s** sale de ~230 RPM en la salida de la caja: M1 (banda ×4) llega a 345 °/s y M2 (×3) a 460 °/s, y se deja 20 % de margen sobre el más lento. `controlRST.c` además satura en `W_MAX_RPM_SALIDA = 200 RPM`, que equivale a 300 °/s en M1.
- **`FF_GAIN` se manda en vivo** al ESP32 con el comando `ff <valor>` por `/scara/cmd`, y se confirma con `FF` (en %) en `/scara/limits`.

## Llegada, zona muerta y diagnóstico (versión 5)

- **Tolerancia de llegada por motor** (`DEAD_TOL_COUNTS` / `DEAD_OUT_COUNTS` en `controlRST.c` y `control.c`):
  - M1 y M2: 10/25 cuentas.
  - M3: 50/120 cuentas (0.01/0.025 mm). Antes eran 10 cuentas, es decir 0.002 mm.
- **Compensación de zona muerta en el RST:** mientras el motor no ha llegado, se suma `V_DEAD[i]` (los mismos valores que `control.c`) en el sentido del movimiento. Va por fuera de la memoria del RST.
- **Diagnóstico:** `/scara/limits` agrega `AR=<M1><M2><M3>` (llegó), `EC1..EC3` (error en cuentas) y `U1..U3` (voltaje).
- **El watchdog ya no cancela:** a los 8 s sin llegar avisa en la consola qué motor falta, con su error y su voltaje, y sigue esperando mientras el RST corrige. Se cancela solo con STOP o Detener.

## Z recalibrado y final de carrera (versión 6)

- **Alturas de Z** medidas con la herramienta montada: final DOWN = 0 mm, final UP = 109.2 mm, estacionamiento = 54.6 mm. Están en `homing_pro.c` (`M3_TOP_MM`, `M3_BOTTOM_MM`) y en `robot_model.py` (`JOINT_LIMITS['z']`, `JOINT_HOME`); los dos tienen que coincidir.
- **Velocidad de Z** medida al 100 % de PWM y corregida a la carrera real de 109.2 mm: 9.82 mm/s subiendo y 9.69 mm/s bajando. El tope de `V_MAX_Z` es el 80 % de la más lenta: 7.5 mm/s.
- **Llegada al tope (cambio A):** si el perfil ya terminó y el final de carrera hacia la meta está presionado, el motor se da por llegado. En `AR` aparece un `2` y la consola dice "llegó al final de carrera". Funciona con el PD y con el RST.
- `AV_NOMINAL_MM = 2` en `homing_pro.c` solo se usa para el retroceso del homing. Por la velocidad medida, el tornillo avanza unos 8 mm por vuelta, así que ese retroceso en realidad es de unos 8 mm. No lo cambié porque el homing funciona.

## ROTATE con ángulo libre (versión 7)

- **Ángulo libre y relativo:** el bloque ROTATE tiene una casilla con el ángulo en grados (entero, con signo, de −180 a +180, distinto de 0) y gira **relativo al ángulo actual** del servo. El firmware acepta `rot <n>` con cualquier entero en ese rango.
- **Rango físico del servo:** −90…+90° (180° en total). Al presionar Iniciar se simula el ángulo acumulado, desde `Q4` y a lo largo de todas las repeticiones. Si algún ROTATE lo sacaría de ese rango, el bloque se marca en rojo y el ciclo no arranca, en lugar de quedarse quieto sin avisar.
- **Bloques viejos** con +90/−90 (`dir`) se siguen leyendo.

## Cómo se usa

1. Para agregar un bloque al final de la cadena (queda conectado solo), **tócalo** en la barra de arriba. Si lo **arrastras**, queda suelto donde lo sueltes.
2. Para conectar dos bloques, arrastra desde el **punto de salida** (a la derecha del bloque) hasta el bloque siguiente. La (×) sobre una flecha la borra.
3. El `#n` de cada bloque es su orden de ejecución, e `INICIO` marca el primero. Los bloques sin conectar se ven tenues y no se ejecutan.
4. Si tocas un GO TO POSE, su pose aparece en el gemelo (cian). Si no la ves, el punto no es alcanzable.
5. Al presionar **Iniciar trayectoria**, se valida todo antes de mover el robot. Un error marca el bloque en rojo y aparece en la consola.
6. Para que el ciclo avance, **mantén el dead-man** (el botón o la barra Espacio). Si lo sueltas, el ciclo se pausa; al volver a mantenerlo sigue desde el mismo punto.
7. El ciclo se cancela con STOP, Esc, Detener, al cambiar de pestaña, si se pierde el ESP32 o si el firmware rechaza una consigna.

El diagrama se guarda solo en el navegador de cada pantalla. El botón "Ejemplo" vuelve a cargar el ciclo de tu imagen.

## Ajustes que dependen del robot real

En `scara_app/trajectory.py`:
- `V_MAX_DEG = 45`, `A_MAX_DEG = 90`: velocidad y aceleración máximas de th1 y th2.
- `V_MAX_Z = 2.0 mm/s` (pico del trapezoide), `A_MAX_Z = 4`: el máximo físico es ≈ 2.2 mm/s (200 RPM, reducción 50·3, tornillo de 2 mm/vuelta). **Mídelo**: el eje Z es el que más pesa en la duración del ciclo.
- `Z_CLEAR_DEFAULT = 5 mm`: despeje sobre la más alta de las dos z (inicio y meta).
- `BLEND_PCT = 70`: % de avance de un tramo intermedio en que se manda el siguiente (más bajo = más mezcla, la esquina se redondea más).
- `TRAP_FRAC = 0.2`: debe ser igual a `PROF_TRAP_FRAC` en `profile.h`.
- En `controlRST.c`: `FF_GAIN = 1.0` (feedforward de velocidad). Si ves sobrepaso al final de los tramos, bájalo a 0.7–0.9.

En `trajectory_tab.py`:
- `SETTLE_TIMEOUT_S = 8`: tiempo máximo (con el dead-man presionado) para que llegue `ARR=1`.

En `effector.c`:
- `SERVO_US_MIN/MAX` (500/2400 µs) y `SERVO_INVERT`.
- `VACUUM_ACTIVE_LEVEL`: ponlo en 0 si tu módulo se activa con BAJO.

Los parámetros d1…av son los `ROBOT_DEFAULTS` de Punto a punto.

## Hardware

- **GPIO 12 (bomba)** es un pin de arranque: debe estar en BAJO al encender. Pon un MOSFET con **pull-down de 10 kΩ en la compuerta** y diodo flyback en la bomba. No sirve un módulo de relé con pull-up, porque el ESP32 no arrancaría.
- **GPIO 16 (servo):** alimenta el servo a 5 V aparte, con GND común con el ESP32. La señal de 3.3 V le sirve al SG90.
- El GPIO 16 era el pin sugerido para un dead-man físico; si algún día lo pones, usa el GPIO 2.

## Si el script dice `[MANUAL]`

Significa que tu archivo local no tiene el texto de referencia. En ese caso aplica a mano solo ese cambio:

**pins.h**, antes de `#endif /* PINS_H */`:
```c
#define PIN_SERVO     16
#define PIN_VACUUM    12
```
**control.h**, antes de `#endif`: `bool control_arrived(void);`

**control.c**, al final:
```c
bool control_arrived(void)
{
    bool ok;
    portENTER_CRITICAL(&ref_lock);
    ok = has_ref && llego[0] && llego[1] && llego[2];
    portEXIT_CRITICAL(&ref_lock);
    return ok;
}
```
**main.c**:
- `#include "effector.h"`.
- En `app_main()`, después de `encoders_init();`: `effector_init();`.
- Primera línea de `cmd_callback` (después de declarar `m`): `if (effector_handle_cmd(m->data.data, m->data.size)) return;`.
- En `limits_publish_tick()`, justo después del `snprintf(limits_buf, ...)`:
```c
{
    size_t len_ = strlen(limits_buf);
    snprintf(limits_buf + len_, sizeof(limits_buf) - len_, " ARR=%d VAC=%d Q4=%d",
             control_arrived() ? 1 : 0, effector_vacuum_on() ? 1 : 0, effector_servo_deg());
}
```
**web/app.py**:
- `from .trajectory_tab import TrajectoryTab`.
- En `TABS`: `('traj', 'Trayectoria', TrajectoryTab),`.
- En `on_tab_change`, después de `name = ...`:
```python
for page in pages.values():
    if hasattr(page, 'on_tab_change'):
        page.on_tab_change(name)
```
**web/ros_node.py**:
- Al final de `emergency_stop()`: `self._push('estop', None)`.
- Copia los métodos `stream_target`, `vacuum` y `rotate` del script (`tools/aplicar_trayectoria.py`, función `patch_ros_node`).

## Pruebas sugeridas (en este orden)

1. Sin motores: `ros2 topic pub --once /scara/cmd std_msgs/String "data: vac1"` → la bomba enciende. Luego `rot +90` y `rot -90`.
2. `ros2 topic echo /scara/limits`: deben aparecer `ARR`, `VAC` y `Q4`.
3. Ciclo con un solo GO TO POSE sin despeje Z, con `U_MAX_V` bajo.
4. Ejemplo completo con 1 repetición; después súbele las repeticiones.
