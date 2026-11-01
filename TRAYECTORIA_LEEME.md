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
- `V_MAX_Z = 1.8 mm/s`, `A_MAX_Z = 3`: los estimé con 200 RPM, la reducción 50·3 y el tornillo de 2 mm/vuelta, que dan ≈ 2.2 mm/s. **Mídelo**: el eje Z es el que más pesa en la duración del ciclo.
- `Z_SAFE_DEFAULT = 50 mm`.

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
3. Ciclo con un solo GO TO POSE sin Z segura, con `U_MAX_V` bajo.
4. Ejemplo completo con 1 repetición; después súbele las repeticiones.
