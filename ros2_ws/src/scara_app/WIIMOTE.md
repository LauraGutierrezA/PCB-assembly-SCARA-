# Control con Wiimote

Mueve el SCARA con un Wiimote (Bluetooth al PC del robot). Funciona en la interfaz Qt
(`scara_gui`) y en la web (`scara_web`): pestana **Wiimote**.

## Como usarlo
1. Empareja el Wiimote por Bluetooth (`bluetoothctl`, 1+2) y revisa `groups` (debe incluir `input`).
2. `sudo apt install python3-evdev` y `colcon build --packages-select scara_app`.
3. Terminal 1: `ros2 run scara_app wiimote_node` (deja el Wiimote QUIETO los primeros 2 s: calibra el giroscopio).
4. Terminal 2: `ros2 run scara_app scara_gui` (o `scara_web`) y abre la pestana **Wiimote**.
5. Presiona **Activado**. Arranca en **Simulacion**: no manda nada al robot, solo mueve el gemelo.
6. Cuando el comportamiento te guste y el robot tenga homing: **Robot real**.

## Botones
| Boton | Funcion |
|---|---|
| 1 | Modo IK: D-pad = X/Y (arriba = +X, derecha = +Y, igual que el gemelo (ver PLAN_Y_RIGHT en robot_model.py)), + / - = z |
| 2 | Modo DK: el giro mueve th1 / th2 / z (estilo joystick) |
| B | Dead-man: solo se mueve mientras se mantiene B |
| HOME | Paro de emergencia (manda `stop` y desactiva el control) |
| A | Bomba de vacio: cada pulsacion la enciende o la apaga (en simulacion solo se muestra; HOME la apaga) |

Cambiar de modo, o HOME, exige soltar y volver a presionar B.

## Modo DK (como funciona)
El giro se integra a un angulo RELATIVO que vuelve a 0 cada vez que presionas B. Pasada la
zona muerta (25 grados) la articulacion se mueve con velocidad proporcional al exceso, hasta
la velocidad maxima a 70 grados. Volver al centro la detiene. Giros mas lentos que 2 grados/s
no se integran (ruido / sesgo). Todo se ajusta en vivo en la pestana.

## Seguridad
- Arranca desactivado y en simulacion; al desactivar siempre vuelve a simulacion.
- Se desactiva solo al salir de la pestana, con STOP / Esc, con HOME y al cerrar la ventana/pagina.
- La consigna parte de la posicion MEDIDA al presionar B y no se aleja mas de 10 grados / 10 mm
  del robot real ("correa").
- Si se pierde el Wiimote (> 0.4 s sin `/wiimote/joy`) se suelta el dead-man.
- Solo mueve el robot con homing hecho.

## Primera prueba (verifica los ejes del giroscopio)
1. En simulacion, modo DK, gira el Wiimote y mira que barra (th1 / th2 / z) se mueve.
2. Si una barra no corresponde, cambia el "Eje del giroscopio" (x/y/z) o marca "invertir".
3. Prueba despues con el robot real a velocidades bajas y la mano en Esc.

## Pruebas automaticas (sin robot ni ROS)
`cd ros2_ws/src/scara_app && python3 -m pytest test/test_wiimote_control.py`

## Archivos
- `wiimote_control.py`  logica pura (modos, zona muerta, limites, correa)
- `wiimote_bridge.py`   pega con ROS: dead-man, envios, watchdog del Wiimote
- `wiimote_node.py`     lee el Wiimote (evdev) y publica `/wiimote/joy`, `/wiimote/imu`, `/wiimote/buttons`
- `gui/wiimote_tab.py`, `web/wiimote_tab.py`  pestanas

## Suavizado (modo IK)
- **Suavizado D-pad (rampa)**: segundos que tarda el D-pad / + / - en llegar a velocidad completa
  (y en frenar al soltar). 0 = sin suavizado; sube el valor si el movimiento se siente brusco.
- **Periodo de envio**: cada cuantos ms se manda una consigna nueva al ESP32 (80 por defecto).
  Mas pequeno = movimiento mas continuo, pero mas trafico por el cable serial.
