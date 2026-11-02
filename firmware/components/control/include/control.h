/*
 * control.h - Control de posicion de las 3 articulaciones (un lazo por motor).
 *
 * Cada motor tiene su propio controlador discreto D(z), disenado en MATLAB
 * (NPI_Proyecto.mlx, pole_assignment "PD") sobre la planta
 *   Gpos(s) = theta_motor(s) / Va(s)      [rad del EJE DEL MOTOR / V]
 * discretizada con ZOH a CONTROL_TS_MS.
 *
 * Por eso el lazo trabaja en esas mismas unidades:
 *   error  e[k] = referencia - posicion   [rad del eje del motor]
 *   salida u[k]                            [V]  -> se convierte a PWM
 *
 * Los coeficientes de D(z) se pegan en control.c (CTRL_NUM / CTRL_DEN),
 * tal cual los da MATLAB con  [num, den] = tfdata(Dz, 'v').
 *
 * Dead-man switch (hold-to-run):
 *   El lazo SOLO mueve los motores mientras la interfaz confirma que el
 *   operador tiene el dedo en el boton (o la barra espaciadora). La GUI
 *   manda un "latido" (control_deadman_beat) cada ~100 ms; si pasan
 *   DEADMAN_TIMEOUT_MS sin latido (se solto el boton, se cayo el WiFi, se
 *   cerro la GUI...), los motores se detienen. La consigna se conserva: al
 *   volver a presionar, el robot sigue hacia ella.
 *
 * Uso entre tareas (seguro):
 *   - control_set_reference() se llama desde micro-ROS (nucleo 0).
 *   - control_update() la llama la tarea de control (nucleo 1) cada
 *     CONTROL_LOOP_MS; el controlador se ejecuta cada CONTROL_TS_MS.
 */
#ifndef CONTROL_H
#define CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include "ik.h"

/* ============================================================
 *   SELECTOR DE CONTROLADOR  ("boton": 0 = PD, 1 = RST cascada)
 * ============================================================
 *   0 -> control.c     : 3 PD de posicion (Ts = CONTROL_TS_MS)
 *   1 -> controlRST.c  : 3 RST en cascada
 *                        interno velocidad (incremental, Ts = CONTROL_LOOP_MS)
 *                        externo posicion  (posicional,  Ts = 8 ms tambien,
 *                                           ver RST_OUT_EVERY en controlRST.c)
 * Los dos archivos implementan esta misma API, asi que main.c (IK,
 * trayectorias, dead-man...) no cambia. Solo se compila el elegido.
 * Cambiar el valor y volver a compilar/flashear (idf.py build flash).
 */
#define CONTROL_USE_RST   1

/* Periodo de la tarea de control (homing, prueba de circuito, control). */
#define CONTROL_LOOP_MS   8

/* Tiempo de muestreo del controlador: el MISMO Ts del diseno en MATLAB.
 * Debe ser multiplo de CONTROL_LOOP_MS. */
#define CONTROL_TS_MS     32

/* Nueva referencia articular. OJO, unidades del control:
 *   th1, th2 -> rad      th3 -> altura en mm desde el piso (NO el angulo
 *   del tornillo de la IK; main.c pasa ahi la z de la consigna). */
void control_set_reference(const joints_t *ref);

/* Copia la referencia actual. Devuelve false si no hay una valida. */
bool control_get_reference(joints_t *ref);

/* Contador de referencias recibidas (cambia con cada consigna nueva). */
uint32_t control_reference_seq(void);

/* Sin latido en este tiempo, el dead-man se considera suelto. */
#define DEADMAN_TIMEOUT_MS  300

/* Dead-man presionado: llamarla cada < DEADMAN_TIMEOUT_MS mientras se mantenga. */
void control_deadman_beat(void);

/* Dead-man suelto: los motores se detienen de inmediato. */
void control_deadman_release(void);

/* Borra la consigna: el robot se queda donde esta hasta una consigna nueva
 * (Stop, entrar a "Prueba de circuito", consigna rechazada). */
void control_release(void);

/* Un paso de la tarea de control. */
void control_update(void);

/* Posicion medida, en las mismas unidades que la referencia (th1, th2 en
 * rad; th3 en mm desde el piso). Vale 0 antes del homingPRO. */
void control_get_position(joints_t *pos);

/* TRAYECTORIA: true si hay consigna, el perfil (profile.c) ya termino y los
 * 3 motores llegaron a ella (histeresis "llego" del lazo, DEAD_TOL_COUNTS).
 * Se publica como ARR en /scara/limits. */
bool control_arrived(void);

/* TRAYECTORIA: meta con perfil generado en el ESP32 (profile.h). Desde la
 * posicion/estado actual hasta goal en duration_s segundos. id: numero del
 * tramo que manda el PC (un reenvio con el mismo id se ignora).
 * Mientras el perfil corre NO se aplica la histeresis "llego" (no se frena
 * ni se borra la memoria del controlador); solo al terminar. */
void control_set_goal(const joints_t *goal, float duration_s, uint32_t id);

/* TRAYECTORIA: ganancia del feedforward de velocidad (solo la usa el RST).
 * La pestana Trayectoria la manda con "ff <valor>" por /scara/cmd; se
 * satura a [FF_GAIN_MIN, FF_GAIN_MAX] y se publica como FF (en %) en
 * /scara/limits. */
#define FF_GAIN_MIN  0.0f
#define FF_GAIN_MAX  1.2f
void  control_set_ff_gain(float g);
float control_get_ff_gain(void);

#endif /* CONTROL_H */
