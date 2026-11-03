/*
 * observers.h - Luenberger y Kalman para los 3 motores (pestana "Observadores").
 *
 * Modelo de cada motor (el mismo del Lab 4, FINALlab4_observers):
 *     x(k) = A x(k-1) + B u(k-1),   y(k) = C x(k),   C = [0 1]
 *     x = [ i_a  (A) ;  omega (rad/s del EJE DEL MOTOR) ],   u = voltaje [V]
 * discretizado con ZOH a Ts = 8 ms = CONTROL_LOOP_MS (la tarea de control).
 *
 * Simplificacion frente al Lab 4 (para que quepa sin problema en el ESP32):
 *   - Las ganancias L (Luenberger, Ackermann con polos z_planta^fac) y K
 *     (Kalman) se calculan UNA vez al arrancar, en double.
 *   - El Kalman usa la ganancia de ESTADO ESTACIONARIO: con Q y R fijos, la
 *     K del Lab 4 converge en ~20 muestras (160 ms) a la misma K. Asi no hay
 *     que propagar P en cada paso.
 *   - En tiempo real cada observador es ~10 multiplicaciones en float.
 *
 * Los parametros (A, B, fac, Q, R) de cada motor estan en la tabla
 * OBS_PARAMS[] de observers.c. Por ahora los 3 motores usan los del motor 2
 * del Lab 4.
 *
 * Entradas (las toma solo):
 *   u(k-1) = voltaje aplicado en el periodo anterior (motor_get_volts)
 *   y(k)   = omega medida = delta de cuentas del encoder / Ts (derivada pura,
 *            con el signo del homingPRO, igual que controlRST.c)
 *
 * Uso:
 *   observers_init()    una vez (app_main), antes de la tarea de control
 *   observers_update()  cada CONTROL_LOOP_MS, al INICIO del ciclo de la
 *                       tarea de control (antes de escribir el voltaje nuevo)
 *   observers_keepalive() / observers_streaming() / observers_take()
 *                       los usa main.c para publicar en /scara/obs
 */
#ifndef OBSERVERS_H
#define OBSERVERS_H

#include <stdbool.h>
#include <stdint.h>

#define OBS_NUM_MOTORS   3

/* Se publica el PROMEDIO de OBS_DECIM muestras de 8 ms: 5 -> cada 40 ms
 * (25 Hz). El UART de micro-ROS va a 115200 baud, no da para 125 Hz. */
#define OBS_DECIM        5

/* La pagina manda "obs1" cada 1 s mientras alguien ve la pestana. Sin
 * latido en este tiempo, el ESP32 deja de publicar /scara/obs. */
#define OBS_KEEPALIVE_MS 2500

/* Valores por motor en /scara/obs (despues del contador):
 *   [w_med, w_luen, w_kal, i_luen, i_kal]   (rad/s del motor y A) */
#define OBS_VALS_PER_MOTOR  5
#define OBS_MSG_LEN         (1 + OBS_NUM_MOTORS * OBS_VALS_PER_MOTOR)   /* 16 */

void observers_init(void);
void observers_update(void);

/* "obs1" desde la interfaz */
void observers_keepalive(void);
/* true si hubo latido en los ultimos OBS_KEEPALIVE_MS */
bool observers_streaming(void);

/* Copia la ultima muestra promediada en out[OBS_MSG_LEN]. out[0] es un
 * contador (sube 1 cada 40 ms). Devuelve false si no hay una nueva desde la
 * ultima llamada. */
bool observers_take(float out[OBS_MSG_LEN]);

#endif /* OBSERVERS_H */
