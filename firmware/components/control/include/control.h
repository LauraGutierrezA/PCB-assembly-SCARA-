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

/* Periodo de la tarea de control (homing, prueba de circuito, control). */
#define CONTROL_LOOP_MS   8

/* Tiempo de muestreo del controlador: el MISMO Ts del diseno en MATLAB.
 * Debe ser multiplo de CONTROL_LOOP_MS. */
#define CONTROL_TS_MS     32

/* Nueva referencia articular [rad] (th3 = angulo del motor del tornillo). */
void control_set_reference(const joints_t *ref);

/* Copia la referencia actual. Devuelve false si no hay una valida. */
bool control_get_reference(joints_t *ref);

/* Contador de referencias recibidas (cambia con cada consigna nueva). */
uint32_t control_reference_seq(void);

/* Un paso de la tarea de control. */
void control_update(void);

/* Posicion medida de cada articulacion [rad], en las mismas unidades que
 * la referencia (sirve para publicar y para la caracterizacion). */
void control_get_position(joints_t *pos);

#endif /* CONTROL_H */
