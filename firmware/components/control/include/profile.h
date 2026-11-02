/*
 * profile.h - Generador de trayectoria EN EL ESP32 (pestana Trayectoria).
 *
 * El PC ya no manda 20 consignas por segundo: manda UNA meta articular y la
 * duracion del tramo (/scara/target con 13 valores, ver main.c). Aqui se
 * interpola la referencia en cada paso del control (8 ms), asi el lazo ve
 * una referencia suave, sin escalones ni el jitter de WiFi / ROS / UART.
 *
 * Cada tramo es un movimiento REPOSO-REPOSO de todas las articulaciones con
 * la misma duracion T (la calcula el PC con los limites de cada eje):
 *   th1, th2 -> QUINTICO (velocidad y aceleracion continuas).
 *   z  (M3)  -> TRAPEZOIDAL de velocidad: acelera en PROF_TRAP_FRAC*T,
 *               crucero, frena en PROF_TRAP_FRAC*T. Pico = 1.25 * promedio
 *               (el quintico es 1.875 * promedio): el eje Z, que es el
 *               lento, aprovecha mejor su velocidad maxima.
 *
 * Mezcla de tramos (SUPERPOSICION): si llega una meta nueva mientras otra
 * esta en curso, el tramo nuevo se SUMA al que va terminando (no se frena en
 * el punto intermedio). Cada tramo sigue siendo reposo-reposo, asi que no
 * hay sobrepaso y la meta final es exacta. El PC manda el siguiente tramo
 * cuando PG (avance) pasa de cierto % (BLEND_PCT en trajectory_tab.py).

 * El reloj del perfil solo avanza mientras el control esta activo (homing
 * hecho + dead-man presionado): al soltar el dead-man el perfil se congela
 * y al volver sigue donde iba.
 *
 * Unidades: las del control (th1, th2 en rad; z en mm desde el piso).
 */
#ifndef PROFILE_H
#define PROFILE_H

#include <stdbool.h>
#include <stdint.h>
#include "ik.h"

/* Fraccion de T para acelerar (y para frenar) en el trapezoide del eje Z. */
#define PROF_TRAP_FRAC   0.2f
/* Duracion minima de un tramo [s]. */
#define PROF_MIN_T       0.05f

/* Nucleo 0 (micro-ROS): pide un tramo nuevo hacia goal en T segundos.
 * Si id es el mismo del ultimo tramo o del pendiente, se ignora (reenvio del
 * PC) y devuelve false. id = 0 no se usa (el PC empieza en 1). */
bool prof_request(const joints_t *goal, float T, uint32_t id);

/* Cualquier nucleo: cancela el perfil (consigna directa, Stop, release...). */
void prof_cancel(void);

/* Nucleo 1, en cada paso ACTIVO del control (dt en s).
 *   start: posicion medida, para arrancar si no hay un perfil previo.
 *   ref:   se sobrescribe con la referencia del instante si hay perfil.
 * Devuelve true mientras el perfil esta en curso (la referencia se mueve). */
bool prof_step(float dt, const joints_t *start, joints_t *ref);

bool     prof_running(void);   /* hay un tramo en curso (o pendiente) */
int      prof_progress(void);  /* 0..100 del tramo actual (100: terminado / sin perfil) */
uint32_t prof_goal_id(void);   /* id del ultimo tramo que ARRANCO */

#endif /* PROFILE_H */
