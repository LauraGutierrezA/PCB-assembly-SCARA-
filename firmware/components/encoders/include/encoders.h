/*
 * encoders.h - Lectura de los 3 encoders de cuadratura con el PCNT del ESP32.
 *
 * El PCNT (contador de pulsos) cuenta en hardware: no gasta CPU ni pierde
 * pulsos aunque la tarea de control este ocupada. Se cuentan los 4 flancos
 * de A y B (x4), asi que por cada vuelta del EJE DEL MOTOR (antes de la
 * caja reductora) hay 4 * ENC_CPR cuentas.
 *
 * Datos del motor (NPI_Proyecto.mlx):
 *   CPR = 16 cuentas por vuelta del motor (un solo flanco)  -> 64 en x4
 *   N   = 50 (relacion de la caja reductora)
 *
 * Pines en pins.h (PIN_ENCx_A / PIN_ENCx_B).
 */
#ifndef ENCODERS_H
#define ENCODERS_H

#include <stdint.h>

#define ENC_CPR             16                     /* un flanco, eje del motor */
#define ENC_COUNTS_PER_REV  (4 * ENC_CPR)          /* x4: 64 cuentas por vuelta del motor */

void encoders_init(void);

/* Pone en cero las 3 cuentas (se llama al terminar el homing). */
void encoders_reset(void);

/* Cuentas acumuladas del encoder i (0..2). */
int32_t encoder_get_count(int i);

/* Angulo del EJE DEL MOTOR i en radianes (antes de la reductora). */
float encoder_get_motor_rad(int i);

#endif /* ENCODERS_H */
