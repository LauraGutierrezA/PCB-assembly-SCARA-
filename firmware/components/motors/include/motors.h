/*
 * motors.h - Manejo de los 3 motores DC (TB6612FNG) y sus finales de carrera.
 *
 * Basado en Scara_Main.c (homing original). Pines en pins.h.
 *
 * Logica TB6612 (IN1, IN2):
 *   (1, 0) -> un sentido        (0, 1) -> sentido contrario
 *   (1, 1) -> freno (short brake)   (0, 0) -> libre (coast)
 */
#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/ledc.h"

#define NUM_MOTORS 3

/* Valor para un final de carrera que NO existe (p. ej. el motor 2 solo
 * tiene final L). Un pin NO_PIN no se configura y siempre lee "suelto". */
#define NO_PIN (-1)

typedef struct {
    int pwm_pin;
    int dir1_pin;
    int dir2_pin;
    int ls_l_pin;              /* final al que se llega con (IN1,IN2) = (1,0) */
    int ls_r_pin;              /* final al que se llega con (0,1); NO_PIN si no hay */
    ledc_channel_t pwm_channel;
    uint32_t backoff_time_ms;  /* tiempo de retroceso despues de B (homing por tiempo) */
    uint32_t homing_speed;     /* duty PWM 0..1023 (10 bits) */
} motor_config_t;

extern motor_config_t motors[NUM_MOTORS];

void motors_init(void);

void motor_set_dir(int i, int d1, int d2);
void motor_set_duty(int i, uint32_t duty);
void motor_brake(int i);        /* (1,1) */
void motor_coast(int i);        /* (0,0) */
void motors_stop_all(void);     /* todos en (0,0) */

bool limit_l_pressed(int i);
bool limit_r_pressed(int i);

/* true si el motor i tiene segundo final de carrera (ls_r_pin != NO_PIN). */
bool motor_has_limit_r(int i);

#endif /* MOTORS_H */
