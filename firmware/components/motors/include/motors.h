/*
 * motors.h - Manejo de los 3 motores DC (TB6612FNG) y sus finales de carrera.
 *
 * Basado en Scara_Main.c (homing original). Mismos pines y parametros.
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

typedef struct {
    int pwm_pin;
    int dir1_pin;
    int dir2_pin;
    int ls_a_pin;              /* final de carrera A */
    int ls_b_pin;              /* final de carrera B */
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

bool limit_a_pressed(int i);
bool limit_b_pressed(int i);

#endif /* MOTORS_H */
