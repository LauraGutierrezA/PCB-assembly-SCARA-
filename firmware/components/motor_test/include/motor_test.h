/*
 * motor_test.h - Accionamiento manual de motores para probar el circuito.
 *
 * Cada motor se puede mover por un tiempo fijo, en un sentido y a una
 * velocidad dados. NO se detiene al tocar un end-stop: la idea es probar
 * cada parte aislada (motores y end-stops por separado).
 *
 * Sentido (igual que el homing):
 *   MT_DIR_L -> (IN1, IN2) = (1, 0)   hacia el end-stop L
 *   MT_DIR_R -> (IN1, IN2) = (0, 1)   hacia el end-stop R
 *
 * Mientras el homing esta corriendo, las ordenes de marcha se ignoran y el
 * homing tiene prioridad sobre los motores.
 *
 * Uso entre tareas (seguro):
 *   - motor_test_request_*() solo dejan la orden pendiente (nucleo 0).
 *   - motor_test_update() la ejecuta la tarea de control (nucleo 1).
 */
#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H

#include <stdbool.h>
#include <stdint.h>

typedef enum { MT_DIR_L = 0, MT_DIR_R = 1 } mt_dir_t;

/* motor: 0..2. duty: 0..1023 (PWM de 10 bits). duration_ms > 0. */
void motor_test_request_run(int motor, mt_dir_t dir, uint32_t duty, uint32_t duration_ms);

/* motor: 0..2, o -1 para todos. */
void motor_test_request_stop(int motor);

/* Un paso: atiende ordenes y apaga los motores cuyo tiempo se cumplio. */
void motor_test_update(void);

bool motor_test_is_running(int motor);

#endif /* MOTOR_TEST_H */
