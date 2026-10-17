/*
 * motor_test.c - Ver motor_test.h
 */
#include "motor_test.h"
#include "motors.h"
#include "homing_pro.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    bool     run;
    bool     stop;
    mt_dir_t dir;
    uint32_t duty;
    uint32_t duration_ms;
} request_t;

static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static request_t pending[NUM_MOTORS];

/* Estado: solo lo escribe motor_test_update() */
static volatile bool running[NUM_MOTORS];
static TickType_t    deadline[NUM_MOTORS];

void motor_test_request_run(int motor, mt_dir_t dir, uint32_t duty, uint32_t duration_ms)
{
    if (motor < 0 || motor >= NUM_MOTORS || duration_ms == 0) return;
    if (duty > 1023) duty = 1023;
    portENTER_CRITICAL(&lock);
    pending[motor].run = true;
    pending[motor].stop = false;
    pending[motor].dir = dir;
    pending[motor].duty = duty;
    pending[motor].duration_ms = duration_ms;
    portEXIT_CRITICAL(&lock);
}

void motor_test_request_stop(int motor)
{
    portENTER_CRITICAL(&lock);
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (motor < 0 || motor == i) {
            pending[i].stop = true;
            pending[i].run = false;
        }
    }
    portEXIT_CRITICAL(&lock);
}

bool motor_test_is_running(int motor)
{
    return motor >= 0 && motor < NUM_MOTORS && running[motor];
}

void motor_test_update(void)
{
    const bool homing_busy = (homing_pro_get_state() == HP_RUNNING);
    const TickType_t now = xTaskGetTickCount();

    for (int i = 0; i < NUM_MOTORS; i++) {
        request_t r;
        portENTER_CRITICAL(&lock);
        r = pending[i];
        pending[i].run = false;
        pending[i].stop = false;
        portEXIT_CRITICAL(&lock);

        if (homing_busy) {
            /* El homing manda sobre los motores: se descarta la prueba */
            running[i] = false;
            continue;
        }

        if (r.stop) {
            motor_coast(i);
            running[i] = false;
        }

        if (r.run) {
            motor_set_duty(i, r.duty);
            if (r.dir == MT_DIR_L) {
                motor_set_dir(i, 1, 0);
            } else {
                motor_set_dir(i, 0, 1);
            }
            deadline[i] = now + pdMS_TO_TICKS(r.duration_ms);
            running[i] = true;
        }

        if (running[i] && (int32_t)(now - deadline[i]) >= 0) {
            motor_coast(i);
            running[i] = false;
        }
    }
}
