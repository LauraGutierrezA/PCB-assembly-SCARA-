/*
 * homing.c - Ver homing.h
 *
 * Diferencias frente a Scara_Main.c:
 *   - Los vTaskDelay(100) despues de tocar un final se volvieron fases de
 *     pausa (PAUSE_L / PAUSE_R). Asi homing_update() nunca bloquea la tarea.
 *   - 'start' reinicia desde el motor 1; 'resume' continua donde quedo.
 *   - Los comandos llegan por ROS 2 (/scara/cmd) en vez de 's'/'x' por serial.
 */
#include "homing.h"
#include "motors.h"

#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BRAKE_PAUSE_MS 100

typedef enum {
    PH_SEEK_L = 0,
    PH_PAUSE_L,
    PH_SEEK_R,
    PH_PAUSE_R,
    PH_BACKOFF,
} phase_t;

static const char *PHASE_NAMES[] = { "SEEK_L", "PAUSE_L", "SEEK_R", "PAUSE_R", "BACKOFF" };

/* Banderas escritas desde otra tarea */
static volatile bool start_req  = false;
static volatile bool resume_req = false;
static volatile bool abort_req  = false;

/* Estado (solo lo escribe homing_update) */
static volatile homing_state_t state = HOMING_IDLE;
static volatile int      cur_motor = 0;
static volatile phase_t  phase = PH_SEEK_L;
static volatile uint32_t seq = 0;
static TickType_t phase_t0 = 0;
static TickType_t paused_elapsed = 0;   /* tiempo ya transcurrido en la fase al abortar */

static void set_phase(phase_t p)
{
    phase = p;
    phase_t0 = xTaskGetTickCount();
    seq++;
}

static bool elapsed_ms(uint32_t ms)
{
    return (xTaskGetTickCount() - phase_t0) >= pdMS_TO_TICKS(ms);
}

static void apply_homing_speeds(void)
{
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_set_duty(i, motors[i].homing_speed);
    }
}

void homing_start(void)  { start_req = true; }
void homing_resume(void) { resume_req = true; }
void homing_abort(void)  { abort_req = true; }

homing_state_t homing_get_state(void) { return state; }
uint32_t homing_status_seq(void) { return seq; }

/* Atiende las banderas. Devuelve false si hay que salir de update. */
static bool handle_requests(void)
{
    /* Abortar tiene prioridad sobre todo */
    if (abort_req) {
        abort_req = false;
        start_req = false;
        resume_req = false;
        motors_stop_all();
        if (state == HOMING_RUNNING) {
            paused_elapsed = xTaskGetTickCount() - phase_t0;
            state = HOMING_ABORTED;
            seq++;
        }
        return false;
    }

    if (start_req) {
        start_req = false;
        resume_req = false;
        motors_stop_all();
        apply_homing_speeds();
        cur_motor = 0;
        state = HOMING_RUNNING;
        set_phase(PH_SEEK_L);
        return true;
    }

    if (resume_req) {
        resume_req = false;
        if (state == HOMING_ABORTED) {
            apply_homing_speeds();
            state = HOMING_RUNNING;
            /* Misma fase; el reloj de la fase continua donde iba, asi el
             * retroceso solo completa el tiempo que le faltaba. */
            phase_t0 = xTaskGetTickCount() - paused_elapsed;
            seq++;
        } else if (state == HOMING_IDLE) {
            start_req = true;          /* nunca se ha corrido: igual que start */
            return handle_requests();
        }
        /* DONE o RUNNING: no hace nada */
    }
    return true;
}

void homing_update(void)
{
    if (!handle_requests() || state != HOMING_RUNNING) {
        return;
    }

    const int m = cur_motor;

    switch (phase) {
    case PH_SEEK_L:
        motor_set_dir(m, 1, 0);
        if (limit_a_pressed(m)) {
            motor_brake(m);
            set_phase(PH_PAUSE_L);
        }
        break;

    case PH_PAUSE_L:
        motor_brake(m);
        if (elapsed_ms(BRAKE_PAUSE_MS)) {
            set_phase(PH_SEEK_R);
        }
        break;

    case PH_SEEK_R:
        motor_set_dir(m, 0, 1);
        if (limit_b_pressed(m)) {
            motor_brake(m);
            set_phase(PH_PAUSE_R);
        }
        break;

    case PH_PAUSE_R:
        motor_brake(m);
        if (elapsed_ms(BRAKE_PAUSE_MS)) {
            set_phase(PH_BACKOFF);
        }
        break;

    case PH_BACKOFF:
        motor_set_dir(m, 1, 0);
        if (elapsed_ms(motors[m].backoff_time_ms)) {
            motor_coast(m);
            if (m + 1 < NUM_MOTORS) {
                cur_motor = m + 1;
                set_phase(PH_SEEK_L);
            } else {
                state = HOMING_DONE;
                seq++;
            }
        }
        break;
    }
}

void homing_status_str(char *buf, size_t len)
{
    switch (state) {
    case HOMING_IDLE:
        snprintf(buf, len, "IDLE");
        break;
    case HOMING_DONE:
        snprintf(buf, len, "HOMED");
        break;
    case HOMING_ABORTED:
        snprintf(buf, len, "ABORTED M%d %s", cur_motor + 1, PHASE_NAMES[phase]);
        break;
    case HOMING_RUNNING:
        snprintf(buf, len, "HOMING M%d %s", cur_motor + 1, PHASE_NAMES[phase]);
        break;
    }
}
