/*
 * motors.c - Ver motors.h
 *
 * Cambios frente a Scara_Main.c:
 *   - Se elimino uart_driver_install(UART_NUM_0): el UART0 ahora lo usa micro-ROS.
 *   - Se elimino todo printf: la consola esta desactivada (menuconfig -> None).
 */
#include "motors.h"
#include "driver/gpio.h"

/* ---------------- Pines (iguales a Scara_Main.c) ---------------- */
/* Motor 1 */
#define M1_PWM_PIN    27
#define M1_DIR_1      2     /* ojo: pin de arranque (strapping) */
#define M1_DIR_2      15    /* ojo: pin de arranque (strapping) */
#define M1_LS_A       5
#define M1_LS_B       4
/* Motor 2 */
#define M2_PWM_PIN    26
#define M2_DIR_1      12    /* ojo: pin de arranque critico, ver notas */
#define M2_DIR_2      14
#define M2_LS_A       16
#define M2_LS_B       17
/* Motor 3 */
#define M3_PWM_PIN    21
#define M3_DIR_1      22
#define M3_DIR_2      23
#define M3_LS_A       18
#define M3_LS_B       19

#define PWM_FREQ_HZ       5000
#define PWM_RESOLUTION    LEDC_TIMER_10_BIT

/* { pwm, dir1, dir2, lsA, lsB, canal, backoff_ms, homing_speed } */
motor_config_t motors[NUM_MOTORS] = {
    {M1_PWM_PIN, M1_DIR_1, M1_DIR_2, M1_LS_A, M1_LS_B, LEDC_CHANNEL_0, 2000, 200},
    {M2_PWM_PIN, M2_DIR_1, M2_DIR_2, M2_LS_A, M2_LS_B, LEDC_CHANNEL_1,  950, 512},
    {M3_PWM_PIN, M3_DIR_1, M3_DIR_2, M3_LS_A, M3_LS_B, LEDC_CHANNEL_2, 3000, 430},
};

void motors_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    uint64_t dir_pin_mask = 0;
    uint64_t ls_pin_mask  = 0;

    for (int i = 0; i < NUM_MOTORS; i++) {
        ledc_channel_config_t ledc_channel = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = motors[i].pwm_channel,
            .timer_sel  = LEDC_TIMER_0,
            .intr_type  = LEDC_INTR_DISABLE,
            .gpio_num   = motors[i].pwm_pin,
            .duty       = motors[i].homing_speed,
            .hpoint     = 0,
        };
        ledc_channel_config(&ledc_channel);

        dir_pin_mask |= (1ULL << motors[i].dir1_pin) | (1ULL << motors[i].dir2_pin);
        ls_pin_mask  |= (1ULL << motors[i].ls_a_pin) | (1ULL << motors[i].ls_b_pin);
    }

    gpio_config_t dir_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = dir_pin_mask,
        .pull_down_en = 0,
        .pull_up_en   = 0,
    };
    gpio_config(&dir_conf);

    gpio_config_t ls_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_INPUT,
        .pin_bit_mask = ls_pin_mask,
        .pull_down_en = 1,
        .pull_up_en   = 0,
    };
    gpio_config(&ls_conf);

    motors_stop_all();   /* arrancar siempre con los motores quietos */
}

void motor_set_dir(int i, int d1, int d2)
{
    gpio_set_level(motors[i].dir1_pin, d1);
    gpio_set_level(motors[i].dir2_pin, d2);
}

void motor_set_duty(int i, uint32_t duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, motors[i].pwm_channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motors[i].pwm_channel);
}

void motor_brake(int i) { motor_set_dir(i, 1, 1); }
void motor_coast(int i) { motor_set_dir(i, 0, 0); }

void motors_stop_all(void)
{
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_coast(i);
    }
}

bool limit_a_pressed(int i) { return gpio_get_level(motors[i].ls_a_pin) == 1; }
bool limit_b_pressed(int i) { return gpio_get_level(motors[i].ls_b_pin) == 1; }
