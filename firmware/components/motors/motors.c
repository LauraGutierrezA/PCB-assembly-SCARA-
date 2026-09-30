/*
 * motors.c - Ver motors.h. Los pines estan en pins.h.
 *
 * Cambios frente a Scara_Main.c:
 *   - Se elimino uart_driver_install(UART_NUM_0): el UART0 ahora lo usa micro-ROS.
 *   - Se elimino todo printf: la consola esta desactivada (menuconfig -> None).
 *   - Nuevo mapa de pines (pins.h). STBY de los drivers va soldado a alto.
 */
#include "motors.h"
#include "pins.h"
#include "driver/gpio.h"

#define PWM_FREQ_HZ       5000
#define PWM_RESOLUTION    LEDC_TIMER_10_BIT

/* { pwm, in1, in2, final L, final R, canal, backoff_ms, homing_speed } */
motor_config_t motors[NUM_MOTORS] = {
    {PIN_M1_PWM, PIN_M1_IN1, PIN_M1_IN2, PIN_LS_L1, PIN_LS_R1, LEDC_CHANNEL_0, 2000, 200},
    {PIN_M2_PWM, PIN_M2_IN1, PIN_M2_IN2, PIN_LS_L2, PIN_LS_R2, LEDC_CHANNEL_1,  950, 512},
    {PIN_M3_PWM, PIN_M3_IN1, PIN_M3_IN2, PIN_LS_L3, PIN_LS_R3, LEDC_CHANNEL_2, 3000, 430},
};

/* Configura un final de carrera. Los GPIO 34-39 no tienen pull interno
 * (llevan resistencia externa); a los demas se les activa el pull-down. */
static void config_limit_input(int pin)
{
    gpio_config_t conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_INPUT,
        .pin_bit_mask = 1ULL << pin,
        .pull_down_en = GPIO_IS_VALID_OUTPUT_GPIO(pin) ? 1 : 0,
        .pull_up_en   = 0,
    };
    gpio_config(&conf);
}

void motors_init(void)
{
    /* 1. Salidas de direccion, en bajo (motores quietos en coast) */
    uint64_t out_mask = 0;
    for (int i = 0; i < NUM_MOTORS; i++) {
        out_mask |= (1ULL << motors[i].dir1_pin) | (1ULL << motors[i].dir2_pin);
    }
    gpio_config_t out_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = out_mask,
        .pull_down_en = 0,
        .pull_up_en   = 0,
    };
    gpio_config(&out_conf);
    motors_stop_all();

    /* 2. PWM */
    ledc_timer_config_t ledc_timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

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
    }

    /* 3. Finales de carrera */
    for (int i = 0; i < NUM_MOTORS; i++) {
        config_limit_input(motors[i].ls_l_pin);
        config_limit_input(motors[i].ls_r_pin);
    }
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

bool limit_l_pressed(int i) { return gpio_get_level(motors[i].ls_l_pin) == 1; }
bool limit_r_pressed(int i) { return gpio_get_level(motors[i].ls_r_pin) == 1; }
