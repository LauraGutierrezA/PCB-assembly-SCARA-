/*
 * effector.c - Ver effector.h
 *
 * Servo: PWM de 50 Hz (periodo 20 ms). El ancho del pulso fija el angulo:
 *   SERVO_US_MIN  -> -90 grados      SERVO_US_MAX -> +90 grados
 * Los motores usan LEDC_TIMER_0 a 5 kHz (canales 0, 1, 2); el servo usa
 * otro timer (LEDC_TIMER_1) y el canal 3 para no cambiarles la frecuencia.
 *
 * Calibracion del servo (hacer una vez, con el servo sin la boquilla):
 *   - Si al mandar "rot +90" gira al reves de lo esperado: SERVO_INVERT 1.
 *   - Si no llega a los 90 grados o golpea al final: ajustar SERVO_US_MIN /
 *     SERVO_US_MAX (Tower Pro SG90 tipico: 500..2400 us).
 */
#include "effector.h"

#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "pins.h"

/* ---------------- Servo ---------------- */
#define SERVO_FREQ_HZ     50
#define SERVO_RES         LEDC_TIMER_14_BIT      /* 16384 pasos en 20 ms */
#define SERVO_TIMER       LEDC_TIMER_1
#define SERVO_CHANNEL     LEDC_CHANNEL_3
#define SERVO_US_MIN      500                    /* pulso para -90 grados */
#define SERVO_US_MAX      2400                   /* pulso para +90 grados */
#define SERVO_INVERT      0                      /* 1: invierte el sentido */

/* ---------------- Bomba ---------------- */
#define VACUUM_ACTIVE_LEVEL  1                   /* 1: el MOSFET conduce con ALTO */

static volatile int  servo_deg = 0;
static volatile bool vacuum = false;

static uint32_t servo_duty_for(int deg)
{
    if (SERVO_INVERT) deg = -deg;
    const float frac = (float)(deg - SERVO_MIN_DEG) / (float)(SERVO_MAX_DEG - SERVO_MIN_DEG);
    const float us = SERVO_US_MIN + frac * (SERVO_US_MAX - SERVO_US_MIN);
    const float period_us = 1000000.0f / SERVO_FREQ_HZ;
    return (uint32_t)(us / period_us * (float)((1 << 14) - 1) + 0.5f);
}

static void servo_write(int deg)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL, servo_duty_for(deg));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL);
}

void effector_init(void)
{
    /* Bomba: salida en reposo (apagada). GPIO 12 es pin de arranque: el
     * MOSFET debe tener pull-down en la compuerta para que quede en BAJO
     * mientras el ESP32 arranca. */
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << PIN_VACUUM,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    effector_set_vacuum(false);

    /* Servo */
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = SERVO_TIMER,
        .duty_resolution = SERVO_RES,
        .freq_hz         = SERVO_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);
    ledc_channel_config_t ch = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = SERVO_CHANNEL,
        .timer_sel  = SERVO_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = PIN_SERVO,
        .duty       = servo_duty_for(0),
        .hpoint     = 0,
    };
    ledc_channel_config(&ch);
    servo_deg = 0;
}

void effector_set_vacuum(bool on)
{
    vacuum = on;
    gpio_set_level(PIN_VACUUM, on ? VACUUM_ACTIVE_LEVEL : !VACUUM_ACTIVE_LEVEL);
}

bool effector_vacuum_on(void) { return vacuum; }

int effector_rotate_rel(int delta_deg)
{
    int d = servo_deg + delta_deg;
    if (d < SERVO_MIN_DEG) d = SERVO_MIN_DEG;
    if (d > SERVO_MAX_DEG) d = SERVO_MAX_DEG;
    servo_deg = d;
    servo_write(d);
    return d;
}

int effector_servo_deg(void) { return servo_deg; }

bool effector_handle_cmd(const char *data, size_t n)
{
    char line[24];
    if (n >= sizeof(line)) return false;
    memcpy(line, data, n);
    line[n] = '\0';

    if (strcmp(line, "vac1") == 0) { effector_set_vacuum(true);  return true; }
    if (strcmp(line, "vac0") == 0) { effector_set_vacuum(false); return true; }
    if (strcmp(line, "stop") == 0) { effector_set_vacuum(false); return false; }

    int delta;
    if (sscanf(line, "rot %d", &delta) == 1) {
        if (delta == 90 || delta == -90) effector_rotate_rel(delta);
        return true;
    }
    return false;
}
