/*
 * encoders.c - Ver encoders.h
 *
 * Configuracion x4 tomada del ejemplo oficial de ESP-IDF
 * (examples/peripherals/pcnt/rotary_encoder).
 */
#include "encoders.h"
#include "pins.h"
#include <stdbool.h>
#include "esp_err.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"

#define NUM_ENC         3
#define PCNT_LIMIT      30000     /* el contador del hardware es de 16 bits; */
                                  /* accum_count extiende el conteo a 32 bits */
#define GLITCH_NS       1000      /* ignora pulsos de menos de 1 us (ruido) */

/* +1 o -1 por encoder. Si al mover un motor en el sentido (IN1,IN2)=(1,0)
 * las cuentas BAJAN, cambiar ese encoder a -1. Asi u > 0 siempre hace
 * crecer el angulo medido (si no, el lazo se vuelve inestable). */
static const int ENC_SIGN[NUM_ENC] = { +1, +1, +1 };

static const int PIN_A[NUM_ENC] = { PIN_ENC1_A, PIN_ENC2_A, PIN_ENC3_A };
static const int PIN_B[NUM_ENC] = { PIN_ENC1_B, PIN_ENC2_B, PIN_ENC3_B };

static pcnt_unit_handle_t unit[NUM_ENC];

static void init_one(int i)
{
    pcnt_unit_config_t unit_cfg = {
        .high_limit = PCNT_LIMIT,
        .low_limit  = -PCNT_LIMIT,
        .flags.accum_count = true,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &unit[i]));

    pcnt_glitch_filter_config_t filter = { .max_glitch_ns = GLITCH_NS };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit[i], &filter));

    /* Canal A: cuenta flancos de A, mirando el nivel de B */
    pcnt_chan_config_t cha_cfg = { .edge_gpio_num = PIN_A[i], .level_gpio_num = PIN_B[i] };
    pcnt_channel_handle_t cha;
    ESP_ERROR_CHECK(pcnt_new_channel(unit[i], &cha_cfg, &cha));

    /* Canal B: cuenta flancos de B, mirando el nivel de A */
    pcnt_chan_config_t chb_cfg = { .edge_gpio_num = PIN_B[i], .level_gpio_num = PIN_A[i] };
    pcnt_channel_handle_t chb;
    ESP_ERROR_CHECK(pcnt_new_channel(unit[i], &chb_cfg, &chb));

    /* El driver activa el pull-up interno en A y B al crear CADA canal.
     * Se apaga DESPUES de crear los dos canales, para que los 3 encoders
     * queden iguales (en 34-39 no hay pull-up; en 32/33 si lo habia). */
    gpio_pullup_dis(PIN_A[i]);
    gpio_pullup_dis(PIN_B[i]);

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(cha,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(cha,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chb,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chb,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    /* Puntos de vigilancia en los limites: necesarios para accum_count */
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit[i], PCNT_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit[i], -PCNT_LIMIT));

    ESP_ERROR_CHECK(pcnt_unit_enable(unit[i]));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit[i]));
    ESP_ERROR_CHECK(pcnt_unit_start(unit[i]));
}

void encoders_init(void)
{
    for (int i = 0; i < NUM_ENC; i++) {
        init_one(i);
    }
}

void encoders_reset(void)
{
    for (int i = 0; i < NUM_ENC; i++) {
        pcnt_unit_clear_count(unit[i]);
    }
}

int32_t encoder_get_count(int i)
{
    int c = 0;
    pcnt_unit_get_count(unit[i], &c);
    return ENC_SIGN[i] * c;
}

float encoder_get_motor_rad(int i)
{
    return (float)encoder_get_count(i) * (6.28318531f / (float)ENC_COUNTS_PER_REV);
}
