/*
 * observers.c - Ver observers.h.
 *
 * Basado en FINALlab4_observers (estimators.cpp + config.h), simplificado:
 * L y K se calculan al arrancar y en tiempo real solo se multiplican.
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "observers.h"

#ifndef OBS_HOST_TEST
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "encoders.h"
#include "motors.h"
#include "homing_pro.h"
#include "control.h"          /* CONTROL_LOOP_MS */
#endif

/* ================================================================
 *   PARAMETROS DE CADA MOTOR   <-- AQUI SE CAMBIAN
 * ================================================================
 * Una fila por motor: { A (2x2), B (2), fac, Q11, Q22, R }
 *   A, B : modelo discreto (c2d, zoh, Ts = 8 ms) de CLabIV_State_Observers.mlx
 *          estados [i_a (A); omega (rad/s del motor)], entrada u [V]
 *   fac  : polos del Luenberger 'fac' veces mas rapidos que la planta (2..4)
 *   Q11  : [A^2]   Q22, R : [(rad/s)^2]
 * Hoy los 3 motores usan los valores del motor 2 del Lab 4 (los mismos
 * DEF_* de config.h). Si identifican cada motor, cambian su fila y listo:
 * L y K se recalculan solos al arrancar. Si se cambia Ts, A y B tambien.
 */
typedef struct {
    double A[2][2];
    double B[2];
    double fac;
    double q11, q22, r;
} obs_params_t;

#define OBS_LAB4_MOTOR2 {                                            \
    { { -0.00656375526807087, -0.0048629579529921 },                 \
      {  1.19105706241618,     0.882426646325206  } },               \
    { 0.469957803662264, 10.9368749012802 },                         \
    3.0,                    /* fac */                                \
    0.0001, 14.5854,        /* Q11 [A^2], Q22 [(rad/s)^2] */          \
    132.603                 /* R [(rad/s)^2] */                       \
}

static const obs_params_t OBS_PARAMS[OBS_NUM_MOTORS] = {
    OBS_LAB4_MOTOR2,   /* motor 1 (th1) */
    OBS_LAB4_MOTOR2,   /* motor 2 (th2) */
    OBS_LAB4_MOTOR2,   /* motor 3 (eje Z, tornillo) */
};

/* Ts del modelo: el periodo de la tarea de control (8 ms) */
#define OBS_TS_S   0.008f

#ifndef OBS_HOST_TEST
#if CONTROL_LOOP_MS != 8
#error "El modelo de los observadores es a Ts = 8 ms: recalcular A y B en MATLAB"
#endif
#endif

/* rad del motor por cuenta (x4, 64 cuentas por vuelta del motor) */
#define OBS_RAD_PER_COUNT  (6.28318531f / 64.0f)

/* Un salto mayor a esto en 8 ms es imposible (200 RPM salida * 50 = ~85
 * cuentas): es el reset de los encoders al terminar el homing. Se reinician
 * los observadores en vez de mostrar el pico. */
#define OBS_MAX_DCOUNTS    400

/* ================================================================
 *   Diseno (double, una vez)
 * ================================================================ */
typedef struct {
    float a11, a12, a21, a22, b1, b2;
    float l1, l2;          /* Luenberger */
    float k1, k2;          /* Kalman, estado estacionario */
} obs_gains_t;

/* Valores propios de una matriz 2x2 (igual que estimators.cpp) */
static void eig2(const double A[2][2], double re[2], double im[2])
{
    const double tr = A[0][0] + A[1][1];
    const double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
    const double h = 0.5 * tr;
    const double disc = h * h - det;
    if (disc >= 0.0) {
        const double s = sqrt(disc);
        const double big = (h >= 0.0) ? h + s : h - s;
        re[0] = big;
        re[1] = (big != 0.0) ? det / big : h - s;
        im[0] = im[1] = 0.0;
    } else {
        const double s = sqrt(-disc);
        re[0] = re[1] = h;
        im[0] = s;
        im[1] = -s;
    }
}

/* Luenberger por Ackermann, C = [0 1]. false si no se puede (L queda en 0) */
static bool design_luenberger(const obs_params_t *p, double L[2])
{
    const double (*A)[2] = p->A;
    double re[2], im[2], a1, a2;
    L[0] = L[1] = 0.0;
    if (fabs(A[1][0]) < 1e-12 || !(p->fac > 0.0)) return false;
    eig2(A, re, im);
    if (im[0] == 0.0) {
        if (re[0] <= 0.0 || re[1] <= 0.0) return false;
        const double q1 = pow(re[0], p->fac), q2 = pow(re[1], p->fac);
        a1 = -(q1 + q2);
        a2 = q1 * q2;
    } else {
        const double r = hypot(re[0], im[0]), th = atan2(im[0], re[0]);
        const double rq = pow(r, p->fac), tq = p->fac * th;
        a1 = -2.0 * rq * cos(tq);
        a2 = rq * rq;
    }
    /* L = Phi(A)(:,1) / a21,   Phi(A) = A^2 + a1 A + a2 I */
    const double phi11 = A[0][0] * A[0][0] + A[0][1] * A[1][0] + a1 * A[0][0] + a2;
    const double phi21 = A[1][0] * A[0][0] + A[1][1] * A[1][0] + a1 * A[1][0];
    L[0] = phi11 / A[1][0];
    L[1] = phi21 / A[1][0];

    /* Verificar: eig(A - L C) dentro del circulo unitario */
    const double ALC[2][2] = { { A[0][0], A[0][1] - L[0] }, { A[1][0], A[1][1] - L[1] } };
    eig2(ALC, re, im);
    for (int i = 0; i < 2; i++) {
        if (hypot(re[i], im[i]) >= 1.0) { L[0] = L[1] = 0.0; return false; }
    }
    return true;
}

/* Kalman de estado estacionario: se itera la ecuacion de Riccati (la misma
 * prediccion/correccion del Lab 4) hasta que K deja de cambiar. */
static void design_kalman(const obs_params_t *p, double K[2])
{
    const double (*A)[2] = p->A;
    double P[2][2] = { { 0.25, 0.0 }, { 0.0, 250000.0 } };   /* P0 del Lab 4 */
    double Pm[2][2], AP[2][2];
    K[0] = K[1] = 0.0;
    for (int it = 0; it < 5000; it++) {
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                AP[i][j] = A[i][0] * P[0][j] + A[i][1] * P[1][j];
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                Pm[i][j] = AP[i][0] * A[j][0] + AP[i][1] * A[j][1];
        Pm[0][0] += p->q11;
        Pm[1][1] += p->q22;
        const double S = Pm[1][1] + p->r;
        const double k0 = Pm[0][1] / S, k1 = Pm[1][1] / S;
        for (int j = 0; j < 2; j++) {
            P[0][j] = Pm[0][j] - k0 * Pm[1][j];
            P[1][j] = Pm[1][j] - k1 * Pm[1][j];
        }
        const double s = 0.5 * (P[0][1] + P[1][0]);
        P[0][1] = P[1][0] = s;
        const bool done = fabs(k0 - K[0]) < 1e-12 && fabs(k1 - K[1]) < 1e-12;
        K[0] = k0;
        K[1] = k1;
        if (done && it > 10) break;
    }
}

static void design_gains(const obs_params_t *p, obs_gains_t *g)
{
    double L[2], K[2];
    design_luenberger(p, L);
    design_kalman(p, K);
    g->a11 = (float)p->A[0][0];  g->a12 = (float)p->A[0][1];
    g->a21 = (float)p->A[1][0];  g->a22 = (float)p->A[1][1];
    g->b1 = (float)p->B[0];      g->b2 = (float)p->B[1];
    g->l1 = (float)L[0];         g->l2 = (float)L[1];
    g->k1 = (float)K[0];         g->k2 = (float)K[1];
}

/* ================================================================
 *   Un paso (float)
 * ================================================================ */
typedef struct {
    float xl[2];       /* Luenberger [A, rad/s] */
    float xk[2];       /* Kalman     [A, rad/s] */
    float y_prev;      /* omega medida en k-1 (el Luenberger predictor la usa) */
} obs_state_t;

/* y = omega medida en k, u_prev = voltaje aplicado en [k-1, k) */
static void obs_step(const obs_gains_t *g, obs_state_t *s, float y, float u_prev)
{
    /* Luenberger:  xl(k) = (A - L C) xl(k-1) + L y(k-1) + B u(k-1) */
    const float l0 = g->a11 * s->xl[0] + (g->a12 - g->l1) * s->xl[1] + g->l1 * s->y_prev + g->b1 * u_prev;
    const float l1 = g->a21 * s->xl[0] + (g->a22 - g->l2) * s->xl[1] + g->l2 * s->y_prev + g->b2 * u_prev;
    s->xl[0] = l0;
    s->xl[1] = l1;

    /* Kalman:  x- = A x+ + B u(k-1);   x+ = x- + K (y(k) - C x-) */
    const float m0 = g->a11 * s->xk[0] + g->a12 * s->xk[1] + g->b1 * u_prev;
    const float m1 = g->a21 * s->xk[0] + g->a22 * s->xk[1] + g->b2 * u_prev;
    const float innov = y - m1;
    s->xk[0] = m0 + g->k1 * innov;
    s->xk[1] = m1 + g->k2 * innov;

    s->y_prev = y;
}

static void obs_reset(obs_state_t *s)
{
    memset(s, 0, sizeof(*s));
}

/* ================================================================
 *   ESP32: lectura de encoders/voltaje, promedio y publicacion
 * ================================================================ */
#ifndef OBS_HOST_TEST

static obs_gains_t gains[OBS_NUM_MOTORS];
static obs_state_t state[OBS_NUM_MOTORS];
static int32_t     c_prev[OBS_NUM_MOTORS];
static bool        first = true;

static float acc[OBS_MSG_LEN - 1];
static int   acc_n = 0;

static portMUX_TYPE obs_lock = portMUX_INITIALIZER_UNLOCKED;
static float    snap[OBS_MSG_LEN];
static uint32_t snap_count = 0;
static bool     snap_new = false;

static volatile TickType_t last_beat = 0;
static volatile bool       beat_seen = false;

void observers_init(void)
{
    for (int i = 0; i < OBS_NUM_MOTORS; i++) {
        design_gains(&OBS_PARAMS[i], &gains[i]);
        obs_reset(&state[i]);
    }
    first = true;
}

void observers_update(void)
{
    float vals[OBS_MSG_LEN - 1];

    for (int i = 0; i < OBS_NUM_MOTORS; i++) {
        const int32_t c = encoder_get_count(i);
        const int32_t d = c - c_prev[i];
        c_prev[i] = c;
        const float u_prev = motor_get_volts(i);   /* aun no se escribe el nuevo */

        float y = 0.0f;
        if (first || abs(d) > OBS_MAX_DCOUNTS) {
            obs_reset(&state[i]);                  /* arranque o reset del encoder */
        } else {
            y = (float)d * (float)homing_pro_dir_sign(i) * OBS_RAD_PER_COUNT / OBS_TS_S;
            obs_step(&gains[i], &state[i], y, u_prev);
        }
        float *v = &vals[i * OBS_VALS_PER_MOTOR];
        v[0] = y;
        v[1] = state[i].xl[1];
        v[2] = state[i].xk[1];
        v[3] = state[i].xl[0];
        v[4] = state[i].xk[0];
    }
    first = false;

    /* Promedio de OBS_DECIM muestras -> una muestra cada 40 ms */
    for (int j = 0; j < OBS_MSG_LEN - 1; j++) acc[j] += vals[j];
    if (++acc_n < OBS_DECIM) return;

    taskENTER_CRITICAL(&obs_lock);
    snap_count++;
    snap[0] = (float)snap_count;
    for (int j = 0; j < OBS_MSG_LEN - 1; j++) snap[j + 1] = acc[j] / (float)OBS_DECIM;
    snap_new = true;
    taskEXIT_CRITICAL(&obs_lock);

    memset(acc, 0, sizeof(acc));
    acc_n = 0;
}

void observers_keepalive(void)
{
    last_beat = xTaskGetTickCount();
    beat_seen = true;
}

bool observers_streaming(void)
{
    return beat_seen &&
           (xTaskGetTickCount() - last_beat) < pdMS_TO_TICKS(OBS_KEEPALIVE_MS);
}

bool observers_take(float out[OBS_MSG_LEN])
{
    bool ok;
    taskENTER_CRITICAL(&obs_lock);
    ok = snap_new;
    if (ok) {
        memcpy(out, snap, sizeof(snap));
        snap_new = false;
    }
    taskEXIT_CRITICAL(&obs_lock);
    return ok;
}

#endif /* OBS_HOST_TEST */
