/*
 * profile.c - Ver profile.h
 *
 * Mezcla por SUPERPOSICION: cada tramo k es un movimiento reposo-reposo de
 * D_k = meta_k - meta_(k-1) en T_k segundos. La referencia es
 *
 *     q_ref(t) = base + suma_k  s_k(t - t_k) * D_k
 *
 * donde base acumula los tramos ya terminados y s(.) va de 0 a 1. Si el PC
 * manda el tramo k+1 antes de que termine el k, los dos se suman: el robot
 * empieza a viajar en XY mientras Z termina de subir, sin frenar en el
 * punto intermedio. Como cada tramo es monotono y reposo-reposo, NO hay
 * sobrepaso y la meta final es exacta (base termina valiendo la ultima meta).
 *
 * Forma de s(tau), tau = t / T en [0, 1]:
 *   th1, th2 -> quintico   s = 10 tau^3 - 15 tau^4 + 6 tau^5
 *   z        -> trapezoide (f = PROF_TRAP_FRAC, vc = 1 / (1 - f)):
 *               tau <= f      : s = vc tau^2 / (2 f)
 *               f < tau < 1-f : s = vc (f/2 + tau - f)
 *               tau >= 1-f    : s = 1 - vc (1 - tau)^2 / (2 f)
 */
#include "profile.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define N_AX    3
#define Z_AXIS  2
#define MAX_SEG 4          /* tramos que pueden estar solapados a la vez */

static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;

/* Pedido pendiente (lo escribe el nucleo 0) */
static bool     pend = false;
static joints_t pend_goal;
static float    pend_T;
static uint32_t pend_id;
static bool     cancel_req = false;

/* Estado (nucleo 1) */
typedef struct {
    bool     used;
    float    D[N_AX];
    float    T, t;
    uint32_t id;
} seg_t;

static seg_t seg[MAX_SEG];
static float base[N_AX];        /* posicion de los tramos ya terminados */
static float last_goal[N_AX];   /* meta del ultimo tramo aceptado */
static bool  have_state = false;

/* Publicados (los lee el nucleo 0) */
static volatile bool     running = false;
static volatile int      progress = 100;
static volatile uint32_t goal_id = 0;

static float q_of(const joints_t *j, int i) { return i == 0 ? j->th1 : i == 1 ? j->th2 : j->th3; }
static void  set_q(joints_t *j, int i, float v)
{
    if (i == 0) j->th1 = v; else if (i == 1) j->th2 = v; else j->th3 = v;
}

static float s_quintic(float tau)
{
    return tau * tau * tau * (10.0f + tau * (-15.0f + 6.0f * tau));
}

static float s_trap(float tau)
{
    const float f = PROF_TRAP_FRAC, vc = 1.0f / (1.0f - f);
    if (tau <= f)        return vc * tau * tau / (2.0f * f);
    if (tau < 1.0f - f)  return vc * (0.5f * f + tau - f);
    const float r = 1.0f - tau;
    return 1.0f - vc * r * r / (2.0f * f);
}

/* ---------------- API ---------------- */
bool prof_request(const joints_t *goal, float T, uint32_t id)
{
    portENTER_CRITICAL(&lock);
    const bool dup = (pend && pend_id == id) || (!pend && goal_id == id);
    if (!dup) {
        pend_goal = *goal;
        pend_T = (T < PROF_MIN_T) ? PROF_MIN_T : T;
        pend_id = id;
        pend = true;
        cancel_req = false;
    }
    portEXIT_CRITICAL(&lock);
    return !dup;
}

void prof_cancel(void)
{
    portENTER_CRITICAL(&lock);
    pend = false;
    cancel_req = true;
    running = false;
    progress = 100;
    goal_id = 0;
    portEXIT_CRITICAL(&lock);
}

bool prof_step(float dt, const joints_t *start, joints_t *ref)
{
    bool take = false, cancel;
    joints_t g;
    float T = 0.0f;
    uint32_t id = 0;

    portENTER_CRITICAL(&lock);
    cancel = cancel_req;
    cancel_req = false;
    if (pend) {
        take = true;
        g = pend_goal; T = pend_T; id = pend_id;
        pend = false;
    }
    portEXIT_CRITICAL(&lock);

    if (cancel) {
        for (int k = 0; k < MAX_SEG; k++) seg[k].used = false;
        have_state = false;
    }

    if (take) {
        if (!have_state) {
            for (int i = 0; i < N_AX; i++) base[i] = last_goal[i] = q_of(start, i);
            have_state = true;
        }
        /* Buscar un lugar; si estan todos ocupados, el que menos le falta se termina ya */
        int k = 0;
        while (k < MAX_SEG && seg[k].used) k++;
        if (k == MAX_SEG) {
            int oldest = 0;
            for (int j = 1; j < MAX_SEG; j++) {
                if (seg[j].T - seg[j].t < seg[oldest].T - seg[oldest].t) oldest = j;
            }
            for (int i = 0; i < N_AX; i++) base[i] += seg[oldest].D[i];
            k = oldest;
        }
        for (int i = 0; i < N_AX; i++) {
            seg[k].D[i] = q_of(&g, i) - last_goal[i];
            last_goal[i] = q_of(&g, i);
        }
        seg[k].T = T;
        seg[k].t = 0.0f;
        seg[k].id = id;
        seg[k].used = true;
        goal_id = id;
    }

    if (!have_state) return false;

    /* Avanzar los tramos y sumar */
    float q[N_AX] = { base[0], base[1], base[2] };
    bool any = false;
    int newest_pg = 100;
    for (int k = 0; k < MAX_SEG; k++) {
        if (!seg[k].used) continue;
        seg[k].t += dt;
        if (seg[k].t >= seg[k].T) {
            for (int i = 0; i < N_AX; i++) {
                base[i] += seg[k].D[i];
                q[i] += seg[k].D[i];
            }
            seg[k].used = false;
            continue;
        }
        const float tau = seg[k].t / seg[k].T;
        const float sq = s_quintic(tau), sz = s_trap(tau);
        for (int i = 0; i < N_AX; i++) q[i] += (i == Z_AXIS ? sz : sq) * seg[k].D[i];
        any = true;
        if (seg[k].id == goal_id) {
            newest_pg = (int)(100.0f * tau);
            if (newest_pg > 99) newest_pg = 99;
        }
    }
    if (!any) {
        for (int i = 0; i < N_AX; i++) q[i] = base[i] = last_goal[i];   /* exacto */
    }
    for (int i = 0; i < N_AX; i++) set_q(ref, i, q[i]);

    portENTER_CRITICAL(&lock);
    if (!cancel_req) {
        running = any;
        progress = any ? newest_pg : 100;
    }
    portEXIT_CRITICAL(&lock);
    return any;
}

bool prof_running(void)
{
    bool r;
    portENTER_CRITICAL(&lock);
    r = running || pend;
    portEXIT_CRITICAL(&lock);
    return r;
}

int prof_progress(void)        { return progress; }
uint32_t prof_goal_id(void)    { return goal_id; }
