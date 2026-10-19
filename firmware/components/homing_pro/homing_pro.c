/*
 * homing_pro.c - Ver homing_pro.h
 *
 * Cada motor sigue un "guion" de pasos (SCRIPT_Mx). Los pasos que mueven el
 * motor terminan con freno y una pausa corta antes del siguiente paso.
 * homing_pro_update() nunca bloquea: se llama cada 8 ms desde la tarea de
 * control.
 */
#include "homing_pro.h"
#include "motors.h"
#include "encoders.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ============================================================
 *                 PARAMETROS (AJUSTAR AQUI)
 * ============================================================ */

/* Valores que se marcan en el segundo toque de cada final */
#define M1_L_DEG        (-85.0f)
#define M1_R_DEG        ( 85.0f)
#define M2_L_DEG        (-90.0f)
#define M3_TOP_MM       (126.0f)    /* final UP (L): 126 mm del piso */
#define M3_BOTTOM_MM    ( 19.3f)    /* final DOWN (R): 19.3 mm del piso */

/* Donde se estaciona cada motor al final */
#define M1_HOME_DEG     (0.0f)
#define M2_HOME_DEG     (0.0f)
#define M3_HOME_MM      ((M3_TOP_MM + M3_BOTTOM_MM) / 2.0f)   /* 72.65 mm del piso (53.35 mm de cada final) */

/* Rango permitido para M2 (solo tiene final en -90) */
#define M2_MAX_DEG      (90.0f)

/* Relaciones de transmision: vueltas del MOTOR por vuelta de la salida.
 *   M1: NOMINAL, solo para calcular el retroceso antes de calibrar
 *       (la relacion real se mide entre -90 y +90).
 *   M2: se usa tal cual (no hay segundo final para calibrar).
 *   M3: NOMINAL motor -> tornillo, junto con AV_NOMINAL_MM, solo para el
 *       retroceso antes de calibrar (los mm/cuenta reales se miden). */
#define GEAR_M1_NOMINAL (50.0f * 4.0f)
#define GEAR_M2         (50.0f * 3.0f)
#define GEAR_M3_NOMINAL (50.0f * 3.0f)
#define AV_NOMINAL_MM   (2.0f)           /* mm por vuelta del tornillo (igual a av de la GUI) */

/* "Un tris": cuanto se devuelve antes del segundo toque */
#define BACKOFF_DEG     (2.0f)           /* M1 y M2 */
#define BACKOFF_MM_M3   (2.0f)           /* M3 */

#define BRAKE_PAUSE_MS      100      /* pausa con freno entre pasos */
#define POS_TOL_COUNTS      3        /* "llego" si esta a +-3 cuentas */
#define SEEK_TIMEOUT_MS     30000    /* max. buscando un final */
#define BACKOFF_TIMEOUT_MS  5000     /* max. para devolverse el "tris" */
#define MOVE_TIMEOUT_MS     30000    /* max. para ir a 0 / a la mitad */

/* La velocidad de todos los pasos es motors[i].homing_speed (motors.c). */

/* ============================================================
 *                        GUIONES
 * ============================================================ */

typedef enum {
    ST_SEEK_L = 0,   /* (1,0) hasta tocar el final L */
    ST_BACKOFF_L,    /* (0,1) un tris, alejandose de L */
    ST_MARK_L,       /* guarda la cuenta del final L */
    ST_SEEK_R,       /* (0,1) hasta tocar el final R */
    ST_BACKOFF_R,    /* (1,0) un tris, alejandose de R */
    ST_MARK_R,       /* guarda la cuenta del final R */
    ST_CALIB,        /* calcula la calibracion del motor */
    ST_MOVE_HOME,    /* va a 0 grados (M3: a la mitad) con el encoder */
    ST_END,
} step_t;

static const char *STEP_NAMES[] = {
    "SEEK_L", "BACKOFF_L", "MARK_L", "SEEK_R", "BACKOFF_R", "MARK_R",
    "CALIB", "MOVE_HOME", "END",
};

/* Doble toque en L y en R (M1 y M3) */
static const step_t SCRIPT_TWO_ENDS[] = {
    ST_SEEK_L, ST_BACKOFF_L, ST_SEEK_L, ST_MARK_L,
    ST_SEEK_R, ST_BACKOFF_R, ST_SEEK_R, ST_MARK_R,
    ST_CALIB, ST_MOVE_HOME, ST_END,
};

/* Doble toque solo en L (M2) */
static const step_t SCRIPT_ONE_END[] = {
    ST_SEEK_L, ST_BACKOFF_L, ST_SEEK_L, ST_MARK_L,
    ST_CALIB, ST_MOVE_HOME, ST_END,
};

static const step_t *const SCRIPT[NUM_MOTORS] = {
    SCRIPT_TWO_ENDS, SCRIPT_ONE_END, SCRIPT_TWO_ENDS,
};

/* ============================================================
 *                        CALIBRACION
 * ============================================================
 * q = base + (cuentas - c_ref) * k       (q en rad para M1/M2, mm para M3)
 */
typedef struct {
    bool    ok;
    float   base;
    int32_t c_ref;
    float   k;          /* unidades de q por cuenta (con signo) */
    int     dir_sign;   /* signo del cambio de cuentas con (1,0) */
    float   q_min, q_max;
} calib_t;

static portMUX_TYPE cal_lock = portMUX_INITIALIZER_UNLOCKED;
static calib_t cal[NUM_MOTORS];

static float deg2rad(float d) { return d * 0.01745329252f; }

/* Cuentas por unidad de salida, NOMINALES (solo para el "tris") */
static float backoff_counts(int m)
{
    const float cpr = (float)ENC_COUNTS_PER_REV;
    switch (m) {
    case 0:  return BACKOFF_DEG / 360.0f * cpr * GEAR_M1_NOMINAL;
    case 1:  return BACKOFF_DEG / 360.0f * cpr * GEAR_M2;
    default: return BACKOFF_MM_M3 / AV_NOMINAL_MM * cpr * GEAR_M3_NOMINAL;
    }
}

/* ============================================================
 *                         ESTADO
 * ============================================================ */

/* Banderas escritas desde otra tarea */
static volatile bool start_req  = false;
static volatile bool resume_req = false;
static volatile bool abort_req  = false;

static volatile hp_state_t state = HP_IDLE;
static volatile uint32_t   seq = 0;
static volatile int        cur_motor = 0;
static volatile int        step_idx = 0;
static const char         *error_reason = "";

static bool       step_started = false;   /* ya se inicializo el paso actual */
static bool       pausing = false;        /* pausa con freno entre pasos */
static TickType_t t0 = 0;                 /* inicio del paso o de la pausa */
static int32_t    c_step_start = 0;       /* cuenta al iniciar el paso */
static float      move_target = 0.0f;     /* MOVE_HOME: cuenta objetivo */
static int        move_err_sign = 0;      /* MOVE_HOME: signo inicial del error */
static bool       keep_step_start = false;/* Resume de un BACKOFF: no reiniciar c_step_start */

/* Marcas de cada motor */
static int32_t c_mark_l[NUM_MOTORS];
static int32_t c_mark_r[NUM_MOTORS];
static int32_t c_backoff_l_delta[NUM_MOTORS];   /* cambio de cuentas en BACKOFF_L */

static void bump(void) { seq++; }

static bool elapsed(TickType_t since, uint32_t ms)
{
    return (xTaskGetTickCount() - since) >= pdMS_TO_TICKS(ms);
}

static step_t cur_step(void) { return SCRIPT[cur_motor][step_idx]; }

static void fail(const char *reason)
{
    motors_stop_all();
    error_reason = reason;
    state = HP_ERROR;
    bump();
}

/* Termina un paso de movimiento: freno + pausa, luego el siguiente paso. */
static void finish_motion_step(int m)
{
    motor_brake(m);
    pausing = true;
    t0 = xTaskGetTickCount();
}

static void next_step(void)
{
    step_idx++;
    step_started = false;
    bump();
}

/* ============================================================
 *                        COMANDOS
 * ============================================================ */

void homing_pro_start(void)  { start_req = true; }
void homing_pro_resume(void) { resume_req = true; }
void homing_pro_abort(void)  { abort_req = true; }

hp_state_t homing_pro_get_state(void) { return state; }
uint32_t homing_pro_status_seq(void)  { return seq; }

static void reset_all(void)
{
    portENTER_CRITICAL(&cal_lock);
    for (int i = 0; i < NUM_MOTORS; i++) {
        cal[i].ok = false;
    }
    portEXIT_CRITICAL(&cal_lock);
    for (int i = 0; i < NUM_MOTORS; i++) {
        c_mark_l[i] = c_mark_r[i] = c_backoff_l_delta[i] = 0;
    }
    encoders_reset();
    keep_step_start = false;
    cur_motor = 0;
    step_idx = 0;
    step_started = false;
    pausing = false;
    error_reason = "";
}

/* Devuelve false si update no debe seguir en este ciclo. */
static bool handle_requests(void)
{
    if (abort_req) {
        abort_req = false;
        start_req = false;
        resume_req = false;
        motors_stop_all();
        if (state == HP_RUNNING) {
            state = HP_ABORTED;
            bump();
        }
        return false;
    }

    if (start_req) {
        start_req = false;
        resume_req = false;
        motors_stop_all();
        reset_all();
        state = HP_RUNNING;
        bump();
        return true;
    }

    if (resume_req) {
        resume_req = false;
        if (state == HP_ABORTED) {
            /* Repite el paso actual desde su inicio. BACKOFF conserva la
             * cuenta de partida, asi solo completa lo que le faltaba. */
            const step_t s = cur_step();
            keep_step_start = (s == ST_BACKOFF_L || s == ST_BACKOFF_R);
            step_started = false;
            pausing = false;
            state = HP_RUNNING;
            bump();
        } else if (state == HP_IDLE) {
            start_req = true;
            return handle_requests();
        }
        /* RUNNING, DONE o ERROR: no hace nada (ERROR necesita Start) */
    }
    return true;
}

/* ============================================================
 *                         PASOS
 * ============================================================ */

static void do_calib(int m)
{
    calib_t c = { 0 };

    if (m == 0) {
        /* M1: dos puntos, -90 en L y +90 en R */
        const int32_t span = c_mark_r[0] - c_mark_l[0];
        if (span == 0) { fail("NO_ENCODER"); return; }
        c.c_ref = c_mark_l[0];
        c.base  = deg2rad(M1_L_DEG);
        c.k     = deg2rad(M1_R_DEG - M1_L_DEG) / (float)span;
        c.dir_sign = (c_mark_l[0] > c_mark_r[0]) ? +1 : -1;   /* (1,0) va hacia L */
        c.q_min = deg2rad(M1_L_DEG);
        c.q_max = deg2rad(M1_R_DEG);
    } else if (m == 1) {
        /* M2: un punto (-90 en L) + relacion nominal. El signo se toma del
         * retroceso: con (0,1) se aleja de L, o sea hacia angulos mayores. */
        const int32_t d = c_backoff_l_delta[1];
        if (d == 0) { fail("NO_ENCODER"); return; }
        const int s01 = (d > 0) ? +1 : -1;     /* cuentas con (0,1) */
        c.c_ref = c_mark_l[1];
        c.base  = deg2rad(M2_L_DEG);
        c.k     = (float)s01 * 6.28318531f / ((float)ENC_COUNTS_PER_REV * GEAR_M2);
        c.dir_sign = -s01;
        c.q_min = deg2rad(M2_L_DEG);
        c.q_max = deg2rad(M2_MAX_DEG);
    } else {
        /* M3: dos puntos, 123 mm arriba (L) y 18 mm abajo (R) */
        const int32_t span = c_mark_l[2] - c_mark_r[2];   /* arriba - abajo */
        if (span == 0) { fail("NO_ENCODER"); return; }
        c.c_ref = c_mark_r[2];
        c.base  = M3_BOTTOM_MM;
        c.k     = (M3_TOP_MM - M3_BOTTOM_MM) / (float)span;
        c.dir_sign = (span > 0) ? +1 : -1;                /* (1,0) sube */
        c.q_min = M3_BOTTOM_MM;
        c.q_max = M3_TOP_MM;
    }
    c.ok = true;

    portENTER_CRITICAL(&cal_lock);
    cal[m] = c;
    portEXIT_CRITICAL(&cal_lock);
}

static float home_target(int m)
{
    switch (m) {
    case 0:  return deg2rad(M1_HOME_DEG);
    case 1:  return deg2rad(M2_HOME_DEG);
    default: return M3_HOME_MM;
    }
}

/* Ejecuta el paso actual del motor m. */
static void run_step(int m)
{
    const step_t s = cur_step();
    const int32_t c = encoder_get_count(m);

    if (!step_started) {
        step_started = true;
        t0 = xTaskGetTickCount();
        if (!keep_step_start) {
            c_step_start = c;
        }
        keep_step_start = false;
        motor_set_duty(m, motors[m].homing_speed);
        if (s == ST_MOVE_HOME) {
            move_target = homing_pro_joint_to_counts(m, home_target(m));
            const float err = move_target - (float)c;
            move_err_sign = (err >= 0.0f) ? +1 : -1;
        }
    }

    switch (s) {
    case ST_SEEK_L:
        motor_set_dir(m, 1, 0);
        if (limit_l_pressed(m)) {
            finish_motion_step(m);
        } else if (elapsed(t0, SEEK_TIMEOUT_MS)) {
            fail("SEEK_TIMEOUT");
        }
        break;

    case ST_SEEK_R:
        motor_set_dir(m, 0, 1);
        if (limit_r_pressed(m)) {
            finish_motion_step(m);
        } else if (elapsed(t0, SEEK_TIMEOUT_MS)) {
            fail("SEEK_TIMEOUT");
        }
        break;

    case ST_BACKOFF_L:
    case ST_BACKOFF_R: {
        if (s == ST_BACKOFF_L) {
            motor_set_dir(m, 0, 1);
        } else {
            motor_set_dir(m, 1, 0);
        }
        const int32_t moved = c - c_step_start;
        if ((float)abs(moved) >= backoff_counts(m)) {
            if (s == ST_BACKOFF_L) {
                c_backoff_l_delta[m] = moved;
            }
            finish_motion_step(m);
        } else if (elapsed(t0, BACKOFF_TIMEOUT_MS)) {
            fail("NO_ENCODER");   /* se movio pero el encoder no conto */
        }
        break;
    }

    case ST_MARK_L:
        c_mark_l[m] = c;
        next_step();
        break;

    case ST_MARK_R:
        c_mark_r[m] = c;
        next_step();
        break;

    case ST_CALIB:
        do_calib(m);
        if (state == HP_RUNNING) {
            next_step();
        }
        break;

    case ST_MOVE_HOME: {
        const float err = move_target - (float)c;
        const int err_sign = (err >= 0.0f) ? +1 : -1;
        const int dir = homing_pro_dir_sign(m);

        if (fabsf(err) <= (float)POS_TOL_COUNTS || err_sign != move_err_sign) {
            finish_motion_step(m);   /* llego (o se paso un poco): frena */
            break;
        }
        /* Si el error crece mucho, va en la direccion equivocada */
        const float err0 = fabsf(move_target - (float)c_step_start);
        if (fabsf(err) > err0 + 2.0f * backoff_counts(m)) {
            fail("WRONG_DIR");
            break;
        }
        if (err_sign == dir) {
            motor_set_dir(m, 1, 0);
            if (limit_l_pressed(m)) { fail("HIT_LIMIT"); break; }
        } else {
            motor_set_dir(m, 0, 1);
            if (limit_r_pressed(m)) { fail("HIT_LIMIT"); break; }
        }
        if (elapsed(t0, MOVE_TIMEOUT_MS)) {
            fail("MOVE_TIMEOUT");
        }
        break;
    }

    case ST_END:
        break;
    }
}

void homing_pro_update(void)
{
    if (!handle_requests() || state != HP_RUNNING) {
        return;
    }

    const int m = cur_motor;

    /* Pausa con freno despues de cada paso de movimiento */
    if (pausing) {
        motor_brake(m);
        if (elapsed(t0, BRAKE_PAUSE_MS)) {
            pausing = false;
            next_step();
        }
        return;
    }

    if (cur_step() == ST_END) {
        motor_coast(m);
        if (m + 1 < NUM_MOTORS) {
            cur_motor = m + 1;
            step_idx = 0;
            step_started = false;
            bump();
        } else {
            motors_stop_all();
            state = HP_DONE;
            bump();
        }
        return;
    }

    run_step(m);
}

void homing_pro_status_str(char *buf, size_t len)
{
    switch (state) {
    case HP_IDLE:
        snprintf(buf, len, "IDLE");
        break;
    case HP_DONE:
        snprintf(buf, len, "HOMED");
        break;
    case HP_RUNNING:
        snprintf(buf, len, "HOMING M%d %s", cur_motor + 1, STEP_NAMES[cur_step()]);
        break;
    case HP_ABORTED:
        snprintf(buf, len, "ABORTED M%d %s", cur_motor + 1, STEP_NAMES[cur_step()]);
        break;
    case HP_ERROR:
        snprintf(buf, len, "ERROR M%d %s", cur_motor + 1, error_reason);
        break;
    }
}

/* ============================================================
 *                 CONVERSIONES (para control y GUI)
 * ============================================================ */

static calib_t get_cal(int i)
{
    calib_t c;
    portENTER_CRITICAL(&cal_lock);
    c = cal[i];
    portEXIT_CRITICAL(&cal_lock);
    return c;
}

bool homing_pro_calibrated(int i)
{
    return i >= 0 && i < NUM_MOTORS && get_cal(i).ok;
}

float homing_pro_counts_to_joint(int i, int32_t counts)
{
    const calib_t c = get_cal(i);
    if (!c.ok) return 0.0f;
    return c.base + (float)(counts - c.c_ref) * c.k;
}

float homing_pro_joint_to_counts(int i, float q)
{
    const calib_t c = get_cal(i);
    if (!c.ok) return 0.0f;
    return (float)c.c_ref + (q - c.base) / c.k;
}

int homing_pro_dir_sign(int i)
{
    const calib_t c = get_cal(i);
    return c.ok ? c.dir_sign : +1;
}

bool homing_pro_in_range(int i, float q)
{
    float lo, hi;
    switch (i) {
    case 0:  lo = deg2rad(M1_L_DEG); hi = deg2rad(M1_R_DEG);   break;
    case 1:  lo = deg2rad(M2_L_DEG); hi = deg2rad(M2_MAX_DEG); break;
    default: lo = M3_BOTTOM_MM;      hi = M3_TOP_MM;           break;
    }
    return q >= lo && q <= hi;
}

float homing_pro_clamp_joint(int i, float q)
{
    const calib_t c = get_cal(i);
    if (!c.ok) return q;
    if (q < c.q_min) return c.q_min;
    if (q > c.q_max) return c.q_max;
    return q;
}
