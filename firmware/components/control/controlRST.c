/*
 * controlRST.c - Control RST en CASCADA de las 3 articulaciones. Ver control.h
 *
 * Se compila SOLO si en control.h  CONTROL_USE_RST = 1  (si es 0 se usa el
 * PD de control.c). Implementa la misma API que control.c, asi que main.c,
 * la IK y las trayectorias no cambian.
 *
 * Mismo esqueleto que control.c (y que el control de pH):
 *   leer sensor -> ecuacion en diferencias -> saturar -> actuador
 *   -> guardar la memoria para la siguiente muestra.
 * Se conservan TODAS las protecciones de control.c: dead-man, finales de
 * carrera, solo con homing hecho, nada mientras corre la prueba de
 * circuito, reset al terminar el homing, spinlock de la referencia,
 * histeresis de "llego" (control_arrived) y freno activo.
 *
 * Por cada motor hay dos lazos:
 *
 *   theta_ref --> [ RST externo ] -- w_ref --> [ RST interno ] -- u --> motor
 *   (posicion)     posicional       (sat.)     incremental     (+-12 V)
 *                  Ts = 8 ms                   Ts = 8 ms
 *                    ^  theta_k                  ^  omega_k
 *                    |__________ encoder ________|
 *
 * Unidades (las de la identificacion, ANTES de la reductora):
 *   theta  [rad del EJE DEL MOTOR]
 *   omega  [rad/s del EJE DEL MOTOR]
 *   u      [V]  -> duty de 10 bits  (u = %PWM * 12 / 100)
 *
 * Ley RST (T escalar):   R(q^-1) u[k] = T r[k] - S(q^-1) y[k]
 *   u[k] = ( T r[k] - s0 y[k] - s1 y[k-1] - s2 y[k-2]
 *            - r1 u[k-1] - r2 u[k-2] ) / r0
 */
#include "control.h"

#if CONTROL_USE_RST   /* ---- RST: solo si el selector de control.h = 1 ---- */
#include <math.h>
#include "encoders.h"
#include "motors.h"
#include "homing_pro.h"
#include "motor_test.h"
#include "profile.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define NUM_JOINTS 3


/* ============================================================
 *                    TIEMPOS DE MUESTREO
 * ============================================================
 * Interno (velocidad): cada llamada de la tarea -> CONTROL_LOOP_MS = 8 ms
 * Externo (posicion):  cada RST_OUT_EVERY llamadas del interno.
 *
 * RST_OUT_EVERY = 1 -> externo tambien a 8 ms (Ts = 8 ms para ambos).
 *   Con los coeficientes de abajo, el externo SOLO es estable a 8 ms
 *   (simulado con Gpos1..3 de NPI_Proyecto.mlx: a 8 ms llega sin
 *   sobrepaso en ~0.8 s; a 32 ms oscila sin parar).
 *   Si se redisena el externo a 32 ms, poner 4 aqui.
 * Deben ser los MISMOS Ts del diseno en MATLAB.
 */
#define RST_OUT_EVERY  1

#define TS_IN_MS    CONTROL_LOOP_MS
#define TS_OUT_MS   (CONTROL_LOOP_MS * RST_OUT_EVERY)
#define TS_IN_S     ((float)TS_IN_MS / 1000.0f)
#define OUT_EVERY   RST_OUT_EVERY

#if RST_OUT_EVERY < 1
#error "RST_OUT_EVERY debe ser >= 1"
#endif


/* ============================================================
 *        COEFICIENTES DE LOS CONTROLADORES  (LLENAR AQUI)
 * ============================================================
 * Tal cual salen de MATLAB (R = [r0 r1 r2], S = [s0 s1 s2], T escalar).
 *
 * LAZO INTERNO (incremental): el R que da MATLAB NO incluye el integrador.
 * Aqui se escribe el R REDUCIDO [1 r1'] y el R completo se arma solo en
 * rst_init_coefs():
 *     R(q^-1) = (1 - q^-1) (1 + r1' q^-1) = 1 + (r1' - 1) q^-1 - r1' q^-2
 *
 * LAZO EXTERNO (posicional): el R ya esta completo.
 */
typedef struct {
    float R[3];     /* r0, r1, r2 */
    float S[3];     /* s0, s1, s2 */
    float T;
} rst_coef_t;

/* Interno: R reducido {1, r1'} (sin integrador), S, T */
static const float IN_R_RED[NUM_JOINTS][2] = {
    { 1.0f, 0.0303141f },   /* motor 1 */
    { 1.0f, 0.0306875f },   /* motor 2 */
    { 1.0f, 0.030556f  },   /* motor 3 */
};
static const float IN_S[NUM_JOINTS][3] = {
    { 0.0459611f, -0.03805f,   1.22228e-9f },   /* motor 1 */
    { 0.0512488f, -0.04256f,   1.35405e-9f },   /* motor 2 */
    { 0.0441946f, -0.0366618f, 1.17036e-9f },   /* motor 3 */
};
static const float IN_T[NUM_JOINTS] = { 0.00791113f, 0.00868888f, 0.00753285f };

/* Externo: R completo, S, T */
static const rst_coef_t OUT_COEF[NUM_JOINTS] = {
    /* motor 1 */
    { { 1.0f, 0.351363f, 0.00202767f }, { 419.777f, -717.048f, 308.189f }, 10.9176f },
    /* motor 2 */
    { { 1.0f, 0.351697f, 0.00203352f }, { 419.588f, -716.704f, 308.034f }, 10.9176f },
    /* motor 3 */
    { { 1.0f, 0.351579f, 0.00203146f }, { 419.654f, -716.825f, 308.089f }, 10.9176f },
};

static rst_coef_t IN_COEF[NUM_JOINTS];   /* interno con el R completo */

static void rst_init_coefs(void)
{
    for (int i = 0; i < NUM_JOINTS; i++) {
        const float r0 = IN_R_RED[i][0];
        const float r1 = IN_R_RED[i][1];
        /* (1 - q^-1)(r0 + r1 q^-1) = r0 + (r1 - r0) q^-1 - r1 q^-2 */
        IN_COEF[i].R[0] = r0;
        IN_COEF[i].R[1] = r1 - r0;
        IN_COEF[i].R[2] = -r1;
        IN_COEF[i].S[0] = IN_S[i][0];
        IN_COEF[i].S[1] = IN_S[i][1];
        IN_COEF[i].S[2] = IN_S[i][2];
        IN_COEF[i].T    = IN_T[i];
    }
}


/* ============================================================
 *                       SATURACIONES
 * ============================================================
 * w_ref: 200 RPM en la SALIDA de la caja reductora (N = 50), pasado a
 *        rad/s del eje del motor (unidades del lazo interno):
 *        200 RPM * 50 * 2*pi/60 = 1047 rad/s.
 * u:     +-12 V (driver).
 */
#define GEARBOX_N         50.0f
#define W_MAX_RPM_SALIDA  200.0f
#define W_MAX_RAD_S       (W_MAX_RPM_SALIDA * GEARBOX_N * 6.28318531f / 60.0f)

#define V_SUPPLY   12.0f        /* voltaje de la fuente de los motores */
#define U_MAX_V    12.0f        /* saturacion fisica del driver */
#define PWM_MAX    1023.0f


/* ============================================================
 *                 MEDICION DE VELOCIDAD (omega)
 * ============================================================
 * omega[k] = (cuentas[k] - cuentas[k-1]) * rad/cuenta / Ts_in
 * Con 64 cuentas por vuelta y Ts = 8 ms, 1 cuenta = 12.3 rad/s.
 *
 * OMEGA_ALPHA: filtro pasa-bajos de primer orden opcional
 *     w_f[k] = OMEGA_ALPHA * omega[k] + (1 - OMEGA_ALPHA) * w_f[k-1]
 *   1.0f -> SIN filtro (derivada pura; lo que se recomienda primero).
 *   Bajarlo (p. ej. 0.5f) solo si u se ve muy ruidoso; ojo, el filtro
 *   agrega retardo que el diseno RST no tiene en cuenta.
 */
#define OMEGA_ALPHA  1.0f

/* ============================================================
 *        FEEDFORWARD DE VELOCIDAD (trayectorias, profile.c)
 * ============================================================
 * Mientras el perfil del ESP32 mueve la referencia, a la salida del externo
 * (w_ref) se le suma la velocidad de la propia referencia (derivada de las
 * cuentas objetivo, en rad/s del motor). El interno es un lazo de
 * VELOCIDAD, asi que sigue el perfil casi sin retraso y el externo solo
 * corrige el error que quede.
 *   1.0 -> feedforward completo.  0.0 -> apagado (solo el RST, como antes).
 * Si ves sobrepaso al final de los tramos, bajarlo (0.7 - 0.9).
 * Se cambia EN VIVO desde la pestana Trayectoria (comando "ff <valor>" por
 * /scara/cmd, ver control_set_ff_gain); FF_GAIN es el valor al arrancar.
 */
#define FF_GAIN      1.0f
static volatile float ff_gain = FF_GAIN;

void control_set_ff_gain(float g)
{
    if (g < FF_GAIN_MIN) g = FF_GAIN_MIN;
    if (g > FF_GAIN_MAX) g = FF_GAIN_MAX;
    ff_gain = g;
}

float control_get_ff_gain(void) { return ff_gain; }

/* Radianes del EJE DEL MOTOR por cuenta del encoder. */
#define MOTOR_RAD_PER_COUNT  (6.28318531f / (float)ENC_COUNTS_PER_REV)


/* ============================================================
 *          HISTERESIS DE "LLEGO" (igual que control.c)
 * ============================================================
 * Entra a "llego" con menos de DEAD_TOL_COUNTS de error de posicion y solo
 * vuelve a controlar si se aleja mas de DEAD_OUT_COUNTS. Ya en el
 * objetivo: u = 0 (freno activo) y memoria de los dos lazos en cero, para
 * que al volver a arrancar no salte. Sin compensacion de zona muerta: el
 * lazo interno tiene integrador.
 */
#define DEAD_TOL_COUNTS   10
#define DEAD_OUT_COUNTS   25
static bool llego[NUM_JOINTS];


/* ============================================================
 *                 REFERENCIA (compartida entre nucleos)
 * ============================================================ */
static portMUX_TYPE ref_lock = portMUX_INITIALIZER_UNLOCKED;
static joints_t ref = { 0.0f, 0.0f, 0.0f };
static bool     has_ref = false;
static volatile uint32_t ref_seq = 0;

void control_set_reference(const joints_t *r)
{
    prof_cancel();          /* consigna directa (Punto a punto, DK, Wiimote): sin perfil */
    portENTER_CRITICAL(&ref_lock);
    ref = *r;
    has_ref = true;
    ref_seq++;
    for (int i = 0; i < NUM_JOINTS; i++) llego[i] = false;
    portEXIT_CRITICAL(&ref_lock);
}

bool control_get_reference(joints_t *r)
{
    portENTER_CRITICAL(&ref_lock);
    *r = ref;
    bool ok = has_ref;
    portEXIT_CRITICAL(&ref_lock);
    return ok;
}

uint32_t control_reference_seq(void)
{
    return ref_seq;
}

static void clear_reference(void)
{
    portENTER_CRITICAL(&ref_lock);
    has_ref = false;
    portEXIT_CRITICAL(&ref_lock);
}

void control_release(void)
{
    prof_cancel();
    clear_reference();
}

/* TRAYECTORIA: meta con perfil (ver control.h y profile.h). Primero se pide
 * el perfil y DESPUES se cambia la referencia: asi el nucleo 1 nunca ve la
 * meta final sin su perfil (no hay un salto de 8 ms). */
void control_set_goal(const joints_t *goal, float duration_s, uint32_t id)
{
    if (!prof_request(goal, duration_s, id)) return;   /* reenvio: ya la tiene */
    portENTER_CRITICAL(&ref_lock);
    ref = *goal;
    has_ref = true;
    ref_seq++;
    for (int i = 0; i < NUM_JOINTS; i++) llego[i] = false;
    portEXIT_CRITICAL(&ref_lock);
}


/* ============================================================
 *                 DEAD-MAN SWITCH (hold-to-run)
 * ============================================================ */
static volatile bool       dm_held = false;
static volatile TickType_t dm_last = 0;

void control_deadman_beat(void)
{
    dm_last = xTaskGetTickCount();
    dm_held = true;
}

void control_deadman_release(void)
{
    dm_held = false;
}

static bool deadman_ok(void)
{
    if (!dm_held) return false;
    if ((xTaskGetTickCount() - dm_last) >= pdMS_TO_TICKS(DEADMAN_TIMEOUT_MS)) {
        dm_held = false;
        return false;
    }
    return true;
}


/* ============================================================
 *                      UTILIDADES
 * ============================================================ */
static float limitar(float valor, float minimo, float maximo)
{
    if (valor > maximo) return maximo;
    if (valor < minimo) return minimo;
    return valor;
}

static float joint_of(const joints_t *q, int i)
{
    return (i == 0) ? q->th1 : (i == 1) ? q->th2 : q->th3;
}

void control_get_position(joints_t *pos)
{
    pos->th1 = homing_pro_counts_to_joint(0, encoder_get_count(0));
    pos->th2 = homing_pro_counts_to_joint(1, encoder_get_count(1));
    pos->th3 = homing_pro_counts_to_joint(2, encoder_get_count(2));
}


/* ============================================================
 *        MEDICIONES EN UNIDADES DEL MOTOR (con el signo correcto)
 * ============================================================
 * Con el signo del homingPRO, theta crece cuando u > 0, es decir
 * (IN1, IN2) = (1, 0). Asi el lazo es de realimentacion negativa sin
 * importar hacia donde cuente cada encoder.
 */
static float counts_to_motor_rad(int i, float counts)
{
    return counts * (float)homing_pro_dir_sign(i) * MOTOR_RAD_PER_COUNT;
}

/* Referencia articular -> cuentas objetivo (dentro del rango medido) */
static float ref_counts(int i, float q_ref)
{
    return homing_pro_joint_to_counts(i, homing_pro_clamp_joint(i, q_ref));
}


/* ============================================================
 *                 MEMORIA DE LOS CONTROLADORES
 * ============================================================
 * En u[k-1], u[k-2] (y w[k-1], w[k-2]) se guarda el valor YA saturado:
 * como el interno tiene integrador, asi no se acumula error mientras el
 * motor esta al maximo (anti-windup simple, igual que en control.c).
 * El externo trabaja en double: theta llega a miles de rad del motor
 * (sobre todo el eje Z) y S tiene coeficientes grandes; con float se
 * perderia precision en T*r - S*y. Son pocas operaciones por muestra.
 */
typedef struct {
    float  y1, y2;       /* omega[k-1], omega[k-2] */
    float  u1, u2;       /* u[k-1], u[k-2] (saturados) */
} rst_in_mem_t;

typedef struct {
    double y1, y2;       /* theta[k-1], theta[k-2] */
    double w1, w2;       /* w_ref[k-1], w_ref[k-2] (saturados) */
} rst_out_mem_t;

typedef struct {
    int32_t c_prev;      /* cuentas en la muestra anterior (para omega) */
    float   w_f;         /* omega filtrada */
    float   w_ref;       /* salida del externo, se mantiene entre muestras */
    bool    first;       /* primera muestra: inicializar memoria */
    float   cr_prev;     /* cuentas objetivo en el paso anterior (feedforward) */
    bool    cr_valid;
} rst_aux_t;

static rst_in_mem_t  mem_in[NUM_JOINTS];
static rst_out_mem_t mem_out[NUM_JOINTS];
static rst_aux_t     aux[NUM_JOINTS];

static void reset_motor(int i)
{
    mem_in[i]  = (rst_in_mem_t){ 0.0f, 0.0f, 0.0f, 0.0f };
    mem_out[i] = (rst_out_mem_t){ 0.0, 0.0, 0.0, 0.0 };
    aux[i].c_prev = encoder_get_count(i);
    aux[i].w_f    = 0.0f;
    aux[i].w_ref  = 0.0f;
    aux[i].first  = true;
    aux[i].cr_valid = false;
}

static void reset_controladores(void)
{
    for (int i = 0; i < NUM_JOINTS; i++) {
        reset_motor(i);
        llego[i] = false;
    }
}


/* ============================================================
 *          LAZO EXTERNO: posicion -> w_ref  (RST posicional)
 * ============================================================
 *   w[k] = ( T th_ref[k] - s0 th[k] - s1 th[k-1] - s2 th[k-2]
 *            - r1 w[k-1] - r2 w[k-2] ) / r0
 */
static float rst_externo(int i, double th_ref, double th)
{
    const rst_coef_t *c = &OUT_COEF[i];
    rst_out_mem_t    *m = &mem_out[i];

    if (aux[i].first) {
        /* Arranque: memoria de posicion = posicion actual (sin salto) */
        m->y1 = th;  m->y2 = th;
        m->w1 = 0.0; m->w2 = 0.0;
    }

    double w = ((double)c->T * th_ref
                - (double)c->S[0] * th - (double)c->S[1] * m->y1 - (double)c->S[2] * m->y2
                - (double)c->R[1] * m->w1 - (double)c->R[2] * m->w2) / (double)c->R[0];

    /* Saturar la referencia de velocidad (200 RPM en la salida) */
    if (w >  W_MAX_RAD_S) w =  W_MAX_RAD_S;
    if (w < -W_MAX_RAD_S) w = -W_MAX_RAD_S;

    m->y2 = m->y1;  m->y1 = th;
    m->w2 = m->w1;  m->w1 = w;
    return (float)w;
}


/* ============================================================
 *          LAZO INTERNO: velocidad -> u  (RST incremental)
 * ============================================================
 * Con el R completo (incluye el integrador):
 *   u[k] = ( T w_ref[k] - s0 w[k] - s1 w[k-1] - s2 w[k-2]
 *            - r1 u[k-1] - r2 u[k-2] ) / r0
 */
static float rst_interno(int i, float w_ref, float omega)
{
    const rst_coef_t *c = &IN_COEF[i];
    rst_in_mem_t     *m = &mem_in[i];

    float u = (c->T * w_ref
               - c->S[0] * omega - c->S[1] * m->y1 - c->S[2] * m->y2
               - c->R[1] * m->u1 - c->R[2] * m->u2) / c->R[0];

    /* Saturacion fisica del driver (+-12 V) */
    u = limitar(u, -U_MAX_V, U_MAX_V);

    m->y2 = m->y1;  m->y1 = omega;
    m->u2 = m->u1;  m->u1 = u;
    return u;
}


/* ============================================================
 *                   ESCRIBIR AL MOTOR  (igual que control.c)
 * ============================================================
 * u > 0 -> (IN1, IN2) = (1, 0);  u < 0 -> (0, 1)
 * u = 0 -> FRENO ACTIVO (1, 1) con PWM al maximo.
 * duty = |u| / 12 V * 1023   (u = %PWM * 12 / 100)
 */
static void escribir_actuador(int i, float u)
{
    if (u == 0.0f) {
        motor_brake(i);
        motor_set_duty(i, (uint32_t)PWM_MAX);
        return;
    }

    float duty = limitar(u, -U_MAX_V, U_MAX_V) / V_SUPPLY * PWM_MAX;

    if (duty >= 0.0f) {
        motor_set_dir(i, 1, 0);
    } else {
        motor_set_dir(i, 0, 1);
        duty = -duty;
    }
    motor_set_duty(i, (uint32_t)(duty + 0.5f));
}


/* ============================================================
 *                   CICLO DE CONTROL
 * ============================================================ */

/* Un paso de los 3 motores, cada TS_IN_MS (8 ms).
 * run_outer = true cada OUT_EVERY pasos: corre tambien el externo.
 * moving = true mientras el perfil del ESP32 mueve la referencia. */
static void control_step(const joints_t *r, bool run_outer, bool moving)
{
    for (int i = 0; i < NUM_JOINTS; i++) {
        /* 1. Leer sensor */
        const int32_t c  = encoder_get_count(i);
        const float   cr = ref_counts(i, joint_of(r, i));

        /* 2. Histeresis de "llego" (en cuentas, como control.c). Mientras el
         *    perfil se mueve NO aplica: si no, el motor frenaria y se le
         *    borraria la memoria cada vez que alcanza a la referencia. */
        const float err_cts = fabsf(cr - (float)c);
        if (moving) {
            llego[i] = false;
        } else if (llego[i]) {
            if (err_cts > (float)DEAD_OUT_COUNTS) {
                llego[i] = false;
                reset_motor(i);          /* vuelve a arrancar desde cero */
            }
        } else if (err_cts <= (float)DEAD_TOL_COUNTS) {
            llego[i] = true;
        }

        if (llego[i]) {
            /* Ya llego: freno, y la memoria queda lista para arrancar limpio */
            reset_motor(i);
            escribir_actuador(i, 0.0f);
            continue;
        }

        /* 3. Velocidad del motor [rad/s] */
        const float omega_raw = counts_to_motor_rad(i, (float)(c - aux[i].c_prev)) / TS_IN_S;
        aux[i].c_prev = c;
        aux[i].w_f = aux[i].first ? omega_raw
                                  : OMEGA_ALPHA * omega_raw + (1.0f - OMEGA_ALPHA) * aux[i].w_f;
        const float omega_k = aux[i].w_f;

        /* 4. LAZO EXTERNO (cada TS_OUT_MS, y siempre en la primera muestra):
         *    posicion -> w_ref. Entre muestras se mantiene w_ref (ZOH). */
        if (run_outer || aux[i].first) {
            const double th_ref = (double)cr * (double)homing_pro_dir_sign(i) * (double)MOTOR_RAD_PER_COUNT;
            const double th_k   = (double)c  * (double)homing_pro_dir_sign(i) * (double)MOTOR_RAD_PER_COUNT;
            aux[i].w_ref = rst_externo(i, th_ref, th_k);
        }

        /* 4b. Feedforward: velocidad de la referencia [rad/s del motor] */
        float w_ff = 0.0f;
        if (moving && aux[i].cr_valid) {
            w_ff = counts_to_motor_rad(i, cr - aux[i].cr_prev) / TS_IN_S;
        }
        aux[i].cr_prev = cr;
        aux[i].cr_valid = true;
        const float w_cmd = limitar(aux[i].w_ref + ff_gain * w_ff, -W_MAX_RAD_S, W_MAX_RAD_S);

        /* 5. LAZO INTERNO (cada 8 ms): velocidad -> u [V] */
        if (aux[i].first) {
            /* Arranque: memoria de velocidad = velocidad actual */
            mem_in[i].y1 = omega_k;  mem_in[i].y2 = omega_k;
        }
        float u = rst_interno(i, w_cmd, omega_k);
        aux[i].first = false;

        /* 6. Finales de carrera: si el motor esta tocando el final hacia
         *    el que lo mandan, se para. Se limpia la memoria del interno
         *    para que el integrador no se cargue contra el final. */
        if ((u > 0.0f && limit_l_pressed(i)) || (u < 0.0f && limit_r_pressed(i))) {
            u = 0.0f;
            mem_in[i].u1 = 0.0f;  mem_in[i].u2 = 0.0f;
        }

        escribir_actuador(i, u);
    }
}

static bool any_motor_test_running(void)
{
    for (int i = 0; i < NUM_JOINTS; i++) {
        if (motor_test_is_running(i)) return true;
    }
    return false;
}

void control_update(void)
{
    static bool     coefs_ok = false;
    static bool     homed_before = false;
    static bool     active = false;
    static uint32_t tick = 0;

    if (!coefs_ok) {
        rst_init_coefs();
        reset_controladores();
        coefs_ok = true;
    }

    const bool homed = (homing_pro_get_state() == HP_DONE);

    /* 1. Al terminar el homing: memoria limpia y se descarta cualquier
     *    consigna que llegara antes. */
    if (homed && !homed_before) {
        reset_controladores();
        prof_cancel();
        clear_reference();
        tick = 0;
    }
    homed_before = homed;

    /* 2. Solo se controla con homing hecho, con consigna, sin prueba de
     *    circuito y con el dead-man presionado. */
    joints_t r;
    if (!homed || !control_get_reference(&r) || any_motor_test_running() ||
        !deadman_ok()) {
        if (active) {
            if (homed) motors_stop_all();
            reset_controladores();
            active = false;
        }
        tick = 0;
        return;
    }
    if (!active) {
        reset_controladores();   /* arranque limpio (cuentas previas actuales) */
        active = true;
        tick = 0;
    }

    /* 3. Perfil del ESP32 (trayectorias): reemplaza r por la referencia del
     *    instante. Solo avanza aqui, con el control activo (dead-man). */
    joints_t meas;
    control_get_position(&meas);
    const bool moving = prof_step(TS_IN_S, &meas, &r);

    /* 4. Interno en cada llamada (8 ms); externo cada OUT_EVERY llamadas. */
    control_step(&r, tick == 0, moving);
    tick++;
    if (tick >= OUT_EVERY) {
        tick = 0;
    }
}


/* ============================================================
 *        TRAYECTORIA: el robot llego a la consigna
 * ============================================================ */
bool control_arrived(void)
{
    bool ok;
    if (prof_running()) return false;    /* el perfil del ESP32 aun no termina */
    portENTER_CRITICAL(&ref_lock);
    ok = has_ref && llego[0] && llego[1] && llego[2];
    portEXIT_CRITICAL(&ref_lock);
    return ok;
}

#endif /* CONTROL_USE_RST */
