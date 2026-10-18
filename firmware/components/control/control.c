/*
 * control.c - Ver control.h
 *
 * Misma estructura que el control de la planta de pH:
 *   leer sensor -> error -> ecuacion en diferencias -> saturar -> actuador
 *   -> guardar e[k] y u[k] para la siguiente muestra.
 * La diferencia es que aqui hay 3 lazos independientes, uno por motor.
 */
#include "control.h"
#include <math.h>
#include "encoders.h"
#include "motors.h"
#include "homing_pro.h"
#include "motor_test.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if (CONTROL_TS_MS % CONTROL_LOOP_MS) != 0
#error "CONTROL_TS_MS debe ser multiplo de CONTROL_LOOP_MS"
#endif

#define NUM_JOINTS 3


/* ============================================================
 *        COEFICIENTES DE LOS CONTROLADORES  (LLENAR AQUI)
 * ============================================================
 *
 * TF de Matlab:
 *     [numD1, denD1] = tfdata(D1z, 'v')
 *
 *          b0 z^2 + b1 z + b2
 *   D(z) = ------------------      ->   NUM = {b0, b1, b2}
 *          a0 z^2 + a1 z + a2               DEN = {a0, a1, a2}
 *
 * Si el controlador es de orden 1 (como el PD), se rellena con 0 al final.
 * Ejemplo PD de pole_assignment:  D(z) = (q0 z + q1) / z
 *     numD = [q0  q1]   denD = [1  0]
 *     NUM  = {q0, q1, 0}
 *     DEN  = {1,  0,  0}
 *
 */
/* Asignacion de polos PD (NPI_Proyecto.mlx): zita = 1.2, ts = 0.32 s,
 * Ts = 0.032 s. D2z se recalculo  */
static const float CTRL_NUM[NUM_JOINTS][3] = {
    { 0.2262f, -0.1305f, 0.0f },   /* motor 1: numD1 */
    { 0.2463f, -0.1472f, 0.0f },   /* motor 2: numD2 */
    { 0.2142f, -0.1265f, 0.0f },   /* motor 3: numD3 */
};

static const float CTRL_DEN[NUM_JOINTS][3] = {
    { 1.0f, 0.0f, 0.0f },   /* motor 1: denD1 = [1 0] */
    { 1.0f, 0.0f, 0.0f },   /* motor 2: denD2 = [1 0] */
    { 1.0f, 0.0f, 0.0f },   /* motor 3: denD3 = [1 0] */
};


/* ============================================================
 *          COMPENSACION DE ZONA MUERTA  (MEDIR Y LLENAR)
 * ============================================================
 *
 * Un PD no tiene integrador: cerca del objetivo su salida es tan pequena
 * que no vence la friccion y el motor se queda corto. Ejemplo (simulado,
 * motor 1): con 1.5 V de zona muerta, una consigna de 90 grados termina en
 * 85.5 grados.
 *
 * Solucion: mientras el error sea mayor que DEAD_TOL_COUNTS, se le suma a
 * la salida V_DEAD (en el sentido de la salida). Ya en el objetivo, la
 * salida es 0 (no vibra).
 *
 * V_DEAD[i] = voltaje minimo con el que arranca el motor i.
 *   Medirlo en "Prueba de circuito": subir la velocidad hasta que el motor
 *   apenas se mueva. V_DEAD = (% / 100) * V_SUPPLY.  Ej: 15 %  ->  1.8 V
 * Con 0.0f no se compensa (como si no existiera esta parte).
 */
static const float V_DEAD[NUM_JOINTS] = { 36.0f/100.0f*12.0f, 10.0f/100.0f*12.0f, 7.0f/100.0f*12.0f };
#define DEAD_TOL_COUNTS   10      /* "llego" si esta a +-10 cuentas del objetivo */


/* ============================================================
 *              DATOS MECANICOS DE CADA ARTICULACION
 * ============================================================
 *
 * Ya no se configuran aqui (antes eran GEAR y HOME_RAD). El homingPRO
 * mide la relacion real cuentas <-> articulacion y el sentido de cada
 * encoder (components/homing_pro). Las relaciones nominales y los valores
 * de los finales (-90/+90 grados, 18/123 mm) estan en homing_pro.c.
 */

/* Radianes del EJE DEL MOTOR por cuenta del encoder (unidades de Gpos). */
#define MOTOR_RAD_PER_COUNT  (6.28318531f / (float)ENC_COUNTS_PER_REV)


/* ============================================================
 *                    SALIDA AL MOTOR
 * ============================================================
 * El controlador entrega voltios (la planta se identifico en V).
 * Se satura a +-U_MAX_V y se convierte a PWM de 10 bits (0..1023).
 */
#define V_SUPPLY   12.0f        /* voltaje de la fuente de los motores */
#define U_MAX_V    12.0f        /* saturacion de la salida del controlador */
#define PWM_MAX    1023.0f


/* ============================================================
 *                 REFERENCIA (compartida entre nucleos)
 * ============================================================
 * La escribe micro-ROS (nucleo 0) y la lee el control (nucleo 1): se
 * protege con un spinlock para no leer th1 nuevo con th2 viejo.
 */
static portMUX_TYPE ref_lock = portMUX_INITIALIZER_UNLOCKED;
static joints_t ref = { 0.0f, 0.0f, 0.0f };
static bool     has_ref = false;
static volatile uint32_t ref_seq = 0;

void control_set_reference(const joints_t *r)
{
    portENTER_CRITICAL(&ref_lock);
    ref = *r;
    has_ref = true;
    ref_seq++;
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

/* Descarta la referencia (para que el robot no salte a una consigna vieja). */
static void clear_reference(void)
{
    portENTER_CRITICAL(&ref_lock);
    has_ref = false;
    portEXIT_CRITICAL(&ref_lock);
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

/* joints_t como arreglo: 0 -> th1, 1 -> th2, 2 -> th3 */
static float joint_of(const joints_t *q, int i)
{
    return (i == 0) ? q->th1 : (i == 1) ? q->th2 : q->th3;
}


/* ============================================================
 *        ERROR: referencia - posicion, en rad del EJE DEL MOTOR
 * ============================================================
 * Unidades de la planta Gpos(s). Con la calibracion del homingPRO:
 *   referencia articular -> cuentas objetivo
 *   error [cuentas] -> rad del motor, con el signo tal que u > 0, es decir
 *   (IN1, IN2) = (1, 0), reduce el error.
 */
static float signo(float x) { return (x > 0.0f) ? 1.0f : (x < 0.0f) ? -1.0f : 0.0f; }

static float error_motor_rad(int i, float q_ref)
{
    const float q   = homing_pro_clamp_joint(i, q_ref);   /* dentro del rango medido */
    const float c_r = homing_pro_joint_to_counts(i, q);
    const float c   = (float)encoder_get_count(i);
    return (c_r - c) * (float)homing_pro_dir_sign(i) * MOTOR_RAD_PER_COUNT;
}

/* Posicion de cada articulacion: th1, th2 en rad; th3 en mm desde el piso.
 * Antes del homingPRO devuelve 0 (ver homing_pro_calibrated()). */
void control_get_position(joints_t *pos)
{
    pos->th1 = homing_pro_counts_to_joint(0, encoder_get_count(0));
    pos->th2 = homing_pro_counts_to_joint(1, encoder_get_count(1));
    pos->th3 = homing_pro_counts_to_joint(2, encoder_get_count(2));
}


/* ============================================================
 *          CONTROLADOR: ecuacion en diferencias de D(z)
 * ============================================================
 *
 * De  D(z) = U(z)/E(z) = (b0 z^2 + b1 z + b2) / (a0 z^2 + a1 z + a2)
 * multiplicando en cruz y dividiendo por z^2:
 *
 *   a0 u[k] + a1 u[k-1] + a2 u[k-2] = b0 e[k] + b1 e[k-1] + b2 e[k-2]
 *
 *   u[k] = ( b0 e[k] + b1 e[k-1] + b2 e[k-2] - a1 u[k-1] - a2 u[k-2] ) / a0
 *
 * Para el PD (q0 z + q1)/z  queda simplemente:
 *   u[k] = q0 e[k] + q1 e[k-1]
 *
 * Igual que en el control de pH, en u[k-1] se guarda el valor YA
 * saturado: si el controlador tiene integrador (PI/PID), asi no se
 * acumula error mientras el motor esta al maximo (anti-windup simple).
 */
typedef struct {
    float e1, e2;   /* e[k-1], e[k-2] */
    float u1, u2;   /* u[k-1], u[k-2] (saturados) */
} ctrl_mem_t;

static ctrl_mem_t mem[NUM_JOINTS];

static void reset_controladores(void)
{
    for (int i = 0; i < NUM_JOINTS; i++) {
        mem[i] = (ctrl_mem_t){ 0.0f, 0.0f, 0.0f, 0.0f };
    }
}

static float controlador(int i, float e)
{
    const float *b = CTRL_NUM[i];
    const float *a = CTRL_DEN[i];
    ctrl_mem_t  *m = &mem[i];

    float u = (b[0] * e + b[1] * m->e1 + b[2] * m->e2
               - a[1] * m->u1 - a[2] * m->u2) / a[0];

    u = limitar(u, -U_MAX_V, U_MAX_V);

    /* Correr la memoria una muestra */
    m->e2 = m->e1;  m->e1 = e;
    m->u2 = m->u1;  m->u1 = u;
    return u;
}


/* ============================================================
 *                   ESCRIBIR AL MOTOR
 * ============================================================
 * u > 0 -> (IN1, IN2) = (1, 0)   (mismo sentido que SEEK_L del homing)
 * u < 0 -> (IN1, IN2) = (0, 1)
 * La magnitud |u| / V_SUPPLY se vuelve duty de 10 bits.
 */
static void escribir_actuador(int i, float u)
{
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

/* Un paso de los 3 lazos (se ejecuta cada CONTROL_TS_MS). */
static void control_step(const joints_t *r)
{
    for (int i = 0; i < NUM_JOINTS; i++) {
        float e = error_motor_rad(i, joint_of(r, i));
        float u = controlador(i, e);

        /* 1. Zona muerta: si aun no llega, empujar al menos V_DEAD;
         *    si ya llego, salida 0 para que no vibre. */
        if (fabsf(e) > (float)DEAD_TOL_COUNTS * MOTOR_RAD_PER_COUNT) {
            u = limitar(u + signo(u) * V_DEAD[i], -U_MAX_V, U_MAX_V);
        } else {
            u = 0.0f;
        }

        /* 2. Finales de carrera: si el motor esta tocando el final hacia
         *    el que lo mandan, se para (alejarse del final si se permite).
         *    u > 0 -> (1,0) -> hacia el final L;  u < 0 -> hacia el final R */
        if ((u > 0.0f && limit_l_pressed(i)) || (u < 0.0f && limit_r_pressed(i))) {
            u = 0.0f;
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
    static bool     homed_before = false;   /* para detectar el fin del homing */
    static bool     active = false;          /* el lazo esta mandando los motores */
    static uint32_t tick = 0;

    const bool homed = (homing_pro_get_state() == HP_DONE);

    /* 1. Al terminar el homing: memoria limpia y se descarta cualquier
     *    consigna que llegara antes. (Los encoders NO se ponen en cero:
     *    la calibracion del homingPRO depende de sus cuentas.) */
    if (homed && !homed_before) {
        reset_controladores();
        clear_reference();
        tick = 0;
    }
    homed_before = homed;

    /* 2. Solo se controla con homing hecho, con consigna y sin una
     *    prueba de circuito corriendo (si no, se pelearian los motores). */
    joints_t r;
    if (!homed || !control_get_reference(&r) || any_motor_test_running()) {
        if (active) {
            /* Se deja de controlar: soltar motores y olvidar la memoria */
            if (homed) motors_stop_all();
            reset_controladores();
            active = false;
        }
        tick = 0;
        return;
    }
    active = true;

    /* 3. El controlador corre cada CONTROL_TS_MS (el Ts del diseno),
     *    aunque esta funcion se llame cada CONTROL_LOOP_MS. */
    if (tick == 0) {
        control_step(&r);
    }
    tick++;
    if (tick >= (CONTROL_TS_MS / CONTROL_LOOP_MS)) {
        tick = 0;
    }
}
