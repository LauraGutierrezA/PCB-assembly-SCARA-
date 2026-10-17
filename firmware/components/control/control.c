/*
 * control.c - Ver control.h
 *
 * Misma estructura que el control de la planta de pH:
 *   leer sensor -> error -> ecuacion en diferencias -> saturar -> actuador
 *   -> guardar e[k] y u[k] para la siguiente muestra.
 * La diferencia es que aqui hay 3 lazos independientes, uno por motor.
 */
#include "control.h"
#include "encoders.h"
#include "motors.h"
#include "homing.h"
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
 * Pegar lo que da MATLAB para cada motor:
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
 * Con todo en 0 el controlador da u = 0 (el motor no se mueve): es seguro
 * dejarlo asi hasta pegar los valores reales.
 */
static const float CTRL_NUM[NUM_JOINTS][3] = {
    { 0.0f, 0.0f, 0.0f },   /* motor 1: numD1 */
    { 0.0f, 0.0f, 0.0f },   /* motor 2: numD2 */
    { 0.0f, 0.0f, 0.0f },   /* motor 3: numD3 */
};

static const float CTRL_DEN[NUM_JOINTS][3] = {
    { 1.0f, 0.0f, 0.0f },   /* motor 1: denD1 */
    { 1.0f, 0.0f, 0.0f },   /* motor 2: denD2 */
    { 1.0f, 0.0f, 0.0f },   /* motor 3: denD3 */
};


/* ============================================================
 *              DATOS MECANICOS DE CADA ARTICULACION
 * ============================================================
 *
 * GEAR: vueltas del eje del motor por cada vuelta de la articulacion.
 *   La reductora es 50:1. Si una articulacion tiene ademas poleas o
 *   engranes, multiplicar (p. ej. 50 * 2.0).
 *   Para th3 la IK entrega el angulo del eje del tornillo, asi que GEAR
 *   es la relacion entre el motor y el tornillo.
 *
 * HOME_RAD: angulo de la articulacion [rad] (en el mismo sistema de la IK)
 *   donde queda el robot al terminar el homing. Ahi se ponen los encoders
 *   en cero. Medirlo en el robot real.
 */
static const float GEAR[NUM_JOINTS]     = { 50.0f, 50.0f, 50.0f };
static const float HOME_RAD[NUM_JOINTS] = { 0.0f,  0.0f,  0.0f };   /* TODO: medir */


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
 *          LEER SENSOR: posicion del eje del motor [rad]
 * ============================================================
 * Se mide relativo al punto de homing, en radianes del EJE DEL MOTOR,
 * que son las unidades de la planta Gpos(s).
 */
static float leer_motor_rad(int i)
{
    return encoder_get_motor_rad(i);
}

/* Referencia articular [rad] -> referencia en radianes del eje del motor,
 * medida desde el punto de homing. */
static float ref_a_motor_rad(int i, float q_ref)
{
    return (q_ref - HOME_RAD[i]) * GEAR[i];
}

void control_get_position(joints_t *pos)
{
    float q[NUM_JOINTS];
    for (int i = 0; i < NUM_JOINTS; i++) {
        q[i] = leer_motor_rad(i) / GEAR[i] + HOME_RAD[i];
    }
    pos->th1 = q[0];
    pos->th2 = q[1];
    pos->th3 = q[2];
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
        float y   = leer_motor_rad(i);
        float ref_m = ref_a_motor_rad(i, joint_of(r, i));
        float e   = ref_m - y;
        float u   = controlador(i, e);
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

    const bool homed = (homing_get_state() == HOMING_DONE);

    /* 1. Al terminar el homing: encoders en cero, memoria limpia y se
     *    descarta cualquier consigna que llegara antes. */
    if (homed && !homed_before) {
        encoders_reset();
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
