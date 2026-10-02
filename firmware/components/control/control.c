/*
 * control.c - Ver control.h
 */
#include "control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* La referencia la escribe un nucleo y la lee el otro: se protege con un
 * spinlock para no leer, por ejemplo, th1 nuevo con th2 viejo. */
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

void control_update(void)
{
    joints_t r;
    if (!control_get_reference(&r)) {
        return;   /* todavia no hay consigna */
    }

    /*
     * TODO: algoritmo de control.
     *   1. Leer la posicion de cada articulacion (encoders).
     *   2. Error = referencia - posicion  (r.th1, r.th2, r.th3).
     *   3. PID discreto por articulacion -> duty y direccion de cada motor
     *      (motor_set_duty / motor_set_dir de components/motors).
     *   4. Solo mover si homing_get_state() == HOMING_DONE.
     */
}
