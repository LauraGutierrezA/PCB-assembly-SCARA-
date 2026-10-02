/*
 * control.h - Control de posicion de las articulaciones.
 *
 * Por ahora SOLO guarda las referencias que calcula la IK. El algoritmo de
 * control (PID con encoders) va en control_update() cuando esten los
 * encoders y los requisitos de Control Digital.
 *
 * Uso entre tareas (seguro):
 *   - control_set_reference() se llama desde micro-ROS (nucleo 0).
 *   - control_update() la llama la tarea de control (nucleo 1) cada 10 ms.
 */
#ifndef CONTROL_H
#define CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include "ik.h"

/* Nueva referencia articular [rad] (th3 = angulo del motor del tornillo). */
void control_set_reference(const joints_t *ref);

/* Copia la referencia actual. Devuelve false si nunca se ha recibido una. */
bool control_get_reference(joints_t *ref);

/* Contador de referencias recibidas (cambia con cada consigna nueva). */
uint32_t control_reference_seq(void);

/* Un paso del lazo de control. Por ahora no hace nada. */
void control_update(void);

#endif /* CONTROL_H */
