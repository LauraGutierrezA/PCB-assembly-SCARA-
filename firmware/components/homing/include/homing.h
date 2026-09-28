/*
 * homing.h - Maquina de estados del homing (misma logica de Scara_Main.c).
 *
 * Por cada motor, en orden M1 -> M2 -> M3:
 *   1. Avanza (1,0) hasta tocar el final A, frena 100 ms.
 *   2. Retrocede (0,1) hasta tocar el final B, frena 100 ms.
 *   3. Avanza (1,0) durante backoff_time_ms y se detiene (centro aproximado).
 *
 * Comandos:
 *   homing_start()  -> empieza SIEMPRE desde el motor 1, fase inicial.
 *   homing_resume() -> despues de un stop, continua en el motor y la fase
 *                      donde quedo (el retroceso conserva el tiempo que le
 *                      faltaba). Si nunca se ha corrido, equivale a start.
 *                      Si ya termino o esta corriendo, no hace nada.
 *   homing_abort()  -> detiene todos los motores y guarda el punto actual.
 *
 * Uso entre tareas (seguro):
 *   - start/resume/abort solo levantan banderas. Se pueden llamar desde el
 *     callback de micro-ROS (nucleo 0).
 *   - homing_update() hace todo el trabajo. La llama la tarea de control
 *     (nucleo 1) cada 10 ms. Es la unica que toca los motores.
 */
#ifndef HOMING_H
#define HOMING_H

#include <stdint.h>
#include <stddef.h>

typedef enum {
    HOMING_IDLE = 0,    /* nunca se ha corrido */
    HOMING_RUNNING,
    HOMING_DONE,
    HOMING_ABORTED,     /* detenido a mitad; se puede reanudar */
} homing_state_t;

void homing_start(void);
void homing_resume(void);
void homing_abort(void);
void homing_update(void);

homing_state_t homing_get_state(void);

/* Contador que aumenta cada vez que cambia el estado. Sirve para publicar
 * /scara/status solo cuando hay algo nuevo. */
uint32_t homing_status_seq(void);

/* Texto legible del estado, p. ej. "HOMING M2 SEEK_B", "ABORTED M2 SEEK_B"
 * o "HOMED". */
void homing_status_str(char *buf, size_t len);

#endif /* HOMING_H */
