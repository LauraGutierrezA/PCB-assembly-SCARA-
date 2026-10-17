/*
 * homing_pro.h - Homing con doble toque y calibracion por encoder.
 *
 * Es el homing que usa la GUI (boton Start/Resume/Stop). El homing viejo
 * (components/homing) queda en el repositorio pero no se usa.
 *
 * Al iniciar (Start) se ponen los encoders en cero y se borra la
 * calibracion. Luego, motor por motor:
 *
 *   "Doble toque" = ir hasta el final, devolverse un poco (BACKOFF) y volver
 *   a tocar. El valor se marca en el SEGUNDO toque.
 *
 *   M1: doble toque en L  -> marca -90 grados
 *       doble toque en R  -> marca +90 grados
 *       con esos dos puntos calcula la relacion real cuentas/grado
 *       y se va a 0 grados.
 *   M2: doble toque en L  -> marca -90 grados
 *       usa la relacion nominal (GEAR_M2) y se va a 0 grados.
 *   M3: doble toque arriba (UP, final L)  -> marca 123 mm
 *       doble toque abajo  (DOWN, final R) -> marca  18 mm
 *       con esos dos puntos calcula mm/cuenta y se estaciona en la mitad.
 *
 * Unidades de cada articulacion ("joint"):
 *   M1, M2 -> radianes     M3 -> mm desde el piso
 *
 * Convencion de direccion (igual que el resto del firmware):
 *   (IN1, IN2) = (1, 0) va hacia el final L (M3: hacia arriba).
 *
 * Uso entre tareas (seguro):
 *   - start/resume/abort solo levantan banderas (nucleo 0, micro-ROS).
 *   - homing_pro_update() hace todo el trabajo (nucleo 1, tarea de control).
 */
#ifndef HOMING_PRO_H
#define HOMING_PRO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    HP_IDLE = 0,    /* nunca se ha corrido */
    HP_RUNNING,
    HP_DONE,        /* calibrado y estacionado */
    HP_ABORTED,     /* detenido con Stop; se puede reanudar (Resume) */
    HP_ERROR,       /* fallo de seguridad; hay que volver a dar Start */
} hp_state_t;

void homing_pro_start(void);
void homing_pro_resume(void);
void homing_pro_abort(void);
void homing_pro_update(void);

hp_state_t homing_pro_get_state(void);

/* Contador que aumenta cada vez que cambia el estado (para publicar). */
uint32_t homing_pro_status_seq(void);

/* "IDLE", "HOMING M1 SEEK_L", "ABORTED M2 BACKOFF_L",
 * "ERROR M3 NO_ENCODER", "HOMED" */
void homing_pro_status_str(char *buf, size_t len);

/* ---------- Calibracion (validas solo si homing_pro_calibrated(i)) ------- */

/* true si la articulacion i (0..2) ya tiene calibracion. */
bool homing_pro_calibrated(int i);

/* Cuentas del encoder -> articulacion (rad para M1/M2, mm para M3). */
float homing_pro_counts_to_joint(int i, int32_t counts);

/* Articulacion -> cuentas del encoder (con decimales). */
float homing_pro_joint_to_counts(int i, float q);

/* +1 o -1: hacia donde cambian las cuentas al mover con (1,0). Medido en el
 * homing, asi no hay que configurar el signo de cada encoder a mano. */
int homing_pro_dir_sign(int i);

/* Limita q al rango que se midio en el homing:
 *   M1, M2: [-90, +90] grados (en rad)   M3: [18, 123] mm */
float homing_pro_clamp_joint(int i, float q);

#endif /* HOMING_PRO_H */
