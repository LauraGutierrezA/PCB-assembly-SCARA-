/*
 * effector.h - Efector final del SCARA: ventosa (bomba) + micro servo de giro.
 *
 *   Bomba de vacio : salida digital PIN_VACUUM (via MOSFET/rele).
 *                    1 = bomba encendida (PICK), 0 = apagada (PLACE).
 *   Servo de giro  : Tower Pro 9 g en PIN_SERVO (PWM 50 Hz, LEDC timer 1).
 *                    Angulo de la boquilla de -90 a +90 grados (0 = centro).
 *                    Arranca en 0. Los giros "rot" son RELATIVOS (+90 / -90) y
 *                    se saturan en los extremos; "servo" fija un angulo ABSOLUTO.
 *
 * Comandos por /scara/cmd (los interpreta effector_handle_cmd):
 *   "vac1"      -> bomba ON
 *   "vac0"      -> bomba OFF
 *   "rot +90"   -> gira +90 grados desde el angulo actual (tambien "rot -90")
 *   "servo 30"  -> pone el servo en 30 grados (absoluto, -90..+90; prueba de circuito)
 *   "stop"      -> apaga la bomba (y main.c sigue con su "stop" normal)
 *
 * Estado en /scara/limits: VAC=0/1 y Q4=<angulo del servo en grados>.
 */
#ifndef EFFECTOR_H
#define EFFECTOR_H

#include <stdbool.h>
#include <stddef.h>

#define SERVO_MIN_DEG   (-90)
#define SERVO_MAX_DEG   ( 90)

void effector_init(void);

void effector_set_vacuum(bool on);
bool effector_vacuum_on(void);

/* Gira delta grados desde el angulo actual (satura en -90..+90).
 * Devuelve el angulo nuevo. */
int  effector_rotate_rel(int delta_deg);

/* Pone el servo en deg grados (satura en -90..+90). Devuelve el angulo nuevo. */
int  effector_set_servo_deg(int deg);
int  effector_servo_deg(void);

/* Interpreta un comando de /scara/cmd (texto SIN '\0', longitud n).
 * Devuelve true si el comando era del efector y ya se atendio (main.c no
 * debe seguir). Para "stop" apaga la bomba y devuelve false (main.c sigue
 * deteniendo el resto). */
bool effector_handle_cmd(const char *data, size_t n);

#endif /* EFFECTOR_H */
