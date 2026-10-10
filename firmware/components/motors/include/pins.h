/*
 * pins.h - Mapa de pines del SCARA (ESP32 DevKit V1, 30 pines).
 *
 * UNICA fuente de verdad para los pines: firmware y PCB deben coincidir
 * con esta tabla. Orden fisico del header (antena hacia arriba):
 *
 *            LADO IZQUIERDO                       LADO DERECHO
 *   EN                                      D23  DEADMAN   (entrada)
 *   VP  (36)  LS_L1                         D22  (libre)
 *   VN  (39)  LS_R1                         TX0  (1)  micro-ROS, NO USAR
 *   D34       LS_L2                         RX0  (3)  micro-ROS, NO USAR
 *   D35       LS_R2                         D21  ENC1_A
 *   D32       LS_L3                         D19  ENC1_B
 *   D33       LS_R3                         D18  ENC2_A
 *   D25       M1_IN1                        D5   ENC2_B
 *   D26       M1_IN2                        TX2 (17) ENC3_A
 *   D27       M2_IN1                        RX2 (16) ENC3_B
 *   D14       M2_IN2                        D4   M1_PWM
 *   D12       M3_IN1                        D2   M2_PWM
 *   D13       M3_IN2                        D15  M3_PWM
 *   GND                                     GND
 *   VIN                                     3V3
 *
 * Notas de hardware:
 *   - GPIO 34, 35, 36, 39: solo entrada y SIN pull interno. Los finales de
 *     carrera en esos pines llevan resistencia externa de 10k a GND.
 *   - GPIO 12 (M3_IN1): debe estar en BAJO al encender (si no, el ESP32 no
 *     arranca). Nada en esa linea debe subirlo.
 *   - GPIO 2 (M2_PWM): debe estar en bajo/flotante para poder flashear.
 *   - GPIO 16/17: validos solo en modulos ESP32-WROOM (no WROVER).
 *   - STBY de los TB6612 va soldado a ALTO en la PCB (no usa GPIO).
 */
#ifndef PINS_H
    #define PINS_H

    /* ---------- Finales de carrera ---------- */
    #define PIN_LS_L1     15
    #define PIN_LS_R1     2
    #define PIN_LS_L2     4
    #define PIN_LS_R2     16
    #define PIN_LS_L3     17
    #define PIN_LS_R3     5

    /* ---------- Direccion (enables) ---------- */
    #define PIN_M1_IN1    25
    #define PIN_M1_IN2    18
    #define PIN_M2_IN1    19
    #define PIN_M2_IN2    21
    #define PIN_M3_IN1    22
    #define PIN_M3_IN2    23

    /* ---------- PWM ---------- */
    #define PIN_M1_PWM    12
    #define PIN_M2_PWM    14
    #define PIN_M3_PWM    27

    /* ---------- Encoders ---------- */
    #define PIN_ENC1_A    36
    #define PIN_ENC1_B    39
    #define PIN_ENC2_A    34
    #define PIN_ENC2_B    35
    #define PIN_ENC3_A    32
    #define PIN_ENC3_B    33

    /* ---------- Seguridad ---------- */
    #define PIN_DEADMAN   26    /* el 23 ahora es M3_IN2 */

#endif /* PINS_H */
