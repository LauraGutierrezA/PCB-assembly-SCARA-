/*
 * ik.h - Cinematica directa e inversa del SCARA RRP con tornillo en Z.
 *
 * Modelo (DH modificada, Craig), igual al de MATLAB:
 *   Fila  a_ij  alpha_ij  s_j                        theta_j
 *    1    0     0         d3                         th1
 *    2    d1    0         d4                         th2
 *    3    d2    pi        d5                         th3
 *    4    0     0         0                          -th3
 *    5    0     0         d6 + av*th3/(2*pi)         0
 *    6    0     0         0                          0
 *
 * th3 es el angulo del MOTOR del tornillo (rad): cada vuelta (2*pi) baja el
 * efector av mm. Por eso th3 puede valer varias vueltas.
 *
 * Posicion del efector:
 *   x = d1*cos(th1) + d2*cos(th1+th2)
 *   y = d1*sin(th1) + d2*sin(th1+th2)
 *   z = d3 + d4 - d5 - d6 - av*th3/(2*pi)
 *
 * Unidades: mm y radianes. Usa float (FPU de precision simple del ESP32).
 */
#ifndef IK_H
#define IK_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float d1, d2, d3, d4, d5, d6;   /* [mm] */
    float av;                       /* avance del tornillo [mm/vuelta] */
} rrp_params_t;

typedef struct { float th1, th2, th3; } joints_t;  /* [rad] */
typedef struct { float x, y, z; }       point_t;   /* [mm]  */

typedef enum {
    IK_OK = 0,
    IK_OUT_OF_REACH,   /* |cos(th2)| > 1: (x, y) fuera del anillo |d1-d2| <= r <= d1+d2 */
    IK_BAD_PARAMS,     /* d1 o d2 <= 0, o av == 0 */
} ik_status_t;

/* Cinematica directa: q -> (x, y, z) */
void rrp_fk(const rrp_params_t *p, const joints_t *q, point_t *out);

/*
 * Cinematica inversa: (x, y, z) -> q
 * codo = +1 o -1 elige la configuracion (signo de sin(th2)), como en MATLAB.
 */
ik_status_t rrp_ik(const rrp_params_t *p, const point_t *target, int codo, joints_t *out);

const char *ik_status_name(ik_status_t s);   /* "OUT_OF_REACH", ... */

#ifdef __cplusplus
}
#endif

#endif /* IK_H */
