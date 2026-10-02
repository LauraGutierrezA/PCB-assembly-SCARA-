/*
 * ik.c - Ver ik.h. Portado de IK_RRP.m.
 */
#include "ik.h"
#include <math.h>

#define IK_2PI  6.28318530717959f
#define IK_TOL  1e-5f

void rrp_fk(const rrp_params_t *p, const joints_t *q, point_t *out)
{
    const float t12 = q->th1 + q->th2;
    out->x = p->d1 * cosf(q->th1) + p->d2 * cosf(t12);
    out->y = p->d1 * sinf(q->th1) + p->d2 * sinf(t12);
    out->z = p->d3 + p->d4 - p->d5 - p->d6 - p->av * q->th3 / IK_2PI;
}

ik_status_t rrp_ik(const rrp_params_t *p, const point_t *t, int codo, joints_t *out)
{
    if (p->d1 <= 0.0f || p->d2 <= 0.0f || fabsf(p->av) < IK_TOL) {
        return IK_BAD_PARAMS;
    }

    /* th2: ley del coseno */
    float c2 = (t->x * t->x + t->y * t->y - p->d1 * p->d1 - p->d2 * p->d2)
               / (2.0f * p->d1 * p->d2);
    if (c2 > 1.0f + IK_TOL || c2 < -1.0f - IK_TOL) {
        return IK_OUT_OF_REACH;
    }
    if (c2 >  1.0f) c2 =  1.0f;      /* recorte numerico en el borde */
    if (c2 < -1.0f) c2 = -1.0f;

    const float s2 = (codo >= 0 ? 1.0f : -1.0f) * sqrtf(1.0f - c2 * c2);
    out->th2 = atan2f(s2, c2);

    /* th1: equivalente a resolver A*[cos th1; sin th1] = [x; y] del .m */
    out->th1 = atan2f(t->y, t->x) - atan2f(p->d2 * s2, p->d1 + p->d2 * c2);
    if (out->th1 >  3.14159265f) out->th1 -= IK_2PI;
    if (out->th1 < -3.14159265f) out->th1 += IK_2PI;

    /* th3: tornillo */
    out->th3 = IK_2PI / p->av * (p->d3 + p->d4 - p->d5 - p->d6 - t->z);

    return IK_OK;
}

const char *ik_status_name(ik_status_t s)
{
    switch (s) {
    case IK_OK:           return "OK";
    case IK_OUT_OF_REACH: return "OUT_OF_REACH";
    case IK_BAD_PARAMS:   return "BAD_PARAMS";
    default:              return "UNKNOWN";
    }
}
