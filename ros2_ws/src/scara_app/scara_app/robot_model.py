"""
robot_model.py - Modelo del robot que comparten las dos interfaces (PyQt5 y web).

Sin Qt ni NiceGUI: solo matematicas y constantes.

Cinematica directa (igual a ik.h del firmware y al HTML "Espacio de Trabajo"):
    x = d1*cos(th1) + d2*cos(th1+th2)
    y = d1*sin(th1) + d2*sin(th1+th2)
    z = altura de la punta sobre el piso de la base (la mide el homingPRO)

Pestana DK: los sliders dan (th1, th2, z). Se calcula (x, y) con la
cinematica directa y se manda como consigna normal a /scara/target con
codo = signo(th2). La IK del firmware devuelve exactamente los mismos
angulos (verificado: diferencia < 0.001 grados), asi que no hace falta un
topico nuevo y se reutilizan todos los chequeos de rango del firmware.
"""
import math

# Rango de cada articulacion para los sliders de DK.
# DEBEN coincidir con firmware/components/homing_pro/homing_pro.c:
#   th1: M1_L_DEG .. M1_R_DEG     th2: M2_L_DEG .. M2_MAX_DEG
#   z:   M3_BOTTOM_MM .. M3_TOP_MM
# Si cambias los finales alla (recalibracion), cambialos tambien aqui.
JOINT_LIMITS = {
    'th1': (-72.0, 72.0),    # grados (M1_L_DEG, M1_R_DEG)
    'th2': (-90.0, 90.0),    # grados
    'z':   (19.3, 126.0),    # mm sobre el piso de la base
}
JOINT_HOME = {'th1': 0.0, 'th2': 0.0, 'z': 72.6}   # donde se estaciona tras el homing

# Dead-man switch: la interfaz manda "dm1" por /scara/cmd con este periodo
# mientras el operador lo mantiene; el firmware detiene los motores si pasan
# 300 ms (DEADMAN_TIMEOUT_MS en control.h) sin latido.
DEADMAN_PERIOD_S = 0.1


def fk_xy(th1_deg: float, th2_deg: float, d1: float, d2: float):
    """(th1, th2) en grados -> (x, y) en mm."""
    a = math.radians(th1_deg)
    b = math.radians(th1_deg + th2_deg)
    return d1 * math.cos(a) + d2 * math.cos(b), d1 * math.sin(a) + d2 * math.sin(b)


def codo_for(th2_deg: float) -> int:
    """Codo que hace que la IK del firmware devuelva este th2 (signo de sin th2)."""
    return 1 if th2_deg >= 0 else -1


# ======================================================================
#                     GEMELO (vista del robot)
# ======================================================================
# Geometria fisica (igual al HTML "Espacio de Trabajo SCARA"), en mm.
# d1, d2 salen de los parametros del robot; estas son solo para dibujar.
D3_BASE = 172.0          # piso -> cara inferior del eslabon 1
D4_LINK = 60.0           # inferior eslabon 1 -> superior eslabon 2
D5_LINK = 30.0           # espesor eslabon 2
Z_TOP = D3_BASE + D4_LINK - D5_LINK     # 202: altura de la punta con d6 = 0

# Vista en planta (mm): x0 hacia arriba, y0 hacia la izquierda (igual al HTML)
PLAN_VIEW = {'xmin': -90.0, 'xmax': 240.0, 'ymin': -240.0, 'ymax': 240.0}
SINGULAR_DEG = 5.0       # |th2| menor que esto: cerca de singularidad


def fk_full(th1_deg, th2_deg, d1, d2):
    """-> (codo_x, codo_y, punta_x, punta_y) en mm."""
    a = math.radians(th1_deg)
    ex, ey = d1 * math.cos(a), d1 * math.sin(a)
    x, y = fk_xy(th1_deg, th2_deg, d1, d2)
    return ex, ey, x, y


def ik_xy(x, y, codo, d1, d2):
    """IK en el plano, igual a rrp_ik del firmware. -> (th1, th2) en grados o None."""
    c2 = (x * x + y * y - d1 * d1 - d2 * d2) / (2 * d1 * d2)
    if abs(c2) > 1 + 1e-5:
        return None
    c2 = max(-1.0, min(1.0, c2))
    s2 = (1 if codo >= 0 else -1) * math.sqrt(1 - c2 * c2)
    th2 = math.atan2(s2, c2)
    th1 = math.atan2(y, x) - math.atan2(d2 * s2, d1 + d2 * c2)
    th1 = (th1 + math.pi) % (2 * math.pi) - math.pi
    return math.degrees(th1), math.degrees(th2)


def in_limits(q):
    """q = {'th1', 'th2', 'z'}: True si todo esta dentro de JOINT_LIMITS."""
    return all(JOINT_LIMITS[k][0] - 1e-6 <= q[k] <= JOINT_LIMITS[k][1] + 1e-6 for k in q)


def zone_rgba(px_per_mm, d1, d2, accent_rgb, warn_rgb):
    """Imagen RGBA (numpy uint8, alto x ancho) de la zona alcanzable en planta,
    con los limites de JOINT_LIMITS. Fila 0 = xmax (arriba), columna 0 = ymax (izq.)."""
    import numpy as np
    v = PLAN_VIEW
    w = int(round((v['ymax'] - v['ymin']) * px_per_mm))
    h = int(round((v['xmax'] - v['xmin']) * px_per_mm))
    ys = v['ymax'] - (np.arange(w) + 0.5) / px_per_mm
    xs = v['xmax'] - (np.arange(h) + 0.5) / px_per_mm
    Y, X = np.meshgrid(ys, xs)
    c2 = (X ** 2 + Y ** 2 - d1 ** 2 - d2 ** 2) / (2 * d1 * d2)
    reach = np.abs(c2) <= 1
    c2 = np.clip(c2, -1, 1)
    best = np.full(X.shape, -1.0)
    (t1lo, t1hi), (t2lo, t2hi) = JOINT_LIMITS['th1'], JOINT_LIMITS['th2']
    for sg in (1, -1):
        s2 = sg * np.sqrt(1 - c2 ** 2)
        t2 = np.degrees(np.arctan2(s2, c2))
        t1 = np.degrees(np.arctan2(Y, X) - np.arctan2(d2 * s2, d1 + d2 * c2))
        t1 = (t1 + 180) % 360 - 180
        ok = reach & (t1 >= t1lo) & (t1 <= t1hi) & (t2 >= t2lo) & (t2 <= t2hi)
        best = np.where(ok, np.maximum(best, np.abs(s2)), best)
    img = np.zeros((h, w, 4), np.uint8)
    sing = (best >= 0) & (best < math.sin(math.radians(SINGULAR_DEG)))
    norm = (best >= 0) & ~sing
    img[norm, :3] = accent_rgb
    img[norm, 3] = (255 * (0.07 + 0.16 * best[norm])).astype(np.uint8)
    img[sing, :3] = warn_rgb
    img[sing, 3] = 120
    return img


def hex_rgb(color):
    color = color.lstrip('#')
    return tuple(int(color[i:i + 2], 16) for i in (0, 2, 4))
