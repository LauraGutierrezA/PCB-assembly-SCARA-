"""
conversions.py - Logica pura del puente (sin rclpy, se puede probar con pytest).

/scara/limits (std_msgs/String) trae el estado medido por los encoders:
    "L1=0 R1=1 ... M1=0 M2=1 M3=0 E1=1234 E2=-56 E3=0 Q1=12.3 Q2=-0.5 Q3=70.5 CAL=1"
      Q1, Q2 = angulo de las articulaciones [grados]
      Q3     = altura de la punta sobre el piso [mm]
      CAL    = 1 si el homingPRO ya calibro (si no, Q no vale)

El URDF (scara_description) usa radianes y metros. La articulacion prismatica
joint3 vale 0 cuando la punta esta a Z_TOP = 202 mm del piso:
    joint3 [m] = (Q3 [mm] - 202) / 1000
"""
import math
from typing import Dict, Optional, Tuple

JOINT_NAMES = ['joint1', 'joint2', 'joint3']

# Deben coincidir con firmware/components/homing_pro/homing_pro.c, con
# scara_app/robot_model.py (JOINT_LIMITS, JOINT_HOME) y con el URDF (z_min, z_max).
TH1_LIM_DEG = (-85.0, 85.0)
TH2_LIM_DEG = (-90.0, 90.0)
Z_LIM_MM = (0.0, 109.2)        # M3_BOTTOM_MM / M3_TOP_MM (homing_pro.c) = JOINT_LIMITS['z']
Z_TOP_MM = 202.0                 # D3_BASE + D4_LINK - D5_LINK (robot_model.py)
Z_HOME_MM = 54.6                 # JOINT_HOME['z'] (robot_model.py): estacionamiento tras el homing

Joints = Tuple[float, float, float]   # (joint1 [rad], joint2 [rad], joint3 [m])


def parse_limits(text: str) -> Dict[str, object]:
    """Convierte el texto de /scara/limits en un diccionario.

    E* -> int (cuentas), Q* -> float, el resto (L, R, M, CAL, VAC, ARR) -> bool.
    Los campos con formato malo se ignoran.
    """
    out: Dict[str, object] = {}
    for token in text.split():
        if '=' not in token:
            continue
        key, value = token.split('=', 1)
        value = value.strip()
        try:
            if key.startswith('E'):
                out[key] = int(value)
            elif key.startswith('Q'):
                out[key] = float(value)
            else:
                out[key] = value == '1'
        except ValueError:
            pass
    return out


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def to_joints(th1_deg: float, th2_deg: float, z_mm: float) -> Joints:
    """(th1 [grados], th2 [grados], z [mm sobre el piso]) -> (rad, rad, m) del URDF.

    Satura a los limites del robot para que el modelo nunca se salga de rango.
    """
    return (
        math.radians(_clamp(th1_deg, *TH1_LIM_DEG)),
        math.radians(_clamp(th2_deg, *TH2_LIM_DEG)),
        (_clamp(z_mm, *Z_LIM_MM) - Z_TOP_MM) / 1000.0,
    )


def home_joints() -> Joints:
    """Posicion de arranque del gemelo (antes de recibir datos validos)."""
    return to_joints(0.0, 0.0, Z_HOME_MM)


def limits_to_joints(limits: Dict[str, object], require_cal: bool = True) -> Optional[Joints]:
    """Diccionario de parse_limits -> joints, o None si el dato no sirve.

    None cuando falta Q1/Q2/Q3, o cuando CAL=0 (si require_cal). Si el firmware
    no manda CAL (version vieja), se asume calibrado.
    """
    if require_cal and not limits.get('CAL', True):
        return None
    try:
        th1 = float(limits['Q1'])
        th2 = float(limits['Q2'])
        z = float(limits['Q3'])
    except (KeyError, TypeError, ValueError):
        return None
    if not all(math.isfinite(v) for v in (th1, th2, z)):
        return None
    return to_joints(th1, th2, z)


def joints_array_to_joints(data, require_cal: bool = True) -> Optional[Joints]:
    """/scara/joints (Float32MultiArray) -> joints del URDF, o None si no sirve.

    El firmware (fase 4) publica [Q1, Q2, Q3, Q4, CAL] con las mismas unidades que
    /scara/limits (grados, grados, mm sobre el piso, grados, 0/1). Q4 (servo) no
    esta en el modelo y se ignora. Se necesitan los 5 valores; con menos se descarta.
    """
    if data is None or len(data) < 5:
        return None
    return limits_to_joints(
        {'Q1': data[0], 'Q2': data[1], 'Q3': data[2], 'CAL': data[4] >= 0.5},
        require_cal)


def smooth_step(current: Joints, target: Joints, alpha: float) -> Joints:
    """Un paso de filtro de primer orden hacia el objetivo (alpha = 1: sin filtro).

    /scara/limits llega a ~10 Hz; el puente publica a 30 Hz y suaviza asi el
    movimiento del gemelo.
    """
    a = _clamp(alpha, 0.0, 1.0)
    return tuple(c + a * (t - c) for c, t in zip(current, target))  # type: ignore[return-value]
