"""
trajectory.py - Planeacion del ciclo de la pestana "Trayectoria" (sin ROS ni NiceGUI).

Un PROGRAMA es lo que el operador arma en el sandbox:
    {'blocks': [{'id': 'b3', 'kind': 'goto', 'params': {...}}, ...],
     'links':  [{'from': 'b3', 'to': 'b4'}, ...]}

Tipos de bloque (parametros):
    goto   -> x, y, z [mm], codo (+1/-1), safe (subir/bajar en vertical con despeje)
    pick   -> wait [s]       bomba ON  y espera (que agarre)
    place  -> wait [s]       bomba OFF y espera (que suelte)
    rotate -> deg [grados, entero, con signo], wait [s]   giro RELATIVO al angulo actual
              del servo de la boquilla (rango fisico del servo: SERVO_LIMITS)

Flujo (igual que la guia de trayectorias, secciones 3 y 5):
  1. order_blocks(): sigue las flechas desde el bloque INICIO (el unico sin
     flecha de entrada) -> lista ordenada. Detecta ramas, ciclos y bloques
     sueltos.
  2. plan_cycle(): para cada GO TO POSE resuelve la IK (la misma del
     firmware, robot_model.ik_xy, con el codo del bloque) y revisa los
     limites articulares ANTES de mover el robot. Luego arma los tramos:
        - con "despeje Z": sube vertical -> viaja en XY -> baja vertical
          (aproximacion vertical en Z que pide la Estacion 3). La altura de
          viaje es  max(z inicial, z final) + despeje  (pocos mm: Z es lento).
        - sin despeje: un solo tramo con th1, th2 y z sincronizados
     Cada tramo es reposo-reposo en espacio ARTICULAR (th1, th2, z), y como
     los limites articulares son una "caja", si los extremos estan dentro
     todo el tramo tambien lo esta (sin singularidades ni flips).
     Duracion del tramo = la mayor de los 3 ejes:
        th1, th2 (quintico):    max(1.875*L/v_max, sqrt(5.77*L/a_max))
        z (trapezoide f = 0.2): max(1.25*L/v_max,  sqrt(6.25*L/a_max))
  3. La pestana manda cada tramo como UNA meta a /scara/target (13 valores:
     ... codo, T, id) y el ESP32 interpola cada 8 ms (profile.c) con
     feedforward de velocidad. Los puntos INTERMEDIOS (arriba del inicio y
     arriba de la meta) no se esperan: cuando el tramo va en BLEND_PCT %
     se manda el siguiente y el ESP32 los superpone (sin frenar). Solo la
     meta final de cada GO TO POSE se espera con ARR=1 (llegada estricta,
     DEAD_TOL_COUNTS del firmware).
"""
import math
from dataclasses import dataclass, field

from scara_app.robot_model import JOINT_LIMITS, codo_for, fk_xy, ik_xy, in_limits

# ======================================================================
#   LIMITES DEL PERFIL  (AJUSTAR con el robot real)
# ======================================================================
# Con el motor de 200 RPM (W_MAX de controlRST.c) y las reducciones del
# firmware (homing_pro.c): M1 50*4 -> 300 grados/s, M2 50*3 -> 400 grados/s,
# M3 50*3 con tornillo de 2 mm/vuelta -> ~2.2 mm/s. Se deja un margen para
# que el control alcance a seguir la referencia.
V_MAX_DEG = 45.0       # grados/s   (th1, th2)
A_MAX_DEG = 90.0       # grados/s^2
V_MAX_Z = 4.0          # mm/s   (pico del trapezoide) - ver TUNING_RANGES
A_MAX_Z = 15.0         # mm/s^2
MIN_SEGMENT_S = 0.3    # duracion minima de un tramo
TRAP_FRAC = 0.2        # = PROF_TRAP_FRAC de firmware/components/control/include/profile.h

Z_CLEAR_DEFAULT = 5.0  # despeje [mm] sobre la mas alta de las dos z (inicio / meta)
Z_CLEAR_MAX = 60.0
BLEND_PCT = 70         # % del tramo intermedio en que se manda el siguiente

# ----------------------------------------------------------------------
#   AJUSTES DESDE LA PESTANA (casillas de la barra) Y SUS RANGOS
# ----------------------------------------------------------------------
# Velocidad maxima de la SALIDA de la caja de los motores: ~230 RPM.
# Con las bandas: M1 (x4) -> 230/4 RPM = 345 grados/s, M2 (x3) -> 460 grados/s.
# th1 y th2 comparten V_MAX_DEG, asi que manda el mas lento (M1): 345 grados/s.
# Se deja 20 % de margen para que el control pueda corregir -> tope 275.
# (Ojo: controlRST.c satura la velocidad en W_MAX_RPM_SALIDA = 200 RPM,
#  o sea 300 grados/s en M1; 275 queda por debajo.)
OUTPUT_RPM_MAX = 230.0
M1_BELT, M2_BELT = 4.0, 3.0
JOINT_DEG_S_MAX = min(OUTPUT_RPM_MAX / M1_BELT, OUTPUT_RPM_MAX / M2_BELT) * 6.0   # 345
SPEED_MARGIN = 0.8

# nombre: (minimo, maximo, por defecto)
TUNING_RANGES = {
    'V_MAX_DEG':     (5.0, round(SPEED_MARGIN * JOINT_DEG_S_MAX / 5) * 5, 90.0),   # 5..275
    'A_MAX_DEG':     (10.0, 1000.0, 200.0),
    # Z MEDIDO (Prueba de circuito, 100 % PWM, 1.5 s, corregido a la carrera
    # real de 109.2 mm): 9.82 mm/s subiendo y 9.69 mm/s bajando -> el mas lento,
    # 9.69 mm/s; 80 % = 7.75 -> tope 7.5.
    # (El tornillo avanza ~4x lo que suponia AV_NOMINAL_MM = 2 mm/vuelta.)
    'V_MAX_Z':       (0.1, 7.5, V_MAX_Z),
    'A_MAX_Z':       (0.5, 50.0, A_MAX_Z),
    'FF_GAIN':       (0.0, 1.2, 1.0),      # = FF_GAIN_MIN / FF_GAIN_MAX de control.h
    'BLEND_PCT':     (30.0, 100.0, float(BLEND_PCT)),
    'MIN_SEGMENT_S': (0.05, 2.0, MIN_SEGMENT_S),   # 0.05 = PROF_MIN_T de profile.h
}


# Cuentas del encoder por unidad de cada articulacion (nominales, homing_pro.c):
# 64 cuentas/vuelta del motor x reduccion. Solo para mostrar el error de llegada.
COUNTS_PER_UNIT = {1: 64 * 200 / 360.0,    # M1: cuentas por grado (50 x 4)
                   2: 64 * 150 / 360.0,    # M2: cuentas por grado (50 x 3)
                   3: 64 * 150 / 8.0}      # M3: cuentas por mm (50 x 3, ~8 mm/vuelta, aprox.
                                           #     segun la velocidad medida de ~9.7 mm/s)
UNIT = {1: '°', 2: '°', 3: 'mm'}


def describe_error(motor, counts):
    """'M3: 41 cuentas (0.009 mm)'"""
    v = counts / COUNTS_PER_UNIT[motor]
    fmt = '.3f' if motor == 3 else '.2f'
    return f'M{motor}: {counts:+d} cuentas ({v:{fmt}} {UNIT[motor]})'


@dataclass
class Tuning:
    v_max_deg: float = TUNING_RANGES['V_MAX_DEG'][2]
    a_max_deg: float = TUNING_RANGES['A_MAX_DEG'][2]
    v_max_z: float = TUNING_RANGES['V_MAX_Z'][2]
    a_max_z: float = TUNING_RANGES['A_MAX_Z'][2]
    ff_gain: float = TUNING_RANGES['FF_GAIN'][2]
    blend_pct: float = TUNING_RANGES['BLEND_PCT'][2]
    min_segment_s: float = TUNING_RANGES['MIN_SEGMENT_S'][2]

    def check(self):
        """Lanza PlanError si algun valor esta fuera de su rango."""
        for name, value in (('V_MAX_DEG', self.v_max_deg), ('A_MAX_DEG', self.a_max_deg),
                            ('V_MAX_Z', self.v_max_z), ('A_MAX_Z', self.a_max_z),
                            ('FF_GAIN', self.ff_gain), ('BLEND_PCT', self.blend_pct),
                            ('MIN_SEGMENT_S', self.min_segment_s)):
            lo, hi, _ = TUNING_RANGES[name]
            if value is None or not math.isfinite(value) or not lo <= value <= hi:
                raise PlanError(f'{name} = {value} fuera de [{lo:g}, {hi:g}].')
SAME_XY_DEG = 0.05     # si th1 y th2 cambian menos que esto, no hay viaje en XY

SERVO_LIMITS = (-90, 90)   # Tower Pro 9 g: 180 grados en total, 0 = centro (effector.h)
ROTATE_LIMITS = (-180, 180)   # lo que se puede escribir en un bloque ROTATE

KIND_LABEL = {'goto': 'GO TO POSE', 'pick': 'PICK', 'place': 'PLACE', 'rotate': 'ROTATE'}

DEFAULT_PARAMS = {
    'goto':   {'x': 180.0, 'y': 0.0, 'z': 54.6, 'codo': 1, 'safe': True},
    'pick':   {'wait': 0.5},
    'place':  {'wait': 0.8},
    'rotate': {'deg': 45, 'wait': 0.4},
}

JOINT_NAMES = {'th1': 'th1 (M1)', 'th2': 'th2 (M2)', 'z': 'z (M3)'}


class PlanError(Exception):
    """Error en el programa. block_id: bloque a marcar en rojo (o None)."""

    def __init__(self, message, block_id=None):
        super().__init__(message)
        self.block_id = block_id


# ======================================================================
#   Perfiles
# ======================================================================
def quintic_s(tau):
    """Perfil quintico reposo-reposo normalizado: s(0)=0, s(1)=1, s', s'' = 0 en los extremos."""
    tau = min(1.0, max(0.0, tau))
    return tau ** 3 * (10 - 15 * tau + 6 * tau * tau)


def trap_duration(dist, v_max, a_max, frac=TRAP_FRAC):
    """Tiempo minimo del trapezoide reposo-reposo (acelera y frena en frac*T):
    v_pico = L / ((1-frac) T),  a = L / (frac (1-frac) T^2)."""
    dist = abs(dist)
    if dist < 1e-9:
        return 0.0
    return max(dist / ((1 - frac) * v_max), math.sqrt(dist / (frac * (1 - frac) * a_max)))


def quintic_duration(dist, v_max, a_max):
    """Tiempo minimo de un quintico reposo-reposo de longitud dist (v_pico = 1.875 L/T,
    a_pico = 5.77 L/T^2)."""
    dist = abs(dist)
    if dist < 1e-9:
        return 0.0
    return max(1.875 * dist / v_max, math.sqrt(5.77 * dist / a_max))


@dataclass
class Pose:
    th1: float   # grados
    th2: float   # grados
    z: float     # mm desde el piso

    def lerp(self, other, s):
        return Pose(self.th1 + (other.th1 - self.th1) * s,
                    self.th2 + (other.th2 - self.th2) * s,
                    self.z + (other.z - self.z) * s)

    def same_xy(self, other):
        return (abs(self.th1 - other.th1) < SAME_XY_DEG and
                abs(self.th2 - other.th2) < SAME_XY_DEG)


def cartesian(pose, d1, d2):
    """Pose articular -> (x, y, z, codo) para /scara/target."""
    x, y = fk_xy(pose.th1, pose.th2, d1, d2)
    return x, y, pose.z, codo_for(pose.th2)


# ======================================================================
#   Pasos del ciclo (los recorre la pestana)
# ======================================================================
@dataclass
class BlockStart:
    block_id: str
    index: int          # posicion en la cadena (1..n)
    cycle: int          # repeticion (1..N)
    text: str           # para la consola


@dataclass
class Goal:
    """Un tramo: se manda como UNA meta al ESP32, que interpola (profile.c)."""
    block_id: str
    start: Pose
    end: Pose
    duration: float
    label: str          # 'subir', 'viaje XY', 'bajar', 'tramo'
    final: bool         # True: meta del GO TO POSE (se espera ARR); False: intermedio


@dataclass
class Arrive:
    block_id: str
    target: Pose


@dataclass
class Command:
    block_id: str
    action: str         # 'vac' | 'rot'
    value: int          # vac: 1/0   rot: +90/-90


@dataclass
class VacConfirm:
    """Espera a que /scara/limits confirme la bomba (VAC); si no, reenvia el comando."""
    block_id: str
    on: bool


@dataclass
class Dwell:
    block_id: str
    seconds: float


@dataclass
class Plan:
    steps: list = field(default_factory=list)
    warnings: list = field(default_factory=list)
    order: list = field(default_factory=list)     # bloques en orden (una repeticion)
    duration: float = 0.0                         # estimado sin esperas de llegada


# ======================================================================
#   1. Orden de los bloques
# ======================================================================
def order_blocks(program):
    """-> (lista de bloques en orden de ejecucion, avisos). Lanza PlanError."""
    blocks = {b['id']: b for b in (program or {}).get('blocks', [])}
    if not blocks:
        raise PlanError('El sandbox esta vacio: arrastra bloques desde arriba.')
    nxt, has_in = {}, set()
    for link in program.get('links', []):
        a, b = link.get('from'), link.get('to')
        if a not in blocks or b not in blocks:
            continue
        if a in nxt:
            raise PlanError('Un bloque tiene dos flechas de salida.', a)
        if b in has_in:
            raise PlanError('Un bloque tiene dos flechas de entrada.', b)
        nxt[a] = b
        has_in.add(b)

    connected = set(nxt) | has_in
    if not connected:
        if len(blocks) == 1:
            return list(blocks.values()), []
        raise PlanError('Conecta los bloques: arrastra desde el punto de salida (derecha) '
                        'de un bloque hasta el siguiente.')
    heads = [bid for bid in blocks if bid in nxt and bid not in has_in]
    if not heads:
        raise PlanError('Las flechas forman un ciclo cerrado: deja un bloque INICIO sin '
                        'flecha de entrada (para repetir usa "Repeticiones").')
    if len(heads) > 1:
        raise PlanError(f'Hay {len(heads)} cadenas separadas: une todo en una sola.', heads[1])

    order, seen, cur = [], set(), heads[0]
    while cur is not None:
        if cur in seen:
            raise PlanError('Las flechas forman un ciclo: usa "Repeticiones" para repetir.', cur)
        seen.add(cur)
        order.append(blocks[cur])
        cur = nxt.get(cur)
    if connected - seen:
        raise PlanError('Hay flechas que no salen de la cadena principal.',
                        sorted(connected - seen)[0])
    loose = [bid for bid in blocks if bid not in seen]
    warnings = []
    if loose:
        warnings.append(f'{len(loose)} bloque(s) sin conectar: no se ejecutan.')
    return order, warnings


# ======================================================================
#   2. Plan
# ======================================================================
def _num(params, key, block_id, label):
    try:
        v = float(params.get(key))
    except (TypeError, ValueError):
        raise PlanError(f'{label}: campo "{key}" vacio o invalido.', block_id) from None
    if not math.isfinite(v):
        raise PlanError(f'{label}: campo "{key}" invalido.', block_id)
    return v


def goto_pose(block, d1, d2):
    """Pose articular de un GO TO POSE (misma IK que el firmware). Lanza PlanError."""
    p, bid = block.get('params', {}), block['id']
    x, y, z = (_num(p, k, bid, 'GO TO POSE') for k in ('x', 'y', 'z'))
    codo = 1 if int(p.get('codo', 1)) >= 0 else -1
    where = f'({x:.1f}, {y:.1f}, {z:.1f})'
    q = ik_xy(x, y, codo, d1, d2)
    if q is None:
        raise PlanError(f'GO TO POSE {where}: fuera del alcance en XY.', bid)
    pose = Pose(q[0], q[1], z)
    for k, v in (('th1', pose.th1), ('th2', pose.th2), ('z', pose.z)):
        lo, hi = JOINT_LIMITS[k]
        if not lo - 1e-6 <= v <= hi + 1e-6:
            hint = '' if k == 'z' else ' (prueba con el otro codo)'
            raise PlanError(f'GO TO POSE {where}: {JOINT_NAMES[k]} = {v:.1f} fuera de '
                            f'[{lo:g}, {hi:g}]{hint}.', bid)
    return pose


def _segment(block_id, a, b, label, tun):
    T = max(quintic_duration(b.th1 - a.th1, tun.v_max_deg, tun.a_max_deg),
            quintic_duration(b.th2 - a.th2, tun.v_max_deg, tun.a_max_deg),
            trap_duration(b.z - a.z, tun.v_max_z, tun.a_max_z))
    if T == 0.0:
        return None
    return Goal(block_id, a, b, max(T, tun.min_segment_s), label, False)


def goto_moves(block_id, start, end, safe, z_clear, tun=None):
    """Tramos de start a end. Con safe: subir -> viaje XY -> bajar, a la altura
    max(start.z, end.z) + z_clear. Solo el ultimo es la meta (final=True)."""
    tun = tun or Tuning()
    if not safe or start.same_xy(end):
        segs = [_segment(block_id, start, end, 'tramo', tun)]
    else:
        z_travel = min(JOINT_LIMITS['z'][1], max(start.z, end.z) + z_clear)
        up = Pose(start.th1, start.th2, z_travel)
        over = Pose(end.th1, end.th2, z_travel)
        segs = [_segment(block_id, start, up, 'subir', tun),
                _segment(block_id, up, over, 'viaje XY', tun),
                _segment(block_id, over, end, 'bajar', tun)]
    segs = [s for s in segs if s is not None]
    if segs:
        segs[-1].final = True
        segs[-1].end = end          # exacta (la del bloque)
    return segs


def describe(block):
    p = block.get('params', {})
    kind = block.get('kind')
    if kind == 'goto':
        safe = ' · despeje Z' if p.get('safe') else ''
        return (f"GO TO POSE ({float(p.get('x', 0)):.1f}, {float(p.get('y', 0)):.1f}, "
                f"{float(p.get('z', 0)):.1f}) codo {int(p.get('codo', 1)):+d}{safe}")
    if kind == 'rotate':
        try:
            return f"ROTATE {rotate_deg(p, block.get('id')):+d}° (relativo)"
        except PlanError:
            return 'ROTATE'
    return KIND_LABEL.get(kind, kind)


def rotate_deg(p, block_id):
    """Angulo del bloque ROTATE (entero, con signo). Acepta bloques viejos con 'dir'."""
    raw = p.get('deg', p.get('dir', 90))
    try:
        v = float(raw)
    except (TypeError, ValueError):
        raise PlanError('ROTATE: el angulo esta vacio o no es un numero.', block_id) from None
    d = int(round(v))
    lo, hi = ROTATE_LIMITS
    if d == 0 or not lo <= d <= hi:
        raise PlanError(f'ROTATE: el angulo debe ser un entero entre {lo} y {hi} y distinto '
                        f'de 0 (escribiste {raw}).', block_id)
    return d


def plan_cycle(program, start_pose, servo_deg, repeats, z_clear, d1, d2, tuning=None):
    """Arma todos los pasos de `repeats` repeticiones. Lanza PlanError.

    start_pose: Pose real del robot (encoders) o None si no se conoce.
    servo_deg:  angulo actual del servo (Q4) para avisar si un ROTATE satura.
    """
    tun = tuning or Tuning()
    tun.check()
    order, warnings = order_blocks(program)
    if not 0 <= z_clear <= Z_CLEAR_MAX:
        raise PlanError(f'Despeje Z = {z_clear:g} mm fuera de [0, {Z_CLEAR_MAX:g}].')
    repeats = int(repeats)
    if repeats < 1:
        raise PlanError('Repeticiones debe ser 1 o mas.')

    # Validar TODO antes de mover (IK + limites de cada GO TO POSE)
    poses = {b['id']: goto_pose(b, d1, d2) for b in order if b.get('kind') == 'goto'}
    for b in order:
        if b.get('kind') not in KIND_LABEL:
            raise PlanError(f"Bloque desconocido: {b.get('kind')}", b['id'])
        if b['kind'] in ('pick', 'place', 'rotate'):
            w = _num(b.get('params', {}), 'wait', b['id'], KIND_LABEL[b['kind']])
            if w < 0:
                raise PlanError(f"{KIND_LABEL[b['kind']]}: la espera no puede ser negativa.",
                                b['id'])
    if poses and start_pose is None:
        raise PlanError('No se conoce la posicion del robot: haz el homing primero.')
    if start_pose is not None and not in_limits(
            {'th1': start_pose.th1, 'th2': start_pose.th2, 'z': start_pose.z}):
        warnings.append('El robot esta fuera de los limites de la interfaz; el primer '
                        'tramo parte de ahi.')

    plan = Plan(order=order, warnings=warnings)
    pose, servo = start_pose, int(round(servo_deg or 0))
    for cycle in range(1, repeats + 1):
        for i, b in enumerate(order, start=1):
            bid, kind, p = b['id'], b['kind'], b.get('params', {})
            plan.steps.append(BlockStart(bid, i, cycle, describe(b)))
            if kind == 'goto':
                end = poses[bid]
                for m in goto_moves(bid, pose, end, bool(p.get('safe')), z_clear, tun):
                    plan.steps.append(m)
                    plan.duration += m.duration * (1.0 if m.final else tun.blend_pct / 100.0)
                plan.steps.append(Arrive(bid, end))
                pose = end
            elif kind in ('pick', 'place'):
                plan.steps.append(Command(bid, 'vac', 1 if kind == 'pick' else 0))
                plan.steps.append(VacConfirm(bid, kind == 'pick'))
                plan.steps.append(Dwell(bid, float(p.get('wait', 0))))
                plan.duration += float(p.get('wait', 0))
            elif kind == 'rotate':
                d = rotate_deg(p, bid)
                new = servo + d
                lo, hi = SERVO_LIMITS
                if not lo <= new <= hi:
                    raise PlanError(
                        f'ROTATE {d:+d}° (bloque {i}, ciclo {cycle}): el servo esta en '
                        f'{servo:+d}° y quedaria en {new:+d}°, fuera de su rango {lo:+d}°…{hi:+d}° '
                        f'(el servo solo gira 180° en total). Reparte el giro o empieza desde '
                        f'el otro extremo.', bid)
                servo = new
                plan.steps.append(Command(bid, 'rot', d))
                plan.steps.append(Dwell(bid, float(p.get('wait', 0))))
                plan.duration += float(p.get('wait', 0))
    return plan
