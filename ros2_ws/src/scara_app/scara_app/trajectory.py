"""
trajectory.py - Planeacion del ciclo de la pestana "Trayectoria" (sin ROS ni NiceGUI).

Un PROGRAMA es lo que el operador arma en el sandbox:
    {'blocks': [{'id': 'b3', 'kind': 'goto', 'params': {...}}, ...],
     'links':  [{'from': 'b3', 'to': 'b4'}, ...]}

Tipos de bloque (parametros):
    goto   -> x, y, z [mm], codo (+1/-1), safe (pasar por la Z segura)
    pick   -> wait [s]       bomba ON  y espera (que agarre)
    place  -> wait [s]       bomba OFF y espera (que suelte)
    rotate -> dir (+90/-90), wait [s]   giro RELATIVO del servo de la boquilla

Flujo (igual que la guia de trayectorias, secciones 3 y 5):
  1. order_blocks(): sigue las flechas desde el bloque INICIO (el unico sin
     flecha de entrada) -> lista ordenada. Detecta ramas, ciclos y bloques
     sueltos.
  2. plan_cycle(): para cada GO TO POSE resuelve la IK (la misma del
     firmware, robot_model.ik_xy, con el codo del bloque) y revisa los
     limites articulares ANTES de mover el robot. Luego arma los tramos:
        - con "Z segura": sube vertical -> viaja en XY -> baja vertical
          (aproximacion vertical en Z que pide la Estacion 3)
        - sin "Z segura": un solo tramo con th1, th2 y z sincronizados
     Cada tramo es un perfil QUINTICO reposo-reposo en espacio ARTICULAR
     (th1, th2, z): velocidad y aceleracion continuas, arranca y para quieto,
     y como los limites articulares son una "caja", si los extremos estan
     dentro todo el tramo tambien lo esta (no hay singularidades ni flips
     a mitad de camino).
     Duracion: dT = max(1.875*L/v_max, sqrt(5.77*L/a_max)) por eje, el mayor.
  3. La pestana recorre los pasos: muestrea el tramo a ~20 Hz y manda cada
     punto a /scara/target como consigna cartesiana (FK -> x, y; el firmware
     hace la IK y el PD sigue la referencia). Al final de cada GO TO POSE
     espera ARR=1 (el firmware dice que los 3 motores llegaron).
"""
import math
from dataclasses import dataclass, field

from scara_app.robot_model import JOINT_LIMITS, codo_for, fk_xy, ik_xy, in_limits

# ======================================================================
#   LIMITES DEL PERFIL  (AJUSTAR con el robot real)
# ======================================================================
# Estimados con el motor de 200 RPM y las reducciones nominales del firmware
# (homing_pro.c): M1 50*4 -> 300 grados/s, M2 50*3 -> 400 grados/s,
# M3 50*3 con tornillo de 2 mm/vuelta -> ~2.2 mm/s. Se usa menos que el
# maximo para que el PD alcance a seguir la referencia.
V_MAX_DEG = 45.0       # grados/s   (th1, th2)
A_MAX_DEG = 90.0       # grados/s^2
V_MAX_Z = 1.8          # mm/s
A_MAX_Z = 3.0          # mm/s^2
MIN_SEGMENT_S = 0.3    # duracion minima de un tramo

Z_SAFE_DEFAULT = 50.0  # mm sobre el piso (el eje Z es lento: ~2 mm/s)
SAME_XY_DEG = 0.05     # si th1 y th2 cambian menos que esto, no hay viaje en XY

SERVO_LIMITS = (-90, 90)

KIND_LABEL = {'goto': 'GO TO POSE', 'pick': 'PICK', 'place': 'PLACE', 'rotate': 'ROTATE'}

DEFAULT_PARAMS = {
    'goto':   {'x': 180.0, 'y': 0.0, 'z': 72.6, 'codo': 1, 'safe': True},
    'pick':   {'wait': 0.5},
    'place':  {'wait': 0.8},
    'rotate': {'dir': 90, 'wait': 0.4},
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
class Move:
    block_id: str
    start: Pose
    end: Pose
    duration: float
    label: str          # 'subir', 'viaje XY', 'bajar', 'tramo'

    def at(self, t):
        return self.start.lerp(self.end, quintic_s(t / self.duration if self.duration else 1.0))


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


def _segment(block_id, a, b, label):
    T = max(quintic_duration(b.th1 - a.th1, V_MAX_DEG, A_MAX_DEG),
            quintic_duration(b.th2 - a.th2, V_MAX_DEG, A_MAX_DEG),
            quintic_duration(b.z - a.z, V_MAX_Z, A_MAX_Z))
    if T == 0.0:
        return None
    return Move(block_id, a, b, max(T, MIN_SEGMENT_S), label)


def goto_moves(block_id, start, end, safe, z_safe):
    """Tramos de start a end. Con safe: subir -> viaje XY -> bajar."""
    if not safe or start.same_xy(end):
        segs = [_segment(block_id, start, end, 'tramo')]
    else:
        z_travel = max(z_safe, start.z, end.z)
        up = Pose(start.th1, start.th2, z_travel)
        over = Pose(end.th1, end.th2, z_travel)
        segs = [_segment(block_id, start, up, 'subir'),
                _segment(block_id, up, over, 'viaje XY'),
                _segment(block_id, over, end, 'bajar')]
    return [s for s in segs if s is not None]


def describe(block):
    p = block.get('params', {})
    kind = block.get('kind')
    if kind == 'goto':
        safe = ' · Z segura' if p.get('safe') else ''
        return (f"GO TO POSE ({float(p.get('x', 0)):.1f}, {float(p.get('y', 0)):.1f}, "
                f"{float(p.get('z', 0)):.1f}) codo {int(p.get('codo', 1)):+d}{safe}")
    if kind == 'rotate':
        return f"ROTATE {int(p.get('dir', 90)):+d}°"
    return KIND_LABEL.get(kind, kind)


def plan_cycle(program, start_pose, servo_deg, repeats, z_safe, d1, d2):
    """Arma todos los pasos de `repeats` repeticiones. Lanza PlanError.

    start_pose: Pose real del robot (encoders) o None si no se conoce.
    servo_deg:  angulo actual del servo (Q4) para avisar si un ROTATE satura.
    """
    order, warnings = order_blocks(program)
    lo, hi = JOINT_LIMITS['z']
    if not lo <= z_safe <= hi:
        raise PlanError(f'Z segura = {z_safe:g} mm fuera de [{lo:g}, {hi:g}].')
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
                for m in goto_moves(bid, pose, end, bool(p.get('safe')), z_safe):
                    plan.steps.append(m)
                    plan.duration += m.duration
                plan.steps.append(Arrive(bid, end))
                pose = end
            elif kind in ('pick', 'place'):
                plan.steps.append(Command(bid, 'vac', 1 if kind == 'pick' else 0))
                plan.steps.append(Dwell(bid, float(p.get('wait', 0))))
                plan.duration += float(p.get('wait', 0))
            elif kind == 'rotate':
                d = 90 if int(p.get('dir', 90)) >= 0 else -90
                new = min(SERVO_LIMITS[1], max(SERVO_LIMITS[0], servo + d))
                if new == servo:
                    msg = (f'Bloque {i} (ROTATE {d:+d}°): el servo ya esta en {servo:+d}° '
                           f'(ciclo {cycle}), no gira.')
                    if not any(w.startswith(f'Bloque {i} (ROTATE') for w in warnings):
                        warnings.append(msg)
                servo = new
                plan.steps.append(Command(bid, 'rot', d))
                plan.steps.append(Dwell(bid, float(p.get('wait', 0))))
                plan.duration += float(p.get('wait', 0))
    return plan
