"""Orientacion global de la boquilla (trajectory.py): sin ROS ni NiceGUI."""
import math

import pytest

from scara_app.robot_model import ik_xy
from scara_app.trajectory import (
    HoldHeading, PlanError, Pose, SERVO_LIMITS, nozzle_heading, nozzle_servo, plan_cycle,
)

D1, D2 = 120.0, 100.0


def goto(bid, x, y, z=54.6, codo=1):
    return {'id': bid, 'kind': 'goto', 'params': {'x': x, 'y': y, 'z': z, 'codo': codo, 'safe': False}}


def rot(bid, deg):
    return {'id': bid, 'kind': 'rotate', 'params': {'deg': deg, 'wait': 0.2}}


def program(*blocks):
    return {'blocks': list(blocks),
            'links': [{'from': a['id'], 'to': b['id']} for a, b in zip(blocks, blocks[1:])]}


def start_pose():
    th1, th2 = ik_xy(150.0, 0.0, 1, D1, D2)
    return Pose(th1, th2, 54.6)


def test_servo_heading_roundtrip():
    for th1, th2, servo in ((10, 30, 20), (-40, 70, -50), (0, 0, 0)):
        h = nozzle_heading(th1, th2, servo)
        assert nozzle_servo(h, th1, th2) == pytest.approx(servo)


def test_holds_initial_heading_and_rotate_is_absolute():
    sp = start_pose()
    prog = program(goto('a', 150.0, 60.0), rot('r', 90), goto('b', 170.0, -40.0))
    plan = plan_cycle(prog, sp, 0, 1, 5.0, D1, D2, global_heading=True)
    holds = [s for s in plan.steps if isinstance(s, HoldHeading)]
    assert holds[0].block_id is None
    assert holds[0].heading == pytest.approx(sp.th1 + sp.th2)
    assert holds[1].heading == 90.0 and holds[1].block_id == 'r'
    # servo esperado = 90 - (th1 + th2) en la pose del ROTATE (la del bloque a)
    th1, th2 = ik_xy(150.0, 60.0, 1, D1, D2)
    assert holds[1].servo == round(90 - (th1 + th2))


def test_servo_out_of_range_is_rejected_before_moving():
    sp = start_pose()
    prog = program(rot('r', 170))     # th1+th2 ~ 0 -> servo 170: fuera de +-90
    with pytest.raises(PlanError) as e:
        plan_cycle(prog, sp, 0, 1, 5.0, D1, D2, global_heading=True)
    assert e.value.block_id == 'r' and 'rango' in str(e.value)


def test_far_pose_makes_servo_saturate_and_is_reported():
    sp = start_pose()
    prog = program(rot('r', 80), goto('far', 40.0, 150.0))
    with pytest.raises(PlanError) as e:
        plan_cycle(prog, sp, 0, 1, 5.0, D1, D2, global_heading=True)
    assert e.value.block_id == 'far'


def test_zero_is_valid_when_absolute_and_old_mode_unchanged():
    sp = start_pose()
    plan_cycle(program(rot('r', 0)), sp, 0, 1, 5.0, D1, D2, global_heading=True)
    with pytest.raises(PlanError):
        plan_cycle(program(rot('r', 0)), sp, 0, 1, 5.0, D1, D2)
    plan = plan_cycle(program(rot('r', 45)), sp, 0, 1, 5.0, D1, D2)
    assert not any(isinstance(s, HoldHeading) for s in plan.steps)


def test_global_needs_known_pose():
    with pytest.raises(PlanError):
        plan_cycle(program(rot('r', 10)), None, 0, 1, 5.0, D1, D2, global_heading=True)


def test_monotone_sum_so_endpoints_are_enough():
    # th1 + th2 a lo largo de un tramo reposo-reposo es monotona
    a, b = Pose(0, 10, 0), Pose(60, -40, 0)
    vals = [a.lerp(b, s / 50).th1 + a.lerp(b, s / 50).th2 for s in range(51)]
    assert all(v2 <= v1 for v1, v2 in zip(vals, vals[1:])) or \
        all(v2 >= v1 for v1, v2 in zip(vals, vals[1:]))
    assert SERVO_LIMITS == (-90, 90) and math.isfinite(sum(vals))
