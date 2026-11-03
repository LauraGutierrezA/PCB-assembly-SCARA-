import math

import pytest

from scara_twin.conversions import (
    Z_TOP_MM, home_joints, joints_array_to_joints, limits_to_joints, parse_limits,
    smooth_step, to_joints,
)

SAMPLE = ('L1=0 R1=1 L2=0 R2=0 L3=0 R3=0 M1=0 M2=1 M3=0 '
          'E1=1234 E2=-56 E3=0 Q1=12.3 Q2=-0.5 Q3=70.5 CAL=1')


def test_parse_limits_types():
    d = parse_limits(SAMPLE)
    assert d['E1'] == 1234 and d['E2'] == -56
    assert d['Q1'] == pytest.approx(12.3) and d['Q3'] == pytest.approx(70.5)
    assert d['R1'] is True and d['L1'] is False
    assert d['CAL'] is True


def test_parse_limits_ignores_garbage():
    d = parse_limits('Q1=abc Q2=5.0 basura E1=x')
    assert d == {'Q2': 5.0}


def test_units_and_z_offset():
    th1, th2, z = limits_to_joints(parse_limits(SAMPLE))
    assert th1 == pytest.approx(math.radians(12.3))
    assert th2 == pytest.approx(math.radians(-0.5))
    assert z == pytest.approx((70.5 - Z_TOP_MM) / 1000.0)


def test_z_extremes_match_urdf_limits():
    assert to_joints(0, 0, 0.0)[2] == pytest.approx(-0.202)
    assert to_joints(0, 0, 109.2)[2] == pytest.approx(-0.0928)
    assert home_joints()[2] == pytest.approx(-0.1474)   # Q3 = 54.6 mm (JOINT_HOME)


def test_clamps_to_joint_limits():
    th1, th2, z = to_joints(200.0, -200.0, 500.0)
    assert th1 == pytest.approx(math.radians(85.0))
    assert th2 == pytest.approx(math.radians(-90.0))
    assert z == pytest.approx((109.2 - Z_TOP_MM) / 1000.0)


def test_uncalibrated_is_ignored():
    d = parse_limits(SAMPLE.replace('CAL=1', 'CAL=0'))
    assert limits_to_joints(d) is None
    assert limits_to_joints(d, require_cal=False) is not None


def test_missing_cal_assumed_calibrated():
    d = parse_limits('Q1=1.0 Q2=2.0 Q3=50.0')
    assert limits_to_joints(d) is not None


def test_missing_q_returns_none():
    assert limits_to_joints(parse_limits('CAL=1 Q1=1.0')) is None


def test_joints_array_matches_limits_text():
    # El arreglo de /scara/joints debe dar EXACTAMENTE lo mismo que el texto de /scara/limits
    from_text = limits_to_joints(parse_limits(SAMPLE))
    from_array = joints_array_to_joints([12.3, -0.5, 70.5, 0.0, 1.0])
    assert from_array == pytest.approx(from_text)


def test_joints_array_float32_precision():
    # El firmware manda float32: el error debe ser despreciable para el gemelo
    import struct
    f32 = lambda v: struct.unpack('f', struct.pack('f', v))[0]
    exact = joints_array_to_joints([33.3, -61.7, 101.4, 90.0, 1.0])
    rounded = joints_array_to_joints([f32(33.3), f32(-61.7), f32(101.4), 90.0, 1.0])
    assert rounded == pytest.approx(exact, abs=1e-6)


def test_joints_array_uncalibrated_and_short():
    assert joints_array_to_joints([1.0, 2.0, 50.0, 0.0, 0.0]) is None
    assert joints_array_to_joints([1.0, 2.0, 50.0, 0.0, 0.0], require_cal=False) is not None
    assert joints_array_to_joints([1.0, 2.0, 50.0]) is None
    assert joints_array_to_joints([]) is None
    assert joints_array_to_joints(None) is None


def test_joints_array_non_finite_ignored():
    assert joints_array_to_joints([float('nan'), 0.0, 50.0, 0.0, 1.0]) is None
    assert joints_array_to_joints([0.0, float('inf'), 50.0, 0.0, 1.0]) is None


def test_smooth_step_converges():
    cur, tgt = home_joints(), to_joints(40.0, 30.0, 100.0)
    for _ in range(60):
        cur = smooth_step(cur, tgt, 0.4)
    assert all(c == pytest.approx(t, abs=1e-6) for c, t in zip(cur, tgt))
    assert smooth_step(cur, home_joints(), 1.0) == pytest.approx(home_joints())
