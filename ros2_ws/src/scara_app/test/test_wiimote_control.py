"""
Pruebas de la logica del Wiimote con datos simulados (sin robot, sin ROS, sin Qt).

    cd ros2_ws/src/scara_app && python3 -m pytest test/test_wiimote_control.py
"""
import math

from scara_app.robot_model import JOINT_LIMITS, PLAN_Y_RIGHT, fk_xy
from scara_app.wiimote_bridge import (
    BEAT_PERIOD_S, LINK_TIMEOUT_S, WiimoteBridge,
)
from scara_app.wiimote_control import MODE_DK, MODE_IK, WiimoteController

D1, D2 = 120.0, 85.0
BENT = {'th1': 0.0, 'th2': 60.0, 'z': 72.6}      # codo doblado: lejos de la singularidad
STILL = {'x': 0.0, 'y': 0.0, 'z': 0.0}
DT = 0.02


def run(ctrl, seconds, buttons=(), gyro=STILL, dt=DT):
    for _ in range(int(round(seconds / dt))):
        ctrl.update(dt, set(buttons), gyro)


def make(mode=MODE_IK, actual=BENT, simulate=True):
    """Por defecto en simulacion (sin correa): asi se prueba la cinematica y los limites
    sin un robot que siga a la consigna. La correa se prueba con simulate=False."""
    c = WiimoteController(D1, D2)
    c.simulate = simulate
    c.set_actual(actual)
    c.q = dict(actual)
    c.mode = mode
    c.codo = 1
    return c


# ---------------------------------------------------------------- IK
def test_ik_no_se_mueve_sin_b():
    c = make()
    run(c, 1.0, {'UP'})
    assert c.q == BENT
    assert not c.held


def test_ik_dpad_arriba_aumenta_x_a_la_velocidad_configurada():
    c = make()
    x0, y0 = c.xy()
    run(c, 1.0, {'B', 'UP'})
    x1, y1 = c.xy()
    # 20 mm/s pedidos; el tope de velocidad de las articulaciones (15 grados/s) puede frenar
    assert 5.0 < x1 - x0 <= c.params.speed_xy_mmps + 0.5
    assert abs(y1 - y0) < 0.5


def test_ik_izquierda_es_y_positivo_y_diagonal_no_es_mas_rapida():
    c = make()
    x0, y0 = c.xy()
    run(c, 1.0, {'B', 'LEFT'})
    assert (c.xy()[1] > y0) == (not PLAN_Y_RIGHT)       # +Y es el lado que muestra el gemelo
    c3 = make()
    run(c3, 1.0, {'B', 'RIGHT'})
    assert (c3.xy()[1] > y0) == PLAN_Y_RIGHT
    c2 = make()
    x0, y0 = c2.xy()
    run(c2, 1.0, {'B', 'UP', 'LEFT'})
    x1, y1 = c2.xy()
    assert math.hypot(x1 - x0, y1 - y0) < c2.params.speed_xy_mmps * 1.05


def test_ik_z_con_mas_y_menos_y_limites():
    c = make()
    run(c, 1.0, {'B', 'PLUS'})
    # 1 s a 10 mm/s, menos lo que tarda la rampa en llegar a velocidad completa
    assert abs(c.q['z'] - (BENT['z'] + c.params.speed_z_mmps - c.params.ramp_s * 5.0)) < 0.5
    run(c, 30.0, {'B', 'PLUS'})
    assert c.q['z'] <= JOINT_LIMITS['z'][1] + 1e-9
    assert c.blocked
    run(c, 60.0, {'B', 'MINUS'})
    assert c.q['z'] >= JOINT_LIMITS['z'][0] - 1e-9


def test_ik_no_pasa_el_alcance_y_avisa_bloqueado():
    c = make()
    run(c, 30.0, {'B', 'UP'})
    x, y = c.xy()
    assert math.hypot(x, y) <= D1 + D2 + 1e-3
    assert c.blocked
    assert ('blocked', '') in c.events


def test_ik_respeta_limites_de_articulacion_y_tope_de_velocidad():
    c = make()
    prev = dict(c.q)
    for _ in range(int(20 / DT)):
        c.update(DT, {'B', 'LEFT'}, STILL)
        for k in ('th1', 'th2'):
            assert abs(c.q[k] - prev[k]) <= c.params.speed_th_dps * DT + 1e-6
            assert JOINT_LIMITS[k][0] - 1e-6 <= c.q[k] <= JOINT_LIMITS[k][1] + 1e-6
        prev = dict(c.q)


# ---------------------------------------------------------------- DK
def test_dk_ruido_pequeno_no_mueve_nada():
    c = make(MODE_DK)
    for i in range(int(20 / DT)):
        noise = 1.5 * math.sin(i)                 # < rate_gate_dps (2 deg/s)
        c.update(DT, {'B'}, {'x': noise, 'y': -noise, 'z': noise})
    assert c.q == BENT


def test_dk_giro_pequeno_dentro_de_la_zona_muerta_no_mueve():
    c = make(MODE_DK)
    run(c, 0.5, {'B'}, {'x': 20.0, 'y': 0.0, 'z': 0.0})      # 10 grados < 15
    assert c.q['th1'] == BENT['th1']
    run(c, 0.5, {'B'}, {'x': -20.0, 'y': 0.0, 'z': 0.0})     # de vuelta al centro
    assert c.q['th1'] == BENT['th1']


def test_dk_giro_grande_mueve_y_volver_al_centro_detiene():
    c = make(MODE_DK)
    run(c, 3.0, {'B'}, {'x': 30.0, 'y': 0.0, 'z': 0.0})      # 90 grados: pasa el maximo
    assert c.angles['th1'] == c.params.full_angle_deg
    moved = c.q['th1'] - BENT['th1']
    assert 0.0 < moved <= c.params.speed_th_dps * 3.0
    run(c, 1.0, {'B'}, {'x': 0.0, 'y': 0.0, 'z': 0.0})       # quieto, pero todavia girado
    assert c.q['th1'] - BENT['th1'] > moved + 10.0            # sigue (estilo joystick)
    run(c, 1.0, {'B'}, {'x': -70.0, 'y': 0.0, 'z': 0.0})     # regresa al centro
    assert abs(c.angles['th1']) < 1.0
    at_neutral = c.q['th1']
    run(c, 2.0, {'B'}, {'x': 0.0, 'y': 0.0, 'z': 0.0})       # centrado: se queda quieto
    assert abs(c.q['th1'] - at_neutral) < 1e-9


def test_dk_el_angulo_se_pone_en_cero_al_soltar_b():
    c = make(MODE_DK)
    run(c, 1.0, {'B'}, {'x': 30.0, 'y': 0.0, 'z': 0.0})
    assert c.angles['th1'] != 0.0
    run(c, 0.1, set())
    run(c, 0.02, {'B'})
    assert c.angles['th1'] == 0.0


def test_dk_mapeo_de_ejes_y_signos_configurables():
    c = make(MODE_DK)
    c.params.gyro_axes = {'th1': ('z', -1.0), 'th2': ('x', 1.0), 'z': ('y', 1.0)}
    run(c, 2.0, {'B'}, {'x': 0.0, 'y': 0.0, 'z': 30.0})
    assert c.q['th1'] < BENT['th1']               # signo -1
    assert c.q['th2'] == BENT['th2'] and c.q['z'] == BENT['z']


def test_dk_respeta_limites():
    c = make(MODE_DK)
    run(c, 60.0, {'B'}, {'x': 30.0, 'y': 0.0, 'z': 0.0})
    assert c.q['th1'] <= JOINT_LIMITS['th1'][1] + 1e-9
    assert c.blocked


# ---------------------------------------------------------------- Seguridad
def test_cambiar_de_modo_exige_volver_a_presionar_b():
    c = make(MODE_IK)
    run(c, 0.2, {'B', 'UP'})
    assert c.held
    c.update(DT, {'B', 'TWO'}, STILL)
    assert c.mode == MODE_DK and not c.held
    q = dict(c.q)
    run(c, 1.0, {'B'}, {'x': 40.0, 'y': 0.0, 'z': 0.0})
    assert c.q == q                                # B seguia presionado: no mueve
    run(c, 0.1, set())
    run(c, 1.0, {'B'}, {'x': 40.0, 'y': 0.0, 'z': 0.0})
    assert c.held and c.q != q


def test_home_pide_paro_y_exige_repress():
    c = make()
    run(c, 0.2, {'B', 'UP'})
    c.update(DT, {'B', 'UP', 'HOME'}, STILL)
    assert ('stop', 'HOME') in c.pop_events()
    assert not c.held
    q = dict(c.q)
    run(c, 0.5, {'B', 'UP'})
    assert c.q == q


def test_ik_correa_la_consigna_no_se_aleja_del_robot_real():
    c = make(simulate=False)
    run(c, 10.0, {'B', 'LEFT', 'PLUS'})              # el robot "real" no se mueve
    assert abs(c.q['th1'] - BENT['th1']) <= c.params.lead_deg + 1e-9
    assert abs(c.q['th2'] - BENT['th2']) <= c.params.lead_deg + 1e-9
    assert abs(c.q['z'] - BENT['z']) <= c.params.lead_mm + 1e-9


def test_la_consigna_arranca_en_la_posicion_medida_y_no_se_aleja_mas_que_lead():
    c = make(MODE_DK, simulate=False)
    c.q = {'th1': 50.0, 'th2': -30.0, 'z': 100.0}     # consigna vieja (cualquiera)
    c.update(DT, {'B'}, STILL)
    assert c.q == BENT                                # al presionar B se resincroniza
    run(c, 30.0, {'B'}, {'x': 30.0, 'y': 0.0, 'z': 0.0})
    assert c.q['th1'] - BENT['th1'] <= c.params.lead_deg + 1e-9


def test_simulacion_no_usa_correa_ni_resincroniza():
    c = make(MODE_DK, simulate=True)
    c.q = {'th1': 20.0, 'th2': 10.0, 'z': 50.0}
    c.update(DT, {'B'}, STILL)
    assert c.q['th1'] == 20.0
    run(c, 20.0, {'B'}, {'x': 30.0, 'y': 0.0, 'z': 0.0})
    assert c.q['th1'] - 20.0 > c.params.lead_deg


def test_cambio_ik_dk_ik_conserva_la_pose():
    c = make(MODE_IK, simulate=True)
    run(c, 0.5, {'B', 'UP'})
    xy = c.xy()
    c.update(DT, {'TWO'}, STILL)
    c.update(DT, {'ONE'}, STILL)
    assert c.mode == MODE_IK
    assert all(abs(a - b) < 1e-9 for a, b in zip(c.xy(), xy))
    th1, th2 = c.q['th1'], c.q['th2']
    assert all(abs(a - b) < 1e-9 for a, b in zip(fk_xy(th1, th2, D1, D2), xy))


# ---------------------------------------------------------------- Bridge
class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class Rig:
    """WiimoteBridge con relojes y salidas falsas."""

    def __init__(self):
        self.clock = Clock()
        self.sent, self.beats, self.releases, self.estops, self.logs = [], [], [], [], []
        self.states = []
        self.bridge = WiimoteBridge(
            {'d1': D1, 'd2': D2},
            send_joints=lambda a, b, z: self.sent.append((a, b, z)),
            deadman_beat=lambda: self.beats.append(self.clock.t),
            deadman_release=lambda: self.releases.append(self.clock.t),
            emergency_stop=lambda: self.estops.append(self.clock.t),
            log=lambda text, level='info': self.logs.append((level, text)),
            on_state=self.states.append,
            clock=self.clock)

    def limits(self, th1=0.0, th2=60.0, z=72.6):
        self.bridge.on_limits({'CAL': True, 'Q1': th1, 'Q2': th2, 'Q3': z})

    def step(self, seconds, buttons=(), gyro_dps=(0, 0, 0), joy=True):
        for _ in range(int(round(seconds / DT))):
            self.clock.t += DT
            if joy:
                self.bridge.on_joy(list(buttons))
                self.bridge.on_imu(*(math.radians(g) for g in gyro_dps))
            self.bridge.tick()


def test_bridge_desactivado_no_manda_nada():
    r = Rig()
    r.limits()
    r.step(1.0, {'B', 'UP'})
    assert not r.sent and not r.beats


def test_bridge_simulacion_no_manda_consigna_ni_deadman():
    r = Rig()
    r.limits()
    r.bridge.set_enabled(True)                  # dry_run por defecto
    r.step(0.1)
    r.step(1.0, {'B', 'UP'})
    assert not r.sent and not r.beats
    assert r.states[-1]['held'] and r.states[-1]['dry_run']


def test_bridge_real_primero_consigna_igual_a_la_medida_luego_latidos_y_envios():
    r = Rig()
    r.limits()
    r.bridge.set_dry_run(False)
    r.bridge.set_enabled(True)
    r.step(0.1)
    r.step(1.0, {'B', 'UP'})
    assert r.sent[0] == (0.0, 60.0, 72.6)       # sin salto
    assert len(r.sent) >= 5                     # ~cada 80 ms
    assert r.sent[-1] != r.sent[0]
    gaps = [b - a for a, b in zip(r.beats, r.beats[1:])]
    assert gaps and max(gaps) <= BEAT_PERIOD_S + 2 * DT
    assert not r.releases
    r.step(0.1)                                 # suelta B
    assert len(r.releases) == 1


def test_bridge_sin_homing_no_mueve_y_avisa():
    r = Rig()
    r.bridge.set_dry_run(False)
    r.bridge.set_enabled(True)
    r.step(0.1)
    r.step(0.5, {'B', 'UP'})
    assert not r.sent and not r.beats
    assert any(level == 'error' and 'homing' in text for level, text in r.logs)


def test_bridge_si_se_pierde_el_wiimote_suelta_el_deadman():
    r = Rig()
    r.limits()
    r.bridge.set_dry_run(False)
    r.bridge.set_enabled(True)
    r.step(0.1)
    r.step(0.5, {'B', 'UP'})
    assert not r.releases
    r.step(LINK_TIMEOUT_S + 0.2, joy=False)     # dejan de llegar mensajes
    assert len(r.releases) == 1
    n = len(r.beats)
    r.step(0.5, joy=False)
    assert len(r.beats) == n                    # ya no hay latidos


def test_bridge_home_manda_stop_y_se_desactiva():
    r = Rig()
    r.limits()
    r.bridge.set_dry_run(False)
    r.bridge.set_enabled(True)
    r.step(0.1)
    r.step(0.3, {'B', 'UP'})
    r.step(0.1, {'B', 'UP', 'HOME'})
    assert len(r.estops) == 1
    assert r.releases and not r.bridge.enabled
    n = len(r.sent)
    r.step(0.5, {'B', 'UP'})
    assert len(r.sent) == n


def test_bridge_desactivar_con_b_presionado_suelta_el_deadman():
    r = Rig()
    r.limits()
    r.bridge.set_dry_run(False)
    r.bridge.set_enabled(True)
    r.step(0.1)
    r.step(0.3, {'B', 'UP'})
    r.bridge.set_enabled(False)
    assert len(r.releases) == 1
    r.step(0.5, {'B', 'UP'})
    assert len(r.releases) == 1 and len(r.beats) < 8


def test_bridge_activar_con_b_ya_presionado_exige_soltarlo():
    r = Rig()
    r.limits()
    r.bridge.set_dry_run(False)
    r.step(0.1, {'B'})
    r.bridge.set_enabled(True)
    r.step(0.5, {'B', 'UP'})
    assert not r.sent
    r.step(0.1)
    r.step(0.3, {'B', 'UP'})
    assert r.sent


def test_bridge_dk_gira_por_gyro_en_rad_por_segundo():
    r = Rig()
    r.limits()
    r.bridge.set_dry_run(False)
    r.bridge.set_enabled(True)
    r.step(0.1, {'TWO'})
    r.step(0.1)
    r.step(3.0, {'B'}, gyro_dps=(30.0, 0.0, 0.0))
    assert r.sent[-1][0] > r.sent[0][0]
    assert r.sent[-1][1] == r.sent[0][1]


def test_bridge_rechazo_del_firmware_se_registra():
    r = Rig()
    r.limits()
    r.bridge.set_dry_run(False)
    r.bridge.set_enabled(True)
    r.step(0.1)
    r.step(0.3, {'B', 'UP'})
    r.bridge.on_ik_result({'ok': False, 'reason': 'OUT_OF_RANGE', 'joint': 2})
    assert any('rechazada' in text and 'articulacion 2' in text for _, text in r.logs)


def test_bridge_la_simulacion_arranca_en_la_posicion_real():
    r = Rig()
    r.limits(th1=10.0, th2=40.0, z=80.0)
    r.step(0.1)                                   # desactivado: el gemelo ya sigue al robot
    assert r.states[-1]['q'] == {'th1': 10.0, 'th2': 40.0, 'z': 80.0}
    r.bridge.set_enabled(True)
    r.step(0.1)
    r.step(0.5, {'B', 'UP'})
    assert r.states[-1]['q'] != {'th1': 10.0, 'th2': 40.0, 'z': 80.0}
    assert not r.sent
    r.bridge.set_dry_run(False)
    r.bridge.set_dry_run(True)                    # volver a simular
    assert r.bridge.snapshot()['q'] == {'th1': 10.0, 'th2': 40.0, 'z': 80.0}


def test_bridge_al_desactivar_vuelve_a_simulacion():
    r = Rig()
    r.limits()
    r.bridge.set_dry_run(False)
    r.bridge.set_enabled(True)
    r.bridge.set_enabled(False)
    assert r.bridge.snapshot()['dry_run']


# ---------------------------------------------------------------- Suavizado
def test_ik_rampa_arranca_y_frena_gradualmente():
    c = make()
    c.params.ramp_s = 0.5
    full = c.params.speed_z_mmps
    run(c, 0.1, {'B', 'PLUS'})
    assert 0.0 < c._vel[2] < 0.3                  # a los 0.1 s va a ~20 % de la velocidad
    run(c, 1.0, {'B', 'PLUS'})
    assert abs(c._vel[2] - 1.0) < 1e-9            # velocidad completa despues de la rampa
    z0 = c.q['z']
    run(c, 0.1, {'B'})                            # suelta el boton: no se frena en seco
    assert 0.0 < c.q['z'] - z0 < full * 0.1 + 1e-6
    run(c, 1.0, {'B'})
    assert c._vel == [0.0, 0.0, 0.0]


def test_ik_rampa_cero_responde_al_instante():
    c = make()
    c.params.ramp_s = 0.0
    run(c, 0.1, {'B', 'PLUS'})
    assert c._vel[2] == 1.0


def test_zona_muerta_dk_por_defecto_es_25_grados():
    c = make(MODE_DK)
    assert c.params.dead_angle_deg == 25.0
    run(c, 1.0, {'B'}, {'x': 20.0, 'y': 0.0, 'z': 0.0})      # 20 grados < 25: no se mueve
    assert c.q['th1'] == BENT['th1']
