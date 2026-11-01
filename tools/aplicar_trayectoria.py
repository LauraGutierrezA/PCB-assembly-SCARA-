#!/usr/bin/env python3
"""
aplicar_trayectoria.py - Agrega la pestana "Trayectoria" a archivos que YA
existen en el repo, con cambios pequenos y anclados (no reemplaza archivos).

    python3 tools/aplicar_trayectoria.py            # desde la raiz del repo
    python3 tools/aplicar_trayectoria.py --check    # solo revisa, no escribe

Que hace (cada cambio lleva la marca TRAYECTORIA para poder ubicarlo):
  firmware/components/motors/include/pins.h  -> PIN_SERVO 16, PIN_VACUUM 12
  firmware/components/control/include/control.h -> prototipo control_arrived()
  firmware/components/control/control.c      -> funcion control_arrived() al final
  firmware/main/main.c                       -> include, comandos vac/rot,
                                                ARR/VAC/Q4 en /scara/limits,
                                                effector_init()
  ros2_ws/.../web/app.py                     -> pestana en TABS + aviso de cambio
  ros2_ws/.../web/ros_node.py                -> stream_target, vacuum, rotate, evento estop

- Es idempotente: si un archivo ya tiene la marca, lo salta.
- Antes de escribir guarda una copia <archivo>.bak.
- Si un ancla no aparece (porque tu archivo local cambio), NO toca ese
  archivo y te dice que cambio aplicar a mano (ver CAMBIOS_MANUALES.md).
"""
import re
import sys
from pathlib import Path

MARK = 'TRAYECTORIA'
WEB = Path('ros2_ws/src/scara_app/scara_app/web')


class AnchorError(Exception):
    pass


def insert_before(text, pattern, block, start_at=None, what=''):
    start = 0
    if start_at is not None:
        m0 = re.search(start_at, text)
        if not m0:
            raise AnchorError(what or start_at)
        start = m0.start()
    m = re.compile(pattern).search(text, start)
    if not m:
        raise AnchorError(what or pattern)
    return text[:m.start()] + block + text[m.start():]


def insert_after(text, pattern, block, start_at=None, what=''):
    start = 0
    if start_at is not None:
        m0 = re.search(start_at, text)
        if not m0:
            raise AnchorError(what or start_at)
        start = m0.start()
    m = re.compile(pattern).search(text, start)
    if not m:
        raise AnchorError(what or pattern)
    return text[:m.end()] + block + text[m.end():]


# ---------------------------------------------------------------- firmware
def patch_pins(t):
    return insert_before(t, r'#endif\s*/\*\s*PINS_H\s*\*/', (
        '    /* ---------- Efector final (TRAYECTORIA) ---------- */\n'
        '    /* Servo de giro Tower Pro 9 g: senal PWM 50 Hz (alimentarlo a 5 V aparte,\n'
        '     * GND comun con el ESP32). */\n'
        '    #define PIN_SERVO     16\n'
        '    /* Bomba de vacio via MOSFET/rele. GPIO 12 es pin de arranque: debe\n'
        '     * estar en BAJO al encender -> pull-down de 10k en la compuerta. */\n'
        '    #define PIN_VACUUM    12\n\n'), what='#endif /* PINS_H */')


def patch_control_h(t):
    return insert_before(t, r'#endif\s*/\*\s*CONTROL_H\s*\*/', (
        '/* TRAYECTORIA: true si hay consigna y los 3 motores ya llegaron a ella\n'
        ' * (misma histeresis "llego" del lazo). Se publica como ARR en /scara/limits. */\n'
        'bool control_arrived(void);\n\n'), what='#endif /* CONTROL_H */')


def patch_control_c(t):
    if not re.search(r'static\s+bool\s+llego\s*\[', t) or 'ref_lock' not in t:
        raise AnchorError('static bool llego[...] / ref_lock')
    return t.rstrip('\n') + (
        '\n\n\n/* ============================================================\n'
        ' *        TRAYECTORIA: el robot llego a la consigna\n'
        ' * ============================================================ */\n'
        'bool control_arrived(void)\n'
        '{\n'
        '    bool ok;\n'
        '    portENTER_CRITICAL(&ref_lock);\n'
        '    ok = has_ref && llego[0] && llego[1] && llego[2];\n'
        '    portEXIT_CRITICAL(&ref_lock);\n'
        '    return ok;\n'
        '}\n')


def patch_main_c(t):
    t = insert_after(t, r'#include\s+"motor_test\.h"[^\n]*\n',
                     '#include "effector.h"          /* TRAYECTORIA: ventosa + servo */\n',
                     what='#include "motor_test.h"')
    t = insert_after(
        t,
        r'static void cmd_callback\(const void \*msgin\)\s*\{\s*'
        r'const std_msgs__msg__String \*m = \(const std_msgs__msg__String \*\)msgin;[^\n]*\n',
        '    /* TRAYECTORIA: "vac1", "vac0", "rot +90", "rot -90" (y "stop" apaga la bomba) */\n'
        '    if (effector_handle_cmd(m->data.data, m->data.size)) return;\n',
        what='inicio de cmd_callback (const std_msgs__msg__String *m = ...)')
    t = insert_before(
        t, r'\n[ \t]*TickType_t now = xTaskGetTickCount\(\);',
        '\n    /* TRAYECTORIA: llego (ARR), bomba (VAC) y angulo del servo (Q4) */\n'
        '    {\n'
        '        size_t len_ = strlen(limits_buf);\n'
        '        snprintf(limits_buf + len_, sizeof(limits_buf) - len_,\n'
        '                 " ARR=%d VAC=%d Q4=%d", control_arrived() ? 1 : 0,\n'
        '                 effector_vacuum_on() ? 1 : 0, effector_servo_deg());\n'
        '    }',
        start_at=r'static void limits_publish_tick\(void\)',
        what='TickType_t now = xTaskGetTickCount(); dentro de limits_publish_tick')
    t = insert_after(t, r'encoders_init\(\);[^\n]*\n',
                     '    effector_init();          /* TRAYECTORIA */\n',
                     start_at=r'void app_main\(void\)', what='encoders_init(); en app_main')
    return t


# ---------------------------------------------------------------- web
def patch_app(t):
    t = insert_after(t, r'from \.point_to_point_tab import PointToPointTab[^\n]*\n',
                     'from .trajectory_tab import TrajectoryTab   # TRAYECTORIA\n',
                     what='from .point_to_point_tab import PointToPointTab')
    t = insert_after(t, r"\(\s*'dk'\s*,\s*'DK'\s*,\s*DkTab\s*\)\s*,[^\n]*\n",
                     "    ('traj', 'Trayectoria', TrajectoryTab),   # TRAYECTORIA\n",
                     what="('dk', 'DK', DkTab), en TABS")
    t = insert_after(
        t, r"\n([ \t]*)name = getattr\(e\.value, '_props', \{\}\)\.get\('name', e\.value\)[^\n]*\n",
        "            for page in pages.values():          # TRAYECTORIA: aborta el ciclo\n"
        "                if hasattr(page, 'on_tab_change'):\n"
        "                    page.on_tab_change(name)\n",
        what="name = getattr(e.value, '_props', {}).get('name', e.value) en on_tab_change")
    return t


def patch_ros_node(t):
    t = insert_after(t, r'def emergency_stop\(self\):(?:.|\n)*?self\.test_stop\(None\)[^\n]*\n',
                     "        self._push('estop', None)   # TRAYECTORIA: las pestanas abortan\n",
                     what='self.test_stop(None) dentro de emergency_stop')
    return insert_before(t, r'\n    def _send_test\(self, text: str\):', '''
    # ---------------- Trayectoria (TRAYECTORIA) ----------------
    def stream_target(self, x: float, y: float, z: float, params: dict, codo: int = 1):
        """Como send_target pero SIN log: la pestana Trayectoria manda ~20 por segundo."""
        msg = Float32MultiArray()
        msg.data = [float(v) for v in (x, y, z,
                    params['d1'], params['d2'], params['d3'],
                    params['d4'], params['d5'], params['d6'], params['av'],
                    1 if codo >= 0 else -1)]
        self._target_pub.publish(msg)

    def vacuum(self, on: bool):
        """Bomba de la ventosa: PICK -> True, PLACE -> False."""
        self.send_command('vac1' if on else 'vac0')

    def rotate(self, delta_deg: int):
        """Servo de la boquilla: giro relativo de +90 o -90 grados."""
        self.send_command(f'rot {int(delta_deg):+d}')
''', what='def _send_test(self, text: str):')


PATCHES = [
    (Path('firmware/components/motors/include/pins.h'), patch_pins),
    (Path('firmware/components/control/include/control.h'), patch_control_h),
    (Path('firmware/components/control/control.c'), patch_control_c),
    (Path('firmware/main/main.c'), patch_main_c),
    (WEB / 'app.py', patch_app),
    (WEB / 'ros_node.py', patch_ros_node),
]


def main():
    check = '--check' in sys.argv
    root = Path.cwd()
    if not (root / 'firmware').is_dir() or not (root / 'ros2_ws').is_dir():
        print('Ejecuta este script desde la raiz del repo (donde estan firmware/ y ros2_ws/).')
        return 2
    failed = 0
    for path, fn in PATCHES:
        if not path.exists():
            print(f'  [FALTA]  {path}')
            failed += 1
            continue
        text = path.read_text()
        if MARK in text:
            print(f'  [ya]     {path}')
            continue
        try:
            new = fn(text)
        except AnchorError as e:
            print(f'  [MANUAL] {path}: no encontre "{e}". Aplica el cambio a mano '
                  f'(CAMBIOS_MANUALES.md).')
            failed += 1
            continue
        if check:
            print(f'  [ok]     {path} (sin escribir: --check)')
            continue
        path.with_name(path.name + '.bak').write_text(text)
        path.write_text(new)
        print(f'  [hecho]  {path}  (copia en {path.name}.bak)')
    print('Listo.' if not failed else f'{failed} archivo(s) para revisar a mano.')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
