"""
twin.py - Gemelo del robot en la interfaz web: Planta (SVG) y 3D (three.js).

Muestra dos robots superpuestos (colores aprobados):
  - consigna:  cian translucido (a donde va a ir)
  - real:      gris solido (encoders: Q1, Q2, Q3 de /scara/limits)
y una barra de z (19.3 .. 126 mm) con las dos alturas.

La planta tiene dos capas con el mismo viewBox:
  - fondo (malla, zona alcanzable, ejes): se dibuja una sola vez.
  - encima (brazos, cruz del objetivo): se redibuja con cada cambio (pequena).
El 3D usa ui.scene (three.js viene con NiceGUI: funciona sin internet).
Ambas vistas siguen el modelo del HTML "Espacio de Trabajo SCARA".
"""
import base64
import io
import math

from nicegui import ui

from scara_app.gui import theme as t
from scara_app.robot_model import (
    D3_BASE, JOINT_LIMITS, PLAN_VIEW, SINGULAR_DEG, Z_TOP, fk_full, hex_rgb, zone_rgba,
)

from . import theme as wt

PLAN_W, PLAN_H = 600, 440          # viewBox de la planta
ZBAR_W = 66
_M = {'l': 34, 'r': 10, 't': 10, 'b': 24}
Z_LO, Z_HI = JOINT_LIMITS['z']
SCENE_MIN_DELTA = 0.2              # no mandar al 3D cambios menores (grados / mm)


def _plan_transform():
    v = PLAN_VIEW
    s = min((PLAN_W - _M['l'] - _M['r']) / (v['ymax'] - v['ymin']),
            (PLAN_H - _M['t'] - _M['b']) / (v['xmax'] - v['xmin']))
    ox = _M['l'] + ((PLAN_W - _M['l'] - _M['r']) - (v['ymax'] - v['ymin']) * s) / 2
    oy = _M['t']
    return s, (lambda y: ox + (v['ymax'] - y) * s), (lambda x: oy + (v['xmax'] - x) * s), ox, oy


def plan_to_mm(px, py):
    """Coordenadas del viewBox -> (x, y) en mm."""
    s, _, _, ox, oy = _plan_transform()
    return PLAN_VIEW['xmax'] - (py - oy) / s, PLAN_VIEW['ymax'] - (px - ox) / s


_zone_cache = {}


def _zone_data_uri(d1, d2):
    key = (round(d1, 2), round(d2, 2))
    if key not in _zone_cache:
        from PIL import Image
        img = zone_rgba(3, d1, d2, hex_rgb(t.ACCENT), hex_rgb(t.CONNECTED))
        buf = io.BytesIO()
        Image.fromarray(img, 'RGBA').save(buf, 'PNG', optimize=True)
        _zone_cache[key] = 'data:image/png;base64,' + base64.b64encode(buf.getvalue()).decode()
    return _zone_cache[key]


def _svg(body):
    return (f'<svg viewBox="0 0 {PLAN_W} {PLAN_H}" preserveAspectRatio="xMidYMid meet" '
            f'xmlns="http://www.w3.org/2000/svg" class="plan-svg">{body}</svg>')


def plan_background_svg(d1, d2):
    v = PLAN_VIEW
    s, sx, sy, _, _ = _plan_transform()
    o = []
    for val in range(int(v['xmin']), int(v['xmax']) + 1, 10):
        strong = val % 50 == 0
        o.append(f'<line x1="{sx(v["ymax"]):.1f}" y1="{sy(val):.1f}" x2="{sx(v["ymin"]):.1f}" '
                 f'y2="{sy(val):.1f}" stroke="{t.BORDER_CARD if strong else t.DIVIDER}" '
                 f'opacity="{0.9 if strong else 0.55}"/>')
    for val in range(int(v['ymin']), int(v['ymax']) + 1, 10):
        strong = val % 50 == 0
        o.append(f'<line x1="{sx(val):.1f}" y1="{sy(v["xmax"]):.1f}" x2="{sx(val):.1f}" '
                 f'y2="{sy(v["xmin"]):.1f}" stroke="{t.BORDER_CARD if strong else t.DIVIDER}" '
                 f'opacity="{0.9 if strong else 0.55}"/>')
    zw, zh = (v['ymax'] - v['ymin']) * s, (v['xmax'] - v['xmin']) * s
    o.append(f'<image href="{_zone_data_uri(d1, d2)}" x="{sx(v["ymax"]):.1f}" '
             f'y="{sy(v["xmax"]):.1f}" width="{zw:.1f}" height="{zh:.1f}" '
             f'preserveAspectRatio="none"/>')
    font = f'font-family=\'{wt.MONO}\' font-size="11" fill="{t.TEXT_MUTED}"'
    for val in range(-50, int(v['xmax']) + 1, 50):
        o.append(f'<text x="{sx(v["ymax"]) - 5:.1f}" y="{sy(val) + 4:.1f}" text-anchor="end" '
                 f'{font}>{val}</text>')
    for val in range(-200, 201, 100):
        o.append(f'<text x="{sx(val):.1f}" y="{sy(v["xmin"]) + 15:.1f}" text-anchor="middle" '
                 f'{font}>{val}</text>')
    reach = d1 + d2 + 20
    for lim in JOINT_LIMITS['th1']:
        a = math.radians(lim)
        o.append(f'<line x1="{sx(0):.1f}" y1="{sy(0):.1f}" x2="{sx(reach * math.sin(a)):.1f}" '
                 f'y2="{sy(reach * math.cos(a)):.1f}" stroke="{t.TEXT_MUTED}" '
                 f'stroke-dasharray="4 4" opacity=".6"/>')
    o.append(f'<line x1="{sx(0):.1f}" y1="{sy(0):.1f}" x2="{sx(0):.1f}" y2="{sy(228):.1f}" '
             f'stroke="{t.TEXT}" stroke-width="1.5"/>')
    o.append(f'<line x1="{sx(0):.1f}" y1="{sy(0):.1f}" x2="{sx(228):.1f}" y2="{sy(0):.1f}" '
             f'stroke="{t.TEXT}" stroke-width="1.5"/>')
    lbl = f'font-family=\'{wt.MONO}\' font-size="13" font-weight="700" fill="{t.TEXT}"'
    o.append(f'<text x="{sx(0) + 6:.1f}" y="{sy(230) + 4:.1f}" {lbl}>x₀</text>')
    o.append(f'<text x="{sx(232):.1f}" y="{sy(0) - 8:.1f}" text-anchor="end" {lbl}>y₀</text>')
    o.append(f'<clipPath id="planclip"><rect x="{sx(v["ymax"]):.1f}" y="{sy(v["xmax"]):.1f}" '
             f'width="{zw:.1f}" height="{zh:.1f}"/></clipPath>')
    o.append(f'<rect x="{sx(45):.1f}" y="{sy(35):.1f}" width="{90 * s:.1f}" '
             f'height="{160 * s:.1f}" fill="none" stroke="{t.TEXT_MUTED}" opacity=".7" '
             f'clip-path="url(#planclip)"/>')
    return _svg(''.join(o))


def plan_overlay_svg(ref, real, target, d1, d2, hint):
    """ref, real: (th1, th2) en grados o None. target: (x, y) o None."""
    s, sx, sy, _, _ = _plan_transform()
    width = max(8.0, 12 * s)
    o = []

    def arm(q, color, w, opacity, dash=None):
        ex, ey, x, y = fk_full(q[0], q[1], d1, d2)
        d = f' stroke-dasharray="{dash}"' if dash else ''
        o.append(f'<polyline points="{sx(0):.1f},{sy(0):.1f} {sx(ey):.1f},{sy(ex):.1f} '
                 f'{sx(y):.1f},{sy(x):.1f}" fill="none" stroke="{color}" stroke-width="{w}" '
                 f'stroke-linecap="round" stroke-linejoin="round" opacity="{opacity}"{d}/>')
        return (sx(ey), sy(ex)), (sx(y), sy(x))

    if real is not None:
        elbow, tip = arm(real, t.TWIN_REAL, width, 1)
        for cx, cy in ((sx(0), sy(0)), elbow):
            o.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="4.5" fill="{t.BG_CARD}" '
                     f'stroke="{t.TEXT}" stroke-width="1.5"/>')
        o.append(f'<circle cx="{tip[0]:.1f}" cy="{tip[1]:.1f}" r="5" fill="{t.TEXT_STRONG}"/>')
    if ref is not None:
        arm(ref, t.ACCENT, width, 0.38)
        elbow, tip = arm(ref, t.ACCENT, 1.6, 1, '5 4')
        o.append(f'<circle cx="{elbow[0]:.1f}" cy="{elbow[1]:.1f}" r="4" fill="none" '
                 f'stroke="{t.ACCENT}" stroke-width="1.5"/>')
        o.append(f'<circle cx="{tip[0]:.1f}" cy="{tip[1]:.1f}" r="6" fill="none" '
                 f'stroke="{t.ACCENT}" stroke-width="2"/>')
    if target is not None:
        X, Y = sx(target[1]), sy(target[0])
        color = t.TEXT_STRONG if ref is not None else t.DANGER
        o.append(f'<path d="M{X - 9:.1f} {Y:.1f}H{X + 9:.1f}M{X:.1f} {Y - 9:.1f}V{Y + 9:.1f}" '
                 f'stroke="{color}" stroke-width="1.8"/>')
    if hint:
        o.append(f'<text x="{PLAN_W - 14}" y="22" text-anchor="end" '
                 f'font-family=\'{wt.SANS}\' font-size="12" fill="{t.TEXT_MUTED}">'
                 f'Toca la planta para poner X, Y</text>')
    return _svg(''.join(o))


def zbar_svg(height, z_ref, z_real):
    top, bot = 22, height - 22
    def zy(z): return bot - (z - Z_LO) / (Z_HI - Z_LO) * (bot - top)
    cx = ZBAR_W / 2 - 10
    f = f'font-family=\'{wt.MONO}\' font-size="11"'
    o = [f'<svg viewBox="0 0 {ZBAR_W} {height}" preserveAspectRatio="xMidYMid meet" '
         f'xmlns="http://www.w3.org/2000/svg" class="plan-svg">',
         f'<rect x="{cx - 5}" y="{top}" width="10" height="{bot - top}" rx="5" fill="{t.DIVIDER}"/>']
    for z in (Z_LO, 50, 75, 100, Z_HI):
        o.append(f'<line x1="{cx + 8}" y1="{zy(z):.1f}" x2="{cx + 13}" y2="{zy(z):.1f}" '
                 f'stroke="{t.BORDER_CARD}"/>')
        o.append(f'<text x="{cx + 16}" y="{zy(z) + 4:.1f}" {f} fill="{t.TEXT_MUTED}">{z:g}</text>')
    if z_real is not None:
        zr = min(max(z_real, Z_LO), Z_HI)
        o.append(f'<rect x="{cx - 5}" y="{zy(zr):.1f}" width="10" height="{bot - zy(zr):.1f}" '
                 f'rx="5" fill="{t.TWIN_REAL}"/>')
        o.append(f'<line x1="{cx - 9}" y1="{zy(zr):.1f}" x2="{cx + 9}" y2="{zy(zr):.1f}" '
                 f'stroke="{t.TEXT_STRONG}" stroke-width="2.5"/>')
    if z_ref is not None:
        y = zy(min(max(z_ref, Z_LO), Z_HI))
        o.append(f'<path d="M{cx - 20} {y - 6:.1f}L{cx - 9} {y:.1f}L{cx - 20} {y + 6:.1f}Z" '
                 f'fill="{t.ACCENT}"/>')
        o.append(f'<line x1="{cx - 9}" y1="{y:.1f}" x2="{cx + 9}" y2="{y:.1f}" '
                 f'stroke="{t.ACCENT}" stroke-width="2" stroke-dasharray="3 2"/>')
    o.append(f'<text x="{cx}" y="12" text-anchor="middle" font-family=\'{wt.SANS}\' '
             f'font-size="10" font-weight="700" letter-spacing="1" fill="{t.TEXT_MUTED}">z mm</text>')
    o.append('</svg>')
    return ''.join(o)


# JS: toque en la planta -> coordenadas del viewBox (con el letterbox de "meet")
_PLAN_CLICK_JS = f"""(e) => {{
  const r = e.currentTarget.getBoundingClientRect();
  const k = Math.min(r.width / {PLAN_W}, r.height / {PLAN_H});
  const ox = (r.width - {PLAN_W} * k) / 2, oy = (r.height - {PLAN_H} * k) / 2;
  emit({{px: (e.clientX - r.left - ox) / k, py: (e.clientY - r.top - oy) / k}});
}}"""


class Robot3D:
    """Un robot en la escena 3D (mismas piezas que el HTML). Se mueve rotando grupos."""

    def __init__(self, scene, d1, d2, color, opacity):
        self.d1, self.d2 = d1, d2

        def mat(obj):
            return obj.material(color, opacity)

        def cyl(r, h):
            return mat(scene.cylinder(r, r, h, 28).rotate(math.pi / 2, 0, 0))
        with scene.group().move(0, 0, D3_BASE) as self.g1:
            mat(scene.box(d1, 60, 30).move(d1 / 2, 0, 15))
            cyl(30, 30).move(0, 0, 15)
            cyl(30, 30).move(d1, 0, 15)
            cyl(18, 72).move(56.5, 0, 66)
            with scene.group().move(d1, 0, 30) as self.g2:
                mat(scene.box(d2, 55, 30).move(d2 / 2, 0, 15))
                cyl(27.5, 30).move(0, 0, 15)
                cyl(27.5, 30).move(d2, 0, 15)
                cyl(18, 55).move(14.5, 0, 57.5)
                with scene.group().move(d2, 0, 0) as self.tool:
                    mat(scene.box(40, 34, 18).move(-12, 0, 13))
                    cyl(2.2, 10).move(0, 0, 5)
                    for xr in (0, -33.35):
                        cyl(4.25, 135).move(xr, 0, 22 + 67.5)
                    mat(scene.box(48, 24, 6).move(-16.7, 0, 160))
                    scene.sphere(5 if opacity < 1 else 4.5).material(
                        t.ACCENT if opacity < 1 else t.TEXT_STRONG)
        self._last = None

    def set_pose(self, th1, th2, z):
        if self._last is not None and all(abs(a - b) < SCENE_MIN_DELTA
                                          for a, b in zip(self._last, (th1, th2, z))):
            return
        self._last = (th1, th2, z)
        self.g1.rotate(0, 0, math.radians(th1))
        self.g2.rotate(0, 0, math.radians(th2))
        self.tool.move(self.d2, 0, z - Z_TOP)

    def show(self, visible):
        for g in (self.g1,):
            g.visible(visible)


class Twin:
    """Panel "Gemelo". on_plan_click(x_mm, y_mm): toque en la planta (solo Punto a punto)."""

    def __init__(self, d1, d2, on_plan_click=None):
        self.d1, self.d2 = d1, d2
        self._ref = None            # (th1, th2) o None
        self._z_ref = None
        self._real = None           # (th1, th2) o None
        self._z_real = None
        self._target = None
        self._hint = on_plan_click is not None
        self._on_plan_click = on_plan_click
        self._view = 'plan'

        with ui.element('div').classes('twin'):
            with ui.element('div').classes('twin-head'):
                ui.label('GEMELO').classes('caps')
                with ui.element('div').classes('segmented small'):
                    self._btn_plan = (ui.button('Planta', on_click=lambda: self.set_view('plan'),
                                                color=None)
                                      .props('unelevated no-caps').classes('segment on'))
                    self._btn_3d = (ui.button('3D', on_click=lambda: self.set_view('3d'),
                                              color=None)
                                    .props('unelevated no-caps').classes('segment'))
            with ui.element('div').classes('twin-body'):
                with ui.element('div').classes('plan-wrap') as self._plan_wrap:
                    ui.html(plan_background_svg(d1, d2), sanitize=False).classes('plan-layer')
                    self._overlay = ui.html('', sanitize=False).classes('plan-layer')
                if on_plan_click is not None:
                    self._plan_wrap.on('click', self._plan_clicked, js_handler=_PLAN_CLICK_JS)
                with ui.element('div').classes('scene-wrap') as self._scene_wrap:
                    self._build_scene()
                self._scene_wrap.set_visibility(False)
                self._zbar = ui.html('', sanitize=False).classes('zbar')
            with ui.element('div').classes('legend'):
                ui.html(f'<span><i class="sw" style="background:{t.ACCENT}"></i>consigna</span>'
                        f'<span><i class="sw" style="background:{t.TWIN_REAL}"></i>'
                        f'robot real (encoders)</span>'
                        f'<span><i class="sw sw-zone"></i>zona alcanzable</span>'
                        f'<span><i class="sw" style="background:{t.CONNECTED}"></i>'
                        f'|θ₂| &lt; {SINGULAR_DEG:g}°</span>', sanitize=False).classes('legend-row')
        self._render()

    # ---------------- 3D ----------------
    def _build_scene(self):
        sc = ui.scene(width=600, height=440, grid=False, background_color=t.BG_PAGE,
                      camera=ui.scene.perspective_camera(fov=38, near=1, far=6000))
        sc.classes('scene3d')
        with sc:
            for v in range(-240, 241, 20):
                c = t.BORDER_CARD if v % 100 == 0 else t.DIVIDER
                sc.line([v, -240, 0], [v, 240, 0]).material(c)
                sc.line([-240, v, 0], [240, v, 0]).material(c)
            for end, name in (([250, 0, 0], 'x₀'), ([0, 250, 0], 'y₀'), ([0, 0, 250], 'z₀')):
                sc.line([0, 0, 0], end).material(t.TEXT)
                sc.text(name, f'color:{t.TEXT};font:600 13px monospace').move(
                    *[c * 1.05 for c in end])
            pts, cols = [], []
            acc = [c / 255 for c in hex_rgb(t.ACCENT)]
            warn = [c / 255 for c in hex_rgb(t.CONNECTED)]
            (a_lo, a_hi), (b_lo, b_hi) = JOINT_LIMITS['th1'], JOINT_LIMITS['th2']
            a = a_lo
            while a <= a_hi + 1e-6:
                b = b_lo
                while b <= b_hi + 1e-6:
                    _, _, x, y = fk_full(a, b, self.d1, self.d2)
                    for z in (Z_LO, Z_HI):
                        pts.append([x, y, z])
                        cols.append(warn if abs(b) < SINGULAR_DEG else acc)
                    b += 3
                a += 3
            sc.point_cloud(pts, cols, point_size=2.2)
            sc.box(160, 90, D3_BASE).move(-45, 0, D3_BASE / 2).material('#3a4470', .35)
            self._robot_real = Robot3D(sc, self.d1, self.d2, t.TWIN_REAL_3D, 1)
            self._robot_ref = Robot3D(sc, self.d1, self.d2, t.ACCENT, .42)
        sc.move_camera(-470, -560, 520, 60, 10, 80, 0, 0, 1, duration=0)
        self._scene = sc

    def set_view(self, view):
        self._view = view
        is3d = view == '3d'
        self._plan_wrap.set_visibility(not is3d)
        self._scene_wrap.set_visibility(is3d)
        self._btn_plan.classes(remove='on') if is3d else self._btn_plan.classes(add='on')
        self._btn_3d.classes(add='on') if is3d else self._btn_3d.classes(remove='on')
        if is3d:   # el canvas estaba oculto: que tome el tamano del panel
            ui.run_javascript('setTimeout(() => window.dispatchEvent(new Event("resize")), 60)')

    # ---------------- Datos ----------------
    def set_ref(self, q, z, target=None):
        """q: (th1, th2) en grados o None (consigna invalida). z en mm."""
        self._ref, self._z_ref, self._target = q, z, target
        self._render()

    def set_real(self, q, z):
        """q: (th1, th2) o None (sin homing). z en mm o None."""
        if q == self._real and z == self._z_real:
            return
        self._real, self._z_real = q, z
        self._render()

    def _render(self):
        self._overlay.set_content(plan_overlay_svg(self._ref, self._real, self._target,
                                                   self.d1, self.d2, self._hint))
        self._zbar.set_content(zbar_svg(PLAN_H, self._z_ref, self._z_real))
        self._robot_ref.show(self._ref is not None)
        if self._ref is not None:
            self._robot_ref.set_pose(self._ref[0], self._ref[1], self._z_ref)
        self._robot_real.show(self._real is not None)
        if self._real is not None:
            self._robot_real.set_pose(self._real[0], self._real[1], self._z_real)

    def _plan_clicked(self, e):
        if self._on_plan_click is None or not e.args:
            return
        x, y = plan_to_mm(e.args['px'], e.args['py'])
        self._on_plan_click(round(x, 1), round(y, 1))
