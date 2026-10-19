"""
twin_widget.py - Gemelo del robot en la interfaz de escritorio (solo planta 2D).

Igual a la planta de la version web (web/twin.py) y del HTML "Espacio de
Trabajo SCARA": x0 hacia arriba, y0 hacia la izquierda, malla de 10 mm,
zona alcanzable sombreada, consigna en cian translucido, robot real en gris
solido y barra de z al lado. El 3D esta solo en la version web.

Uso:
    twin = TwinWidget(d1, d2, on_plan_click=None)
    twin.set_ref((th1, th2) | None, z | None, target=(x, y) | None)
    twin.set_real((th1, th2) | None, z | None)
"""
import math

from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QColor, QImage, QPainter, QPainterPath, QPen, QPolygonF
from PyQt5.QtWidgets import QHBoxLayout, QLabel, QVBoxLayout, QWidget

from scara_app.robot_model import (
    JOINT_LIMITS, PLAN_VIEW, SINGULAR_DEG, fk_full, hex_rgb, zone_rgba,
)

from . import theme

Z_LO, Z_HI = JOINT_LIMITS['z']
ZONE_PX_PER_MM = 2


def _color(hex_color, alpha=1.0):
    c = QColor(hex_color)
    c.setAlphaF(alpha)
    return c


class PlanView(QWidget):
    """Vista en planta. Se escala al tamano del widget (sin deformar)."""

    MARGIN = {'l': 34, 'r': 10, 't': 10, 'b': 24}

    def __init__(self, d1, d2, on_click=None, parent=None):
        super().__init__(parent)
        self.d1, self.d2 = d1, d2
        self.ref = self.real = self.target = None
        self.on_click = on_click
        self.setMinimumSize(360, 280)
        if on_click is not None:
            self.setCursor(Qt.CrossCursor)
        img = zone_rgba(ZONE_PX_PER_MM, d1, d2, hex_rgb(theme.ACCENT), hex_rgb(theme.CONNECTED))
        h, w = img.shape[:2]
        self._zone_bytes = img.tobytes()      # QImage no copia: guardar la referencia
        self._zone = QImage(self._zone_bytes, w, h, 4 * w, QImage.Format_RGBA8888)

    # ---------------- Transformacion mm <-> pixeles ----------------
    def _transform(self):
        v, m = PLAN_VIEW, self.MARGIN
        W, H = self.width(), self.height()
        s = min((W - m['l'] - m['r']) / (v['ymax'] - v['ymin']),
                (H - m['t'] - m['b']) / (v['xmax'] - v['xmin']))
        ox = m['l'] + ((W - m['l'] - m['r']) - (v['ymax'] - v['ymin']) * s) / 2
        oy = m['t'] + ((H - m['t'] - m['b']) - (v['xmax'] - v['xmin']) * s) / 2
        return s, ox, oy

    def _pt(self, x, y):
        s, ox, oy = self._transform()
        return QPointF(ox + (PLAN_VIEW['ymax'] - y) * s, oy + (PLAN_VIEW['xmax'] - x) * s)

    def mousePressEvent(self, event):
        if self.on_click is None or event.button() != Qt.LeftButton:
            return
        s, ox, oy = self._transform()
        x = PLAN_VIEW['xmax'] - (event.pos().y() - oy) / s
        y = PLAN_VIEW['ymax'] - (event.pos().x() - ox) / s
        self.on_click(round(x, 1), round(y, 1))

    # ---------------- Dibujo ----------------
    def paintEvent(self, _event):
        v = PLAN_VIEW
        s, _, _ = self._transform()
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor(theme.BG_PAGE))

        # Malla: 10 mm (tenue) y 50 mm (marcada)
        for val in range(int(v['xmin']), int(v['xmax']) + 1, 10):
            strong = val % 50 == 0
            p.setPen(QPen(_color(theme.BORDER_CARD if strong else theme.DIVIDER,
                                 0.9 if strong else 0.55), 1))
            p.drawLine(self._pt(val, v['ymax']), self._pt(val, v['ymin']))
        for val in range(int(v['ymin']), int(v['ymax']) + 1, 10):
            strong = val % 50 == 0
            p.setPen(QPen(_color(theme.BORDER_CARD if strong else theme.DIVIDER,
                                 0.9 if strong else 0.55), 1))
            p.drawLine(self._pt(v['xmax'], val), self._pt(v['xmin'], val))

        # Zona alcanzable
        area = QRectF(self._pt(v['xmax'], v['ymax']), self._pt(v['xmin'], v['ymin']))
        p.drawImage(area, self._zone)

        # Cotas
        p.setFont(theme.mono(8))
        p.setPen(QColor(theme.TEXT_MUTED))
        for val in range(-50, int(v['xmax']) + 1, 50):
            pt = self._pt(val, v['ymax'])
            p.drawText(QRectF(pt.x() - 40, pt.y() - 8, 35, 16), Qt.AlignRight | Qt.AlignVCenter,
                       str(val))
        for val in range(-200, 201, 100):
            pt = self._pt(v['xmin'], val)
            p.drawText(QRectF(pt.x() - 25, pt.y() + 3, 50, 16), Qt.AlignHCenter, str(val))

        # Limites de th1
        reach = self.d1 + self.d2 + 20
        p.setPen(QPen(_color(theme.TEXT_MUTED, 0.6), 1, Qt.DashLine))
        for lim in JOINT_LIMITS['th1']:
            a = math.radians(lim)
            p.drawLine(self._pt(0, 0), self._pt(reach * math.cos(a), reach * math.sin(a)))

        # Ejes
        p.setPen(QPen(QColor(theme.TEXT), 1.5))
        p.drawLine(self._pt(0, 0), self._pt(228, 0))
        p.drawLine(self._pt(0, 0), self._pt(0, 228))
        p.setFont(theme.mono(10, bold=True))
        p.drawText(self._pt(230, 0) + QPointF(6, 4), 'x₀')
        p.drawText(self._pt(0, 232) + QPointF(-4, -8), 'y₀')

        # Huella de la base (recortada al area)
        p.save()
        p.setClipRect(area)
        p.setPen(QPen(_color(theme.TEXT_MUTED, 0.7), 1))
        p.setBrush(Qt.NoBrush)
        p.drawRect(QRectF(self._pt(35, 45), self._pt(-125, -45)))
        p.restore()

        width = max(8.0, 12 * s)
        if self.real is not None:
            elbow, tip = self._arm(p, self.real, QColor(theme.TWIN_REAL), width)
            p.setPen(QPen(QColor(theme.TEXT), 1.5))
            p.setBrush(QColor(theme.BG_CARD))
            for c in (self._pt(0, 0), elbow):
                p.drawEllipse(c, 4.5, 4.5)
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(theme.TEXT_STRONG))
            p.drawEllipse(tip, 5, 5)
        if self.ref is not None:
            self._arm(p, self.ref, _color(theme.ACCENT, 0.38), width)
            elbow, tip = self._arm(p, self.ref, QColor(theme.ACCENT), 1.6, dashed=True)
            p.setBrush(Qt.NoBrush)
            p.setPen(QPen(QColor(theme.ACCENT), 1.5))
            p.drawEllipse(elbow, 4, 4)
            p.setPen(QPen(QColor(theme.ACCENT), 2))
            p.drawEllipse(tip, 6, 6)
        if self.target is not None:
            c = self._pt(*self.target)
            p.setPen(QPen(QColor(theme.TEXT_STRONG if self.ref is not None else theme.DANGER), 1.8))
            p.drawLine(c + QPointF(-9, 0), c + QPointF(9, 0))
            p.drawLine(c + QPointF(0, -9), c + QPointF(0, 9))
        if self.on_click is not None:
            p.setFont(theme.sans(9))
            p.setPen(QColor(theme.TEXT_MUTED))
            p.drawText(QRectF(0, 6, self.width() - 14, 18), Qt.AlignRight,
                       'Clic en la planta para poner X, Y')
        p.end()

    def _arm(self, p, q, color, width, dashed=False):
        ex, ey, x, y = fk_full(q[0], q[1], self.d1, self.d2)
        pen = QPen(color, width, Qt.DashLine if dashed else Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin)
        if dashed:
            pen.setDashPattern([3, 2.5])
        p.setPen(pen)
        p.setBrush(Qt.NoBrush)
        elbow, tip = self._pt(ex, ey), self._pt(x, y)
        p.drawPolyline(QPolygonF([self._pt(0, 0), elbow, tip]))
        return elbow, tip


class ZBar(QWidget):
    """Barra de z (19.3 .. 126 mm): triangulo cian = consigna, barra gris = real."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.z_ref = self.z_real = None
        self.setFixedWidth(66)

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor(theme.BG_PAGE))
        p.setPen(QPen(QColor(theme.DIVIDER), 1))
        p.drawLine(0, 0, 0, self.height())
        top, bot = 26.0, self.height() - 22.0
        cx = self.width() / 2 - 10

        def zy(z):
            return bot - (min(max(z, Z_LO), Z_HI) - Z_LO) / (Z_HI - Z_LO) * (bot - top)
        p.setFont(theme.caps_label_font(6))
        p.setPen(QColor(theme.TEXT_MUTED))
        p.drawText(QRectF(0, 4, self.width(), 14), Qt.AlignHCenter, 'z mm')
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(theme.DIVIDER))
        p.drawRoundedRect(QRectF(cx - 5, top, 10, bot - top), 5, 5)
        p.setFont(theme.mono(7))
        for z in (Z_LO, 50, 75, 100, Z_HI):
            p.setPen(QColor(theme.BORDER_CARD))
            p.drawLine(QPointF(cx + 8, zy(z)), QPointF(cx + 13, zy(z)))
            p.setPen(QColor(theme.TEXT_MUTED))
            p.drawText(QPointF(cx + 15, zy(z) + 4), f'{z:g}')
        if self.z_real is not None:
            y = zy(self.z_real)
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(theme.TWIN_REAL))
            p.drawRoundedRect(QRectF(cx - 5, y, 10, bot - y), 5, 5)
            p.setPen(QPen(QColor(theme.TEXT_STRONG), 2.5))
            p.drawLine(QPointF(cx - 9, y), QPointF(cx + 9, y))
        if self.z_ref is not None:
            y = zy(self.z_ref)
            path = QPainterPath()
            path.moveTo(cx - 20, y - 6)
            path.lineTo(cx - 9, y)
            path.lineTo(cx - 20, y + 6)
            path.closeSubpath()
            p.fillPath(path, QColor(theme.ACCENT))
            pen = QPen(QColor(theme.ACCENT), 2, Qt.DashLine)
            pen.setDashPattern([1.5, 1])
            p.setPen(pen)
            p.drawLine(QPointF(cx - 9, y), QPointF(cx + 9, y))
        p.end()


class TwinWidget(QWidget):
    """Panel "Gemelo": titulo, planta + barra de z y leyenda."""

    def __init__(self, d1, d2, on_plan_click=None, parent=None):
        super().__init__(parent)
        self.setObjectName('panel')
        self.setAttribute(Qt.WA_StyledBackground, True)
        title = QLabel('GEMELO')
        title.setObjectName('capsLabel')
        title.setFont(theme.caps_label_font(7))
        head = QHBoxLayout()
        head.setContentsMargins(14, 10, 14, 10)
        head.addWidget(title)
        head.addStretch()

        self.plan = PlanView(d1, d2, on_plan_click)
        self.zbar = ZBar()
        body = QHBoxLayout()
        body.setContentsMargins(0, 0, 0, 0)
        body.setSpacing(0)
        body.addWidget(self.plan, stretch=1)
        body.addWidget(self.zbar)

        legend = QLabel(
            f'<span style="color:{theme.ACCENT}">━━</span>&nbsp;consigna&nbsp;&nbsp;&nbsp;'
            f'<span style="color:{theme.TWIN_REAL}">━━</span>&nbsp;robot real (encoders)&nbsp;&nbsp;&nbsp;'
            f'<span style="color:{theme.ACCENT}">■</span>&nbsp;zona alcanzable&nbsp;&nbsp;&nbsp;'
            f'<span style="color:{theme.CONNECTED}">━━</span>&nbsp;|θ₂| &lt; {SINGULAR_DEG:g}°')
        legend.setFont(theme.sans(9))
        legend.setStyleSheet(f'color:{theme.TEXT_MUTED}; padding: 6px 14px;')

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        lay.addLayout(head)
        lay.addWidget(self._line())
        lay.addLayout(body, stretch=1)
        lay.addWidget(self._line())
        lay.addWidget(legend)

    @staticmethod
    def _line():
        line = QWidget()
        line.setFixedHeight(1)
        line.setStyleSheet(f'background:{theme.DIVIDER};')
        return line

    def set_ref(self, q, z, target=None):
        self.plan.ref, self.plan.target = q, target
        self.zbar.z_ref = z
        self.plan.update()
        self.zbar.update()

    def set_real(self, q, z):
        if q == self.plan.real and z == self.zbar.z_real:
            return
        self.plan.real = q
        self.zbar.z_real = z
        self.plan.update()
        self.zbar.update()
