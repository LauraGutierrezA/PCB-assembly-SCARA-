"""
theme.py - Design system de la interfaz (colores, tipografias, estilos).

Todos los colores estan aqui. Para ajustar la estetica, cambia los tokens;
las pestanas no definen colores propios.
"""
from PyQt5.QtGui import QFont, QFontDatabase

# ---------------- Tokens de color ----------------
BG_PAGE      = '#080918'   # fondo de la ventana y del historial
BG_CARD      = '#0a0f2e'   # fondo de la tarjeta principal
BORDER_CARD  = '#2c3a80'   # borde de la tarjeta y de botones secundarios
DIVIDER      = '#1c2554'   # lineas divisorias y borde del historial

TEXT         = '#d8ddec'   # texto principal
TEXT_STRONG  = '#ffffff'   # botones y pestana activa
TEXT_MUTED   = '#8b93ad'   # etiquetas, horas, mensajes de sistema
TEXT_DISABLED = '#6c6f82'

ACCENT       = '#3fe0e8'   # estado actual, subrayado de pestana
PRIMARY      = '#1550f0'   # boton principal (Start)
PRIMARY_HOVER = '#2a63ff'
DANGER       = '#ff4d5e'   # textos de error (ABORTED en el historial)
DANGER_BG    = '#d93a50'   # boton Stop habilitado
DANGER_BG_HOVER = '#ec4a60'
DANGER_BG_OFF = '#6c2842'  # boton Stop deshabilitado
CONNECTED    = '#f0b429'   # punto de conexion activo
DISCONNECTED = DANGER      # punto de conexion inactivo

# ---------------- Tipografias ----------------
_MONO_CANDIDATES = ['JetBrains Mono', 'IBM Plex Mono', 'Fira Code', 'Ubuntu Mono',
                    'DejaVu Sans Mono', 'Monospace']
_SANS_CANDIDATES = ['Inter', 'IBM Plex Sans', 'Ubuntu', 'Noto Sans', 'DejaVu Sans', 'Sans']


def _first_available(candidates):
    installed = set(QFontDatabase().families())
    for name in candidates:
        if name in installed:
            return name
    return candidates[-1]


_mono_family = None
_sans_family = None


def mono_family():
    global _mono_family
    if _mono_family is None:
        _mono_family = _first_available(_MONO_CANDIDATES)
    return _mono_family


def sans_family():
    global _sans_family
    if _sans_family is None:
        _sans_family = _first_available(_SANS_CANDIDATES)
    return _sans_family


def mono(size, bold=False):
    f = QFont(mono_family(), size)
    f.setBold(bold)
    return f


def sans(size, bold=False):
    f = QFont(sans_family(), size)
    f.setBold(bold)
    return f


def caps_label_font(size=8):
    """Etiquetas pequenas en mayusculas con espaciado (ESTADO ACTUAL, HISTORIAL)."""
    f = QFont(sans_family(), size)
    f.setBold(True)
    f.setLetterSpacing(QFont.AbsoluteSpacing, 1.2)
    return f


# ---------------- Hoja de estilos global ----------------
def stylesheet():
    return f"""
    QMainWindow, QWidget#page {{
        background: {BG_PAGE};
    }}
    QWidget {{
        color: {TEXT};
    }}
    QFrame#card {{
        background: {BG_CARD};
        border: 1px solid {BORDER_CARD};
        border-radius: 10px;
    }}
    QStackedWidget, QStackedWidget > QWidget, QWidget#header {{
        background: transparent;
    }}
    QFrame#divider {{
        background: {DIVIDER};
        border: none;
        max-height: 1px;
        min-height: 1px;
    }}
    QLabel#capsLabel {{
        color: {TEXT_MUTED};
    }}
    QLabel#statusValue {{
        color: {ACCENT};
    }}

    /* ---------- Pestanas ---------- */
    QTabWidget::pane {{
        border: none;
        border-top: 1px solid {DIVIDER};
    }}
    QTabBar {{
        qproperty-drawBase: 0;
        background: transparent;
    }}
    QTabBar::tab {{
        background: transparent;
        color: {TEXT_MUTED};
        padding: 12px 14px;
        margin-left: 10px;
        font-size: 13px;
        border: none;
        border-bottom: 2px solid transparent;
        font-weight: bold;
    }}
    QTabBar::tab:selected {{
        color: {TEXT_STRONG};
        border-bottom: 2px solid {ACCENT};
    }}
    QTabBar::tab:hover:!selected {{
        color: {TEXT};
    }}

    /* ---------- Botones ---------- */
    QPushButton {{
        font-weight: bold;
        font-size: 15px;
        border-radius: 4px;
        padding: 14px 24px;
    }}
    QPushButton#primary {{
        background: {PRIMARY};
        color: {TEXT_STRONG};
        border: 1px solid {PRIMARY};
    }}
    QPushButton#primary:hover {{ background: {PRIMARY_HOVER}; }}
    QPushButton#primary:disabled {{
        background: {DIVIDER}; border-color: {DIVIDER}; color: {TEXT_DISABLED};
    }}

    QPushButton#secondary {{
        background: transparent;
        color: {TEXT_STRONG};
        border: 1px solid {BORDER_CARD};
    }}
    QPushButton#secondary:hover {{ border-color: {ACCENT}; }}
    QPushButton#secondary:disabled {{
        border-color: {DIVIDER}; color: {TEXT_DISABLED};
    }}

    QPushButton#danger {{
        background: {DANGER_BG};
        color: {TEXT_STRONG};
        border: 1px solid {DANGER_BG};
    }}
    QPushButton#danger:hover {{ background: {DANGER_BG_HOVER}; }}
    QPushButton#danger:disabled {{
        background: {DANGER_BG_OFF}; border-color: {DANGER_BG_OFF}; color: {TEXT_DISABLED};
    }}

    QPushButton#textButton {{
        background: transparent;
        border: none;
        color: {TEXT};
        font-size: 13px;
        padding: 4px 0px;
    }}
    QPushButton#textButton:hover {{ color: {ACCENT}; }}

    /* ---------- Historial ---------- */
    QTextEdit#log {{
        background: {BG_PAGE};
        border: 1px solid {DIVIDER};
        border-radius: 4px;
        padding: 8px 10px;
    }}
    QScrollBar:vertical {{
        background: transparent; width: 8px; margin: 4px 2px;
    }}
    QScrollBar::handle:vertical {{
        background: {DIVIDER}; border-radius: 3px; min-height: 24px;
    }}
    QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0; }}
    QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {{ background: none; }}
    """
