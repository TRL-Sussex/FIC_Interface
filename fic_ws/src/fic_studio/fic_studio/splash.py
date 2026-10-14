# splash.py — startup splash for FIC Studio entry points
#
# Cold start: ~250 ms of Python+Qt setup runs before QApplication exists, so
# the splash can't cover that early window. It does cover the dominant
# ~1.5-2 s of ROS imports + UI build, which is what users perceive as the
# "long delay" between launcher click and window paint.
#
# Style matches the Warmth (warmth_dark) theme: warm #1B1714 surface,
# #3A312A border, #C56A42 terracotta accent bar, #EDE6D9 text.
# Font stack mirrors the GUI ('Segoe UI', 'Ubuntu', 'Roboto').

from PyQt5.QtCore import Qt
from PyQt5.QtGui import (
    QPixmap, QPainter, QColor, QFont, QPen, QBrush,
)
from PyQt5.QtWidgets import QApplication, QSplashScreen


# Warmth (warmth_dark) surface, so the splash and the main window read
# as the same warm surface — no jarring colour shift on hand-off.
_BG       = QColor('#1B1714')   # bg_base
_BG_INSET = QColor('#241E19')   # bg_raised
_BORDER   = QColor('#3A312A')   # border_subtle
_ACCENT   = QColor('#C56A42')   # terracotta accent
_TEXT     = QColor('#EDE6D9')   # text_primary
_DIM      = QColor('#B3A897')   # text_secondary

_W, _H = 480, 260
_FONT_FAMILY = 'Segoe UI'   # main GUI uses 'Segoe UI', 'Ubuntu', 'Roboto' fallback
_FONT_FALLBACK = ['SF Pro Text', 'Helvetica Neue', 'Ubuntu', 'Roboto', 'Sans Serif']


def _font(size: int, bold: bool = False) -> QFont:
    f = QFont(_FONT_FAMILY, size)
    f.setBold(bold)
    f.setHintingPreference(QFont.PreferFullHinting)
    f.insertSubstitutions(_FONT_FAMILY, _FONT_FALLBACK)
    return f


def _build_pixmap(title: str, version: str) -> QPixmap:
    pm = QPixmap(_W, _H)
    pm.fill(Qt.transparent)
    p = QPainter(pm)
    p.setRenderHint(QPainter.Antialiasing)

    # Flat dark surface with a subtle 1px border — matches GroupBox / dock
    # panel styling in DARK_THEME.
    p.setBrush(QBrush(_BG))
    p.setPen(QPen(_BORDER, 1))
    p.drawRoundedRect(0, 0, _W - 1, _H - 1, 6, 6)

    # Inset card behind the title — gives the splash some depth without a
    # gradient, matching the bg_raised treatment used in the main GUI.
    inset_x, inset_y, inset_w, inset_h = 24, 36, _W - 48, 130
    p.setBrush(QBrush(_BG_INSET))
    p.setPen(QPen(_BORDER, 1))
    p.drawRoundedRect(inset_x, inset_y, inset_w, inset_h, 4, 4)

    # Title
    p.setPen(_TEXT)
    p.setFont(_font(22, bold=True))
    p.drawText(inset_x, inset_y + 28, inset_w, 50, Qt.AlignHCenter | Qt.AlignTop, title)

    # Version line
    p.setPen(_DIM)
    p.setFont(_font(10))
    p.drawText(inset_x, inset_y + 78, inset_w, 20, Qt.AlignHCenter | Qt.AlignTop, version)

    # Accent bar — single decisive terracotta line, the same accent used for
    # primary buttons / selected sidebar entries in the GUI.
    bar_y = inset_y + inset_h - 1
    p.setPen(QPen(_ACCENT, 2))
    p.drawLine(inset_x + 24, bar_y, inset_x + inset_w - 24, bar_y)

    # Bottom-row label (static "Fractal Impedance Control") — provides a
    # second line of context while the dynamic status sits below it.
    p.setPen(_DIM)
    p.setFont(_font(9))
    p.drawText(0, _H - 56, _W, 18, Qt.AlignHCenter, "Fractal Impedance Control")

    p.end()
    return pm


def make_splash(title: str = "FIC Control Studio",
                version: str = "") -> QSplashScreen:
    pm = _build_pixmap(title, version)
    splash = QSplashScreen(pm, Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)
    splash.setAttribute(Qt.WA_TranslucentBackground)
    splash.setMask(pm.mask())
    # Status messages appear in this font, not the QPainter ones above.
    splash.setFont(_font(10))
    splash.show()
    QApplication.processEvents()
    return splash


def splash_status(splash: QSplashScreen, message: str) -> None:
    splash.showMessage(
        message,
        Qt.AlignBottom | Qt.AlignHCenter,
        _DIM,
    )
    QApplication.processEvents()
