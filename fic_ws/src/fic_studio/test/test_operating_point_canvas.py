"""
Tests for FICCanvas live operating-point state (set / clear / bounded trail).

Offscreen-Qt: a valid point populates _op_point and appends to the bounded
_op_trail; an invalid point or clear() empties both.
"""
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from fic_studio.mixins.tuning_mixin import FICCanvas  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402

_app = QApplication.instance() or QApplication([])


def test_valid_point_sets_state_and_trail():
    c = FICCanvas()
    c.set_operating_point(0.02, 12.0, True)
    assert c._op_point == (0.02, 12.0)
    assert list(c._op_trail) == [(0.02, 12.0)]


def test_invalid_point_clears_state_and_trail():
    c = FICCanvas()
    c.set_operating_point(0.02, 12.0, True)
    c.set_operating_point(0.0, 0.0, False)
    assert c._op_point is None
    assert len(c._op_trail) == 0


def test_clear_empties_state_and_trail():
    c = FICCanvas()
    c.set_operating_point(0.02, 12.0, True)
    c.clear_operating_point()
    assert c._op_point is None
    assert len(c._op_trail) == 0


def test_trail_is_bounded():
    c = FICCanvas()
    for i in range(FICCanvas.TRAIL_LEN + 10):
        c.set_operating_point(0.001 * i, float(i), True)
    assert len(c._op_trail) == FICCanvas.TRAIL_LEN


def test_paint_does_not_raise_in_any_state():
    from PyQt5.QtGui import QPixmap
    c = FICCanvas()
    c.resize(420, 320)
    c.grab()
    c.set_operating_point(0.03, 10.0, True)
    c.grab()
    c.set_operating_point(99.0, 9999.0, True)
    assert isinstance(c.grab(), QPixmap)


def test_op_colors_present_after_paint():
    c = FICCanvas()
    c.resize(420, 320)
    c.grab()
    assert hasattr(c, 'COL_OP') and hasattr(c, 'COL_DEV_FAR')
