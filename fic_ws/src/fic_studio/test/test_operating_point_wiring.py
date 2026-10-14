"""
Wiring tests: _refresh_feedback_strip pushes the operating point to the canvas.

A light stub provides the telemetry helpers + dropdowns + a recording canvas,
then drives the real TuningMixin._refresh_feedback_strip joint/Cartesian paths.
"""
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from fic_studio.mixins.tuning_mixin import TuningMixin  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402

_app = QApplication.instance() or QApplication([])


class _RecCanvas:
    def __init__(self):
        self.last = None
        self.k0 = 600.0
        self.xbs = [0.05, 0.10]
        self.fs = [5.0, 39.0]
        self.srs = [0.9]

    def set_operating_point(self, x, y, valid):
        self.last = (x, y, valid)

    def clear_operating_point(self):
        self.last = ('cleared',)


class _Combo:
    def __init__(self, text):
        self._t = text

    def currentText(self):
        return self._t


class _Strip:
    def set_error(self, *a):
        pass

    def set_effort(self, *a):
        pass

    def set_wrench(self, *a):
        pass


class _Node:
    station_robot_id = 'robot1'


def _stub(mode):
    s = TuningMixin.__new__(TuningMixin)
    s.fic_canvas = _RecCanvas()
    s._feedback_strip = _Strip()
    s.node = _Node()
    s.stack = None
    s.joint_id_dropdown = _Combo('2')
    s.axis_dropdown = _Combo('x')
    s._refresh_mode = mode
    s.joint_commanded_for = lambda rid=None: [0.0, 0.0, 0.30, 0.0, 0.0, 0.0, 0.0]
    s.joint_positions_for = lambda rid=None: [0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0]
    s.joint_effort_for = lambda rid=None: [0.0, 0.0, 14.0, 0.0, 0.0, 0.0, 0.0]
    s.wrench_for = lambda rid=None: (7.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    s.cartesian_error_for = lambda rid=None: (0.012, 0.0, 0.0, 0.0)
    s._get_current_mode = lambda: s._refresh_mode
    return s


def test_joint_mode_pushes_error_and_effort():
    s = _stub(mode=1)
    s._refresh_feedback_strip()
    x, y, valid = s.fic_canvas.last
    assert valid is True
    assert abs(x - 0.05) < 1e-9
    assert abs(y - 14.0) < 1e-9


def test_cartesian_mode_pushes_axis_error_and_wrench():
    s = _stub(mode=5)
    s._refresh_feedback_strip()
    x, y, valid = s.fic_canvas.last
    assert valid is True
    assert abs(x - 0.012) < 1e-9
    assert abs(y - 7.0) < 1e-9


class _Label:
    def __init__(self):
        self._t = ''

    def setText(self, s):
        self._t = s

    def text(self):
        return self._t


def test_readout_text_updates_in_joint_mode():
    s = _stub(mode=1)
    s._op_readout = _Label()
    s._refresh_feedback_strip()
    txt = s._op_readout.text()
    assert 'F' in txt and '0.05' in txt
