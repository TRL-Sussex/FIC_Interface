"""Robot Config, MCU section: the Cartesian-routing option is gone (inner-loop split, FIC-link v3).

Cartesian modes always pass their torque through the inner loop, so the section no longer offers or
saves `cartesian_routing`, and a value left in an older robots_config.yaml is dropped on save.
"""
from types import SimpleNamespace

from fic_studio.mixins.robot_config_mixin import RobotConfigMixin


class _Val:
    def __init__(self, v):
        self._v = v

    def value(self):
        return self._v

    def text(self):
        return self._v

    def currentData(self):
        return self._v


def _panel(base):
    return SimpleNamespace(
        _rc_mcu_adapter=_Val('kortex'), _rc_mcu_hold=_Val(0.05), _rc_mcu_ramp=_Val(2.0),
        _rc_mcu_arm_ip=_Val('192.168.1.10'), _rc_mcu_setup=_Val(''), _rc_mcu_base=base)


def test_collect_drops_cartesian_routing():
    mcu = RobotConfigMixin._rc_collect_mcu(_panel({'adapter': 'kortex', 'cartesian_routing': 'haptic',
                                                    'arm_port': 10001}))
    assert 'cartesian_routing' not in mcu
    assert mcu['adapter'] == 'kortex' and mcu['arm_port'] == 10001 and mcu['arm_ip'] == '192.168.1.10'


def test_section_has_no_routing_widget():
    import inspect
    src = inspect.getsource(RobotConfigMixin)
    assert '_rc_mcu_routing' not in src and 'Cartesian routing' not in src
