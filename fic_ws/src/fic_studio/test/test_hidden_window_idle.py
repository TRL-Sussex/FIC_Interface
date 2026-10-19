"""The classic window, hidden behind the web UI, must cost next to nothing.

With the web UI the classic QMainWindow is never shown, yet its periodic
refreshers kept repainting it: the dashboard 20 times a second, the status LEDs
and the toolbar every half second, each through setStyleSheet. On the MCU board
that held the Qt main thread at ~90 % (profiled 19 Oct 2026), and the web UI's
commands, dispatched on the same thread, waited behind it: buttons took seconds.
"""
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication, QStackedWidget, QWidget  # noqa: E402

from fic_studio import theme  # noqa: E402
from fic_studio.components.dash_widgets import StatusRow  # noqa: E402
from fic_studio.pages import PageRegistry  # noqa: E402
from fic_studio.widgets import StatusLED  # noqa: E402

_app = QApplication.instance() or QApplication([])


def _count_restyles(widget):
    """Count setStyleSheet calls on ``widget`` from now on."""
    calls = []
    real = widget.setStyleSheet
    widget.setStyleSheet = lambda qss: (calls.append(qss), real(qss))
    return calls


def test_a_page_in_a_hidden_window_is_not_showing():
    """The refreshers ask is_visible(); a window nobody can see shows nothing."""
    stack = QStackedWidget()
    registry = PageRegistry(stack)
    registry.register('dashboard', QWidget())
    registry.show('dashboard')

    assert registry.is_visible('dashboard') is False      # never shown (the web UI's case)
    stack.show()
    assert registry.is_visible('dashboard') is True
    stack.hide()
    assert registry.is_visible('dashboard') is False


def test_led_is_restyled_only_when_its_status_changes():
    """The status timer sets every LED twice a second; an unchanged one is left alone."""
    led = StatusLED()
    led.set_status('connected')
    calls = _count_restyles(led)
    for _ in range(5):
        led.set_status('connected')
    assert calls == []
    led.set_status('idle')
    assert len(calls) == 1


def test_led_follows_a_theme_change_even_with_the_same_status():
    """The restyle cache is keyed by the theme too."""
    led = StatusLED()
    led.set_status('connected')
    before = led.styleSheet()
    names = [n for n in theme.THEMES if n != theme.current_name()]
    old = theme.current_name()
    try:
        theme._current_name = names[0]
        led.set_status('connected')
        same_green = theme.THEMES[names[0]]['accent_green'] == theme.THEMES[old]['accent_green']
        assert led.styleSheet() != before or same_green
    finally:
        theme._current_name = old


def test_status_row_is_restyled_only_when_ok_changes():
    """Dashboard rows are refreshed 20 times a second; restyle only on OK <-> WARN."""
    row = StatusRow('limits', ok=True)
    calls = _count_restyles(row._status_w)
    for _ in range(5):
        row.set_ok(True)
    assert calls == []
    row.set_ok(False)
    assert len(calls) == 1
    assert row._status_w.text() == 'WARN'


def test_toolbar_refresh_restyles_nothing_when_nothing_changed():
    """_update_status refreshes the toolbar every 500 ms; a quiet refresh must not restyle."""
    from PyQt5.QtWidgets import QPushButton
    from fic_studio.main_window_pro import ProfessionalRobotGUI as G

    class _Bar:
        control_target = 'simulation'
        simulation_running = fic_running = driver_running = False
        controller_running = vr_relay_running = False
        _tb_session_running = G._tb_session_running
        _tb_target_styles = G._tb_target_styles
        _tb_startstop_styles = G._tb_startstop_styles
        _refresh_toolbar_launch = G._refresh_toolbar_launch

        def __init__(self):
            self._tb_target_btns = {k: QPushButton() for k in ('simulation', 'hardware', 'vr')}
            self._tb_start_btn = QPushButton()
            self.robot_dropdown = QPushButton()

    bar = _Bar()
    bar._refresh_toolbar_launch()
    calls = [_count_restyles(b) for b in list(bar._tb_target_btns.values()) + [bar._tb_start_btn]]
    bar._refresh_toolbar_launch()
    assert all(c == [] for c in calls)
    bar.simulation_running = True                        # Start -> Stop: that one restyles
    bar._refresh_toolbar_launch()
    assert len(calls[-1]) == 1 and bar._tb_start_btn.text() == '■ Stop'
