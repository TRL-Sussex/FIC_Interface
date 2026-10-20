"""Keyboard Control in the web UI: the page's side (ui/keys.js), run in Node.

The page reads the drive keys (it knows when someone is typing in one of its
boxes) and sends the ones held down: at once on every change, again every
100 ms while a drive key is down, and an empty list when the last is let go.
"""
import json
from pathlib import Path
import shutil
import subprocess

import pytest

UI_DIR = Path(__file__).resolve().parents[1] / 'fic_studio' / 'web_remote' / 'ui'
NODE = shutil.which('node') or shutil.which('nodejs')

pytestmark = pytest.mark.skipif(NODE is None, reason='needs Node.js')


def _run(body):
    """Run ``body`` with ``K`` = keys.js's exports; it prints JSON, which is returned."""
    script = ("const K = require(%s);\n" % json.dumps(str(UI_DIR / 'keys.js'))) + body
    out = subprocess.run([NODE, '-e', script], capture_output=True, text=True, timeout=20)
    assert out.returncode == 0, out.stderr
    return json.loads(out.stdout)


_TIMERS = """
const sent = [];
let ticker = null;
const timers = { set: (f, ms) => { ticker = { f, ms }; return 1; }, clear: () => { ticker = null; } };
const d = K.Driver((keys, tap) => sent.push(tap ? [keys, tap] : [keys]), timers);
"""


def test_keys_are_named_by_their_place_not_their_text():
    """Shift turns 1 into "!" and Ctrl turns W into a control character: the code does not change."""
    got = _run("""console.log(JSON.stringify([
      K.name({code: 'KeyW', key: 'W'}), K.name({code: 'Digit3', key: '#'}), K.name({code: 'ArrowUp'}),
      K.name({code: 'Comma'}), K.name({code: 'ShiftLeft'}), K.name({code: 'ControlRight'}),
      K.name({code: 'Tab'}), K.name({code: 'F1'}), K.name({code: 'Space'}), K.name({})]));""")
    assert got == ['W', '3', 'UP', ',', 'SHIFT', 'CTRL', 'TAB', None, None, None]


def test_without_a_code_the_key_is_used():
    """Some synthetic and remote-desktop key events carry no code: fall back to the key."""
    got = _run("""console.log(JSON.stringify([
      K.name({code: '', key: 'w'}), K.name({key: 'W'}), K.name({key: '7'}), K.name({key: 'ArrowDown'}),
      K.name({key: 'Shift'}), K.name({key: 'Control'}), K.name({key: 'Tab'}), K.name({key: ','}),
      K.name({key: 'Enter'}), K.name({key: 'é'})]));""")
    assert got == ['W', 'W', '7', 'DOWN', 'SHIFT', 'CTRL', 'TAB', ',', None, None]


def test_mode_keys_are_on_shift():
    """J C 1-4 and the option keys drive the arm while Keyboard Control is on, so the mode keys need Shift."""
    got = _run("""const k = (code, more) => K.modeKey(Object.assign({code, shiftKey: true}, more || {}));
    console.log(JSON.stringify([k('KeyJ'), k('KeyC'), k('Digit2'), k('Digit5'), k('KeyF'), k('KeyP'), k('KeyE'),
      k('KeyW'), k('KeyJ', {shiftKey: false}), k('KeyJ', {ctrlKey: true})]));""")
    assert got == ['j', 'c', '2', None, 'f', 'p', 'e', None, None, None]


def test_the_driver_keys_are_the_studios():
    got = _run("""console.log(JSON.stringify(['W','S','A','D','Q','E','J','L','O','C','UP','DOWN',',','.','TAB','1','9',
      'SHIFT','CTRL','G','H','0'].map(K.isOurs)));""")
    assert got == [True] * 19 + [False] * 3


def test_held_keys_are_sent_at_once_then_every_100_ms_then_let_go():
    got = _run(_TIMERS + """
    d.down('W'); const t1 = ticker && ticker.ms;
    ticker.f(); ticker.f();
    d.down('SHIFT'); d.up('W');
    console.log(JSON.stringify({sent, t1, after: ticker}));""")
    assert got['t1'] == 100
    assert got['sent'] == [[['W']], [['W']], [['W']], [['SHIFT', 'W']], [['SHIFT']]]
    assert got['after'] is None                         # nothing held that drives: no more sends


def test_a_modifier_alone_sends_nothing():
    got = _run(_TIMERS + """d.down('SHIFT'); d.up('SHIFT'); console.log(JSON.stringify({sent, ticker}));""")
    assert got == {'sent': [], 'ticker': None}


def test_autorepeat_and_a_second_press_send_nothing_new():
    got = _run(_TIMERS + """d.down('W'); d.down('W'); d.up('S'); console.log(JSON.stringify(sent));""")
    assert got == [[['W']]]


def test_taps_go_at_once_with_what_is_held():
    got = _run(_TIMERS + """d.down('UP'); d.down('3'); d.down('TAB'); console.log(JSON.stringify({sent, held: d.held()}));""")
    assert got['sent'] == [[['UP']], [['UP'], '3'], [['UP'], 'TAB']]
    assert got['held'] == ['UP']                        # a tap is not held


def test_clear_lets_go_of_everything_once():
    """Losing the focus or hiding the tab: one empty list, then quiet."""
    got = _run(_TIMERS + """d.down('W'); d.down('A'); d.clear(); d.clear(); console.log(JSON.stringify({sent, ticker}));""")
    assert got == {'sent': [[['W']], [['A', 'W']], [[]]], 'ticker': None}


def test_the_page_loads_keys_before_the_shell_uses_them():
    html = (UI_DIR / 'index.html').read_text()
    assert html.index('/ui/app.js') < html.index('/ui/keys.js') < html.index('/ui/shell.js')
