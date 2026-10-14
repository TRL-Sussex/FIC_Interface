"""editor.js's pure functions (web/editor.js), run with Node; skipped without it."""
import json
import os
import shutil
import subprocess

import pytest

EDITOR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'web', 'editor.js')
NODE = shutil.which('node')
pytestmark = pytest.mark.skipif(NODE is None, reason='node is not installed')


def js(expr):
    """Evaluate ``expr`` with editor.js imported as E; return its value through JSON."""
    src = ('import * as E from %s;\nprocess.stdout.write(JSON.stringify(%s));\n'
           % (json.dumps('file://' + EDITOR), expr))
    r = subprocess.run([NODE, '--input-type=module', '-e', src],
                       capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stderr
    return json.loads(r.stdout)


def toks(line, api='[]', open_=''):
    """The tokens editor.js makes of one line."""
    return js('E.tokenize(%s, %s, new Set(%s)).tokens' % (json.dumps(line), json.dumps(open_), api))


def test_robot_calls_and_comments():
    assert toks('robot.motion.home()  # go') == [['api', 'robot.motion.home'], ['txt', '()  '], ['com', '# go']]


def test_keywords_and_the_name_after_def():
    assert toks('def go(x):') == [['kw', 'def'], ['txt', ' '], ['def', 'go'], ['txt', '(x):']]


def test_a_hash_in_a_string_is_not_a_comment():
    assert toks("print('a # b')") == [['builtin', 'print'], ['txt', '('], ['str', "'a # b'"], ['txt', ')']]


def test_numbers():
    assert toks('x = 3.5e2') == [['txt', 'x = '], ['num', '3.5e2']]


def test_flat_aliases_are_coloured_like_robot_calls():
    assert toks('move_joints([0] * 7)', api='["move_joints"]')[0] == ['api', 'move_joints']


def test_triple_quoted_strings_span_lines():
    assert js('E.tokenize(\'"""doc\', "", new Set())') == {'tokens': [['str', '"""doc']], 'open': '"""'}
    assert js('E.tokenize(\'end""" x\', \'"""\', new Set())') == {'tokens': [['str', 'end"""'], ['txt', ' x']], 'open': ''}
    assert js('E.tokenize("", \'"""\', new Set())') == {'tokens': [], 'open': '"""'}


def test_highlight_escapes_html_and_marks_a_line():
    assert js('E.highlight("a < b", new Set())') == 'a &lt; b'
    assert '<span class="se-api">robot.wait</span>' in js('E.highlight("robot.wait(1)", new Set())')
    assert js('E.highlight("x\\ny", new Set(), 2)') == 'x\n<span class="se-markline">y</span>'


def test_tab_indents_the_selected_lines():
    r = js('E.shiftLines("a\\nb\\nc", 0, 3, false)')
    assert (r['text'], r['start'], r['end']) == ('    a\n    b\nc', 0, 11)


def test_a_selection_ending_at_a_line_start_leaves_that_line():
    r = js('E.shiftLines("a\\nb", 0, 2, false)')
    assert (r['text'], r['start'], r['end']) == ('    a\nb', 0, 6)


def test_shift_tab_dedents_the_caret_line():
    r = js('E.shiftLines("        x", 8, 8, true)')
    assert (r['text'], r['start'], r['end']) == ('    x', 4, 4)
    r = js('E.shiftLines("  x", 2, 2, true)')
    assert (r['text'], r['start'], r['end']) == ('x', 0, 0)


def test_enter_keeps_the_indent_and_adds_one_after_a_colon():
    assert js('E.newlineIndent("    for i in range(3):")') == ' ' * 8
    assert js('E.newlineIndent("    x = 1  # note:")') == ' ' * 4
    assert js('E.newlineIndent("if x:  # check")') == ' ' * 4
    assert js('E.newlineIndent("x = 1")') == ''


def test_output_lines_that_point_at_the_script():
    assert js('E.scriptLine(\'  File "<script>", line 12, in <module>\')') == 12
    assert js('E.scriptLine(\'  File "x.py", line 3\')') is None
