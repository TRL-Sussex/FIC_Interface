"""Tests for Extensions/script_editor/web_backend.py: no Qt, no ROS."""
import os as _os
import sys
import time

import pytest

_EXT_ROOT = _os.path.abspath(_os.path.join(_os.path.dirname(__file__), '..', '..'))
if _EXT_ROOT not in sys.path:
    sys.path.insert(0, _EXT_ROOT)

from Extensions.script_editor import web_backend as wb  # noqa: E402


@pytest.mark.parametrize('name', ['pick_place', 'Demo-2', 'a', '01_read_state'])
def test_plain_names_are_accepted(name):
    assert wb.check_name('  %s ' % name) == name


@pytest.mark.parametrize('name', ['', '   ', '../x', 'a/b', 'a\\b', 'x.py', 'two words', None, 'é'])
def test_other_names_are_refused(name):
    with pytest.raises(ValueError, match='Use letters, digits, _ or - in the name'):
        wb.check_name(name)


def test_failing_line_is_the_last_script_frame():
    tb = ('Traceback (most recent call last):\n'
          '  File "/opt/fic/Extensions/script_editor/extension.py", line 320, in run\n'
          '  File "<script>", line 3, in <module>\n'
          '  File "<script>", line 9, in helper\n'
          '  File "/opt/fic/Extensions/script_editor/robot_api.py", line 88, in move_joints\n'
          'ValueError: bad\n')
    assert wb.failing_line(tb) == 9


def test_failing_line_of_a_syntax_error():
    assert wb.failing_line('  File "<script>", line 4\n    for x in\n SyntaxError: invalid syntax\n') == 4


def test_no_script_frame_gives_none():
    assert wb.failing_line('  File "<string>", line 2\nNameError: x') is None
    assert wb.failing_line('') is None
    assert wb.failing_line(None) is None


def test_printed_chunks_join_into_lines():
    out = wb.OutputStore()
    out.write('hello')                  # print('hello') writes the text, then the newline
    assert out.tail() == ['hello']      # an open line is shown
    out.write('\n')
    out.write('a\nb\n')
    assert out.tail() == ['hello', 'a', 'b']


def test_seq_changes_only_when_the_output_does():
    out = wb.OutputStore()
    seq = out.seq
    out.write('')
    assert out.seq == seq
    out.write('x\n')
    assert out.seq == seq + 1
    out.clear()
    assert out.tail() == [] and out.seq == seq + 2


def test_only_the_newest_lines_are_kept():
    out = wb.OutputStore(maxlen=3)
    out.write('1\n2\n3\n4\n')
    assert out.tail() == ['2', '3', '4']
    assert out.tail(2) == ['3', '4']


def test_stamped_lines_carry_the_time_they_began():
    clock = [3600.0]
    out = wb.OutputStore(clock=lambda: clock[0])
    out.write('hel')
    clock[0] = 3605.0
    out.write('lo\n\n[09:00:00] already stamped\n')
    first = time.strftime('[%H:%M:%S] ', time.localtime(3600.0))
    assert out.tail(stamped=True) == [first + 'hello', '', '[09:00:00] already stamped']
    assert out.tail() == ['hello', '', '[09:00:00] already stamped']


def test_list_scripts_skips_names_the_page_cannot_use(tmp_path):
    for f in ('b.py', 'a.py', 'a.b.py', 'two words.py', 'notes.txt'):
        (tmp_path / f).write_text('x = 1\n')
    (tmp_path / 'examples').mkdir()
    assert wb.list_scripts(str(tmp_path)) == ['a', 'b']
    assert wb.list_scripts(str(tmp_path / 'missing')) == []


def test_describe_takes_the_docstrings_first_line_without_the_file_name():
    src = '"""01_read_state.py — Read robot state using the new API.\n\nMore.\n"""\nprint(1)\n'
    assert wb.describe(src) == 'Read robot state using the new API.'
    assert wb.describe('"""Moves home."""\n') == 'Moves home.'
    assert wb.describe('x = 1\n') == ''
    assert wb.describe('def (:\n') == ''


def test_read_script(tmp_path):
    (tmp_path / 'demo.py').write_text('print(1)\n', encoding='utf-8')
    assert wb.read_script(str(tmp_path), 'demo') == 'print(1)\n'
    with pytest.raises(ValueError, match='No such script'):
        wb.read_script(str(tmp_path), 'other')
    with pytest.raises(ValueError, match='Use letters'):
        wb.read_script(str(tmp_path), '../demo')


def test_a_file_that_is_not_utf8_still_opens(tmp_path):
    (tmp_path / 'old.py').write_bytes(b'# caf\xe9\nx = 1\n')
    assert wb.read_script(str(tmp_path), 'old') == '# caf�\nx = 1\n'
