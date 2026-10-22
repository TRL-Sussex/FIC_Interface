"""The examples shipped in examples/ (lessons with solutions, and demos): seeded into the user's examples folder
beside the built-in ones, never over a copy the user changed; each compiles and says what it is."""
import ast
import importlib
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
SHIPPED = sorted((HERE / 'examples').glob('*.py'))


def _extension_module():
    pytest.importorskip('PyQt5')
    if str(HERE.parent) not in sys.path:
        sys.path.insert(0, str(HERE.parent))
    return importlib.import_module('script_editor.extension')


def test_there_are_five_lessons_each_with_a_solution_and_some_demos():
    names = [p.stem for p in SHIPPED]
    lessons = [n for n in names if n.startswith('lesson_') and not n.endswith('_solution')]
    assert len(lessons) == 5
    for n in range(1, 6):
        assert f'lesson_{n}_solution' in names
        assert any(name.startswith(f'lesson_{n}_') for name in lessons)
    assert len([n for n in names if n.startswith('demo_')]) >= 5


@pytest.mark.parametrize('path', SHIPPED, ids=lambda p: p.name)
def test_each_example_compiles_and_says_what_it_is(path):
    tree = ast.parse(path.read_text(), str(path))
    doc = ast.get_docstring(tree)
    assert doc and doc.splitlines()[0].strip(), 'the gallery shows the first line of the docstring'


def test_the_shipped_examples_are_seeded_beside_the_built_in_ones_without_overwriting(tmp_path):
    ext = _extension_module()
    (tmp_path / SHIPPED[0].name).write_text('# my own changes\n')
    ext._seed_examples(str(tmp_path))
    for path in SHIPPED[1:]:
        assert (tmp_path / path.name).read_text() == path.read_text()
    assert (tmp_path / SHIPPED[0].name).read_text() == '# my own changes\n'
    assert (tmp_path / '01_read_state.py').exists()
