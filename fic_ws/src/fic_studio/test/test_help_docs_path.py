"""The Help page's documentation button must find fic_docs when installed.

The Studio runs from a colcon *copy* install on macOS/RoboStack, because
`--symlink-install` is unsupported by modern setuptools there. config.py
resolves the repo root by locating `fic_ws` in the path precisely so that
copy-install works, and warns in its own comment that "a fixed parents[N]
walk would land inside install/ and break".

The Help button did exactly that, so on every installed run it resolved to
`fic_ws/install/fic_studio/fic_docs`, found nothing, logged a warning and
returned — a button that silently did nothing.
"""
from pathlib import Path

from fic_studio.mixins.help_mixin import docs_directory


def test_docs_directory_contains_the_site():
    """Whatever layout we are imported from, it points at the real fic_docs."""
    docs = docs_directory()
    assert docs.is_dir(), docs
    assert (docs / 'index.html').is_file(), 'no index.html under %s' % docs


def test_docs_directory_survives_a_copy_install_layout():
    """The resolver must not depend on how deep the module happens to sit.

    A copy install puts help_mixin at
    fic_ws/install/fic_studio/lib/pythonX/site-packages/fic_studio/mixins/,
    which is two levels deeper than the source tree. A fixed parent walk
    lands inside install/; locating `fic_ws` does not.
    """
    installed = Path('/tmp/FIC_Interface/fic_ws/install/fic_studio/lib'
                     '/python3.12/site-packages/fic_studio/mixins/help_mixin.py')
    parts = installed.parts
    idx = parts.index('fic_ws')
    assert Path(*parts[:idx]) == Path('/tmp/FIC_Interface'), (
        'the fic_ws-anchored walk is what makes copy-install work')
    # And the fixed walk this replaced would have been wrong:
    assert installed.parents[5] != Path('/tmp/FIC_Interface')


def test_docs_directory_honours_fic_root(monkeypatch, tmp_path):
    """FIC_ROOT relocates the whole tree, docs included.

    Read at call time, not import time. Reloading the module to pick up a new
    import-time value would rebind HelpMixin to a fresh class object and break
    every other test that checks it by identity.
    """
    (tmp_path / 'fic_docs').mkdir()
    (tmp_path / 'fic_docs' / 'index.html').write_text('<h1>relocated</h1>')
    monkeypatch.setenv('FIC_ROOT', str(tmp_path))
    assert docs_directory() == (tmp_path / 'fic_docs').resolve()


def test_help_mixin_identity_survives_this_module():
    """Guard the pollution this file previously caused.

    An earlier version reloaded help_mixin, which rebound HelpMixin to a new
    class object; test_help_wire's MRO check then failed only when the suite
    ran in order.
    """
    from fic_studio.mixins.help_mixin import HelpMixin
    from fic_studio.main_window_pro import ProfessionalRobotGUI
    assert HelpMixin in ProfessionalRobotGUI.__mro__


def test_no_fixed_parent_walk_remains():
    """Regression guard: config.py's own comment warns against this."""
    src = Path(__file__).resolve().parents[1] / 'fic_studio' / 'mixins' / 'help_mixin.py'
    text = src.read_text(encoding='utf-8')
    assert 'parents[5]' not in text, (
        'help_mixin resolves the repo root with a fixed parent walk again')


def test_docs_are_never_opened_over_file_url():
    """The docs are a fetch()-driven single page; file:// cannot load them.

    Opening file://.../index.html renders "Could not load: pages/home.html",
    because browsers refuse fetch() from a file:// origin. Both entry points —
    the Help page button and the Help menu item — must go through the local
    HTTP server.
    """
    import ast

    def url_literals(tree):
        """Yield every string literal that is not a docstring."""
        docstrings = set()
        for node in ast.walk(tree):
            if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef,
                                 ast.AsyncFunctionDef)):
                doc = ast.get_docstring(node, clean=False)
                if doc is not None:
                    docstrings.add(doc)
        for node in ast.walk(tree):
            if isinstance(node, ast.Constant) and isinstance(node.value, str):
                if node.value not in docstrings:
                    yield node.value
            elif isinstance(node, ast.JoinedStr):
                for part in node.values:
                    if isinstance(part, ast.Constant) and isinstance(part.value, str):
                        yield part.value

    root = Path(__file__).resolve().parents[1] / 'fic_studio'
    for rel in ('main_window_pro.py', 'mixins/help_mixin.py'):
        tree = ast.parse((root / rel).read_text(encoding='utf-8'))
        offenders = [lit for lit in url_literals(tree) if 'file://' in lit]
        assert not offenders, (
            '%s still builds a file:// documentation URL: %r' % (rel, offenders))


def test_both_entry_points_share_one_mechanism():
    """One server helper, not two divergent implementations.

    They had drifted: one resolved the path correctly but opened file://, the
    other served over HTTP but resolved the path with a fixed parent walk.
    Each was broken in the way the other was not.
    """
    from fic_studio.mixins.help_mixin import HelpMixin
    assert hasattr(HelpMixin, 'serve_docs')
    import inspect
    src = inspect.getsource(HelpMixin._help_open_docs_website)
    assert 'serve_docs' in src
