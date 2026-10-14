"""The estimator modules must not import ROS, Qt or plotting libraries."""

import ast
import os

_PURE = ('residual.py', 'hypotheses.py', 'geometry.py', 'recorder.py',
         'gate.py', 'admittance.py')
_BANNED = {'rclpy', 'PyQt5', 'fic_studio', 'matplotlib', 'pandas', 'dill'}
_SRC = os.path.join(os.path.dirname(__file__), '..', 'fic_contact')


def _imports(path):
    """Return the set of top-level module names imported by a file."""
    with open(path, 'r', encoding='utf-8') as handle:
        tree = ast.parse(handle.read())
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(a.name.split('.')[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            names.add(node.module.split('.')[0])
    return names


def test_pure_modules_have_no_banned_imports():
    """Every pure module avoids ROS, Qt, plotting and absent dependencies."""
    for name in _PURE:
        path = os.path.join(_SRC, name)
        if not os.path.exists(path):
            continue
        assert not (_imports(path) & _BANNED), \
            f'{name} imports a banned module'
