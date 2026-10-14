"""Platform detection for the workspace build commands.

macOS and Linux need genuinely different ``colcon build`` invocations, and
getting one wrong breaks the build in ways that are slow to diagnose. The
regression these tests exist for: conda's ``rcutils`` exports a bare
``-latomic`` link flag (``rcutilsExport.cmake`` sets
``INTERFACE_LINK_LIBRARIES "atomic"``), which every dependent target inherits.
``libatomic.dylib`` *is* present in the RoboStack env, but colcon links every
other library by absolute path, so the link line carries no ``-L`` at all and
Apple's ``ld`` searches only ``/usr/lib`` and the SDK::

    ld: library 'atomic' not found

The build command therefore has to put the conda prefix on the library search
path itself. These tests pin that, and pin that Linux is left alone. Since
v4.5.0 no package is skipped on any platform.
"""
import pytest

from fic_studio import platform_utils
from fic_studio.platform_utils import colcon_build_command


@pytest.fixture
def on_macos(monkeypatch):
    """Force the platform helpers to report macOS."""
    monkeypatch.setattr(platform_utils, 'IS_MACOS', True)
    monkeypatch.setattr(platform_utils, 'IS_LINUX', False)


@pytest.fixture
def on_linux(monkeypatch):
    """Force the platform helpers to report Linux."""
    monkeypatch.setattr(platform_utils, 'IS_MACOS', False)
    monkeypatch.setattr(platform_utils, 'IS_LINUX', True)


def test_macos_build_puts_conda_lib_on_the_linker_search_path(on_macos):
    """Without this the rosidl targets fail with ``library 'atomic' not found``."""
    cmd = colcon_build_command()
    assert 'LIBRARY_PATH=' in cmd
    assert '$CONDA_PREFIX/lib' in cmd


def test_macos_build_appends_to_an_existing_library_path(on_macos):
    """A LIBRARY_PATH already set in the environment must survive, not be clobbered."""
    cmd = colcon_build_command()
    assert '${LIBRARY_PATH:+:$LIBRARY_PATH}' in cmd


def test_macos_library_path_precedes_the_colcon_invocation(on_macos):
    """The assignment only reaches the compiler if it prefixes the command."""
    cmd = colcon_build_command()
    assert cmd.index('LIBRARY_PATH=') < cmd.index('colcon build')


def test_macos_build_uses_the_conda_interpreter(on_macos):
    """The C++ packages resolve NumPy/Pinocchio headers via the conda Python."""
    assert '-DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python"' in colcon_build_command()


def test_macos_build_omits_symlink_install(on_macos):
    """setuptools>=80 dropped ``setup.py develop --editable``, which it relies on."""
    assert '--symlink-install' not in colcon_build_command()


def test_macos_build_is_the_whole_workspace(on_macos):
    """Every package builds on macOS since the C++ `mujoco` simulator (which
    linked the Linux-only ``-lrt``) was retired in v4.5.0, so nothing is
    skipped and nothing is allow-listed."""
    cmd = colcon_build_command()
    assert '--packages-skip' not in cmd
    assert '--packages-select' not in cmd


def test_linux_build_is_left_alone(on_linux):
    """Linux has a real libatomic on the default search path; adding one would be noise."""
    cmd = colcon_build_command()
    assert 'LIBRARY_PATH' not in cmd
    assert cmd.startswith('colcon build --symlink-install')


def test_linux_build_selects_only_what_was_asked_for(on_linux):
    """Explicit package selection passes straight through on Linux."""
    assert colcon_build_command(['fic_hardware']) == (
        'colcon build --symlink-install --packages-select fic_hardware')


def test_explicit_packages_still_get_the_macos_link_path(on_macos):
    """Building one package from Settings hits the same linker problem."""
    cmd = colcon_build_command(['communication'])
    assert '$CONDA_PREFIX/lib' in cmd
    assert '--packages-select communication' in cmd


# ── The shell build scripts encode the same knowledge as platform_utils. ──────
# They cannot import it (a fresh clone has nothing built yet), so these tests
# pin the two representations together instead of letting them drift.

def _repo_root():
    """Locate the repo root, skipping this test if running from an installed copy."""
    from fic_studio.config import FIC_ROOT
    return FIC_ROOT


def _read(relpath):
    """Return the text of a repo file, or skip if it isn't reachable from here."""
    path = _repo_root() / relpath
    if not path.is_file():
        pytest.skip('%s not reachable from an installed copy' % relpath)
    return path.read_text()


def test_build_script_skips_nothing():
    """./build.sh, like colcon_build_command(), builds the whole workspace."""
    assert '--packages-skip' not in _read('build.sh')


def test_build_script_sets_the_same_linker_search_path():
    """A divergent LIBRARY_PATH here would reintroduce the -latomic failure."""
    from fic_studio.platform_utils import MACOS_LINK_ENV
    expr = MACOS_LINK_ENV.split('=', 1)[1]
    assert expr in _read('build.sh')


def test_build_script_keeps_symlink_install_on_linux():
    """Linux still gets an editable install; only macOS cannot."""
    assert 'colcon build --symlink-install' in _read('build.sh')


def test_macos_installer_uses_the_same_skip_and_link_path():
    """The installer builds the workspace too, and hits the same linker problem."""
    from fic_studio.platform_utils import MACOS_LINK_ENV
    text = _read('fic_install/macos/install-fic-studio-macos.sh')
    assert '--packages-skip' not in text
    assert MACOS_LINK_ENV.split('=', 1)[1] in text


def test_linux_arm64_full_build_skips_nothing(on_linux):
    """Linux arm64 (e.g. an Arduino VENTUNO Q) builds everything too: the CPU
    simulator is the mujoco wheel, which has aarch64 builds."""
    assert colcon_build_command() == 'colcon build --symlink-install'


def test_no_cpp_mujoco_package_in_the_workspace():
    """The retired C++ simulator must not come back under fic_ws/src/mujoco."""
    root = _repo_root()
    if not (root / 'fic_ws' / 'src').is_dir():
        pytest.skip('workspace sources not reachable from an installed copy')
    # (Untracked leftovers of an old checkout may remain; a package.xml
    # would make colcon build it again.)
    assert not list((root / 'fic_ws' / 'src' / 'mujoco').glob('**/package.xml'))
