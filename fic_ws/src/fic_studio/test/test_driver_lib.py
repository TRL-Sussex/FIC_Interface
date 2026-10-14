"""Tests for driver_lib: reading, copying and building a robot's driver library.

Libraries are made up in temporary folders; the repository's own
robot_library is only read, never written.
"""
import os
import shutil
import stat
import subprocess
import time
import zipfile

from fic_studio.robot_import import driver_lib
from fic_studio.robot_import.driver_lib import (
    analyse_library, driver_command, find_programs, guess_program, install_library,
    LibraryError, pick_program,
)
import pytest

REPO_LIB = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..',
                                        'robot_library'))
BRIDGE = ('// Talks to the robot and to FIC through fic/shared_params.h.\n'
          'int main(int argc, char** argv) { return 0; }\n')
STUB = ('#include <sys/mman.h>\nstatic const char* seg = "/robot_feedback_memory";\n'
        'int main() { return 0; }\n')
VENDOR = 'int vendor_call() { return 1; }\n'


def _write(path, text='', mode=None):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as fh:
        fh.write(text)
    if mode:
        os.chmod(path, mode)
    return path


def _fake_program(path, mtime=None):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'wb') as fh:
        fh.write(b'\x7fELF' + b'\0' * 60)
    os.chmod(path, 0o755)
    if mtime is not None:
        os.utime(path, (mtime, mtime))
    return path


def make_cmake_lib(root, name='acme_driver'):
    """Make a vendor SDK with FIC's bridge in examples/, built by a top CMake project."""
    lib = os.path.join(root, name)
    _write(os.path.join(lib, 'CMakeLists.txt'),
           'cmake_minimum_required(VERSION 3.10)\nproject(acme CXX)\n'
           'add_subdirectory(examples)\n')
    _write(os.path.join(lib, 'examples', 'CMakeLists.txt'),
           'add_executable(acme_bridge acme_bridge.cpp)\n'
           'add_executable(vendor_demo vendor_demo.cpp)\n')
    _write(os.path.join(lib, 'examples', 'acme_bridge.cpp'), BRIDGE)
    _write(os.path.join(lib, 'examples', 'vendor_demo.cpp'), 'int main() { return 0; }\n')
    _write(os.path.join(lib, 'src', 'vendor.cpp'), VENDOR)
    _write(os.path.join(lib, 'README.md'), 'Acme SDK\n')
    return lib


def make_kinova_like(root):
    """Kinova's layout: a build script beside a CMake project, and a built program."""
    lib = os.path.join(root, 'Kinova')
    ex = os.path.join(lib, 'api_cpp', 'examples')
    _write(os.path.join(ex, 'CMakeLists.txt'), 'project(examples)\n')
    _write(os.path.join(ex, 'scripts', 'build-gcc.sh'), '#!/bin/bash\n', 0o755)
    _write(os.path.join(ex, '01-torque-control', '00-Torque_Controller.cpp'), BRIDGE)
    _fake_program(os.path.join(ex, 'build-gcc-release', '01-torque-control_00-Torque_Controller'))
    _fake_program(os.path.join(ex, 'build-gcc-release', 'CMakeFiles', '3.28', 'a.out'))
    return lib


# ── Reading a library ─────────────────────────────────────────────────────

class TestAnalyse:

    def test_cmake_library(self, tmp_path):
        plan = analyse_library(make_cmake_lib(str(tmp_path)))
        assert plan.name == 'acme_driver' and not plan.in_place
        assert plan.drivers == ['examples/acme_bridge.cpp']
        assert plan.build.kind == 'cmake' and plan.build.directory == 'build'
        assert plan.build.command.startswith('cmake .. -DCMAKE_BUILD_TYPE=Release && make')
        assert plan.build.build_directory('acme_driver') == '{robot_lib}/acme_driver/build'
        assert plan.build.covers_driver
        assert guess_program(plan) == 'build/acme_bridge'
        assert (driver_command(plan.name, guess_program(plan))
                == 'cd {robot_lib}/acme_driver/build && ./acme_bridge')
        assert plan.files == 6 and plan.size_bytes > 0
        assert not [m for m in plan.messages if m[0] == driver_lib.WARN]

    def test_kinova_layout(self, tmp_path):
        plan = analyse_library(make_kinova_like(str(tmp_path)))
        b = plan.build
        assert (b.kind, b.directory, b.command) == ('script', 'api_cpp/examples',
                                                    './scripts/build-gcc.sh')
        assert b.build_directory('Kinova') == '{robot_lib}/Kinova/api_cpp/examples'
        # Built already: the program is known, CMake's probe binary is not one.
        assert plan.programs == [
            'api_cpp/examples/build-gcc-release/01-torque-control_00-Torque_Controller']
        assert (driver_command('Kinova', guess_program(plan))
                == 'cd {robot_lib}/Kinova/api_cpp/examples/build-gcc-release'
                   ' && ./01-torque-control_00-Torque_Controller')

    def test_bridge_ranks_before_stub(self, tmp_path):
        lib = str(tmp_path / 'bridge')
        _write(os.path.join(lib, 'scripts', 'build.sh'), '#!/bin/bash\n', 0o755)
        _write(os.path.join(lib, 'src', 'fic_shm_stub.cpp'), STUB)
        _write(os.path.join(lib, 'src', 'zz_bridge.cpp'), BRIDGE)
        plan = analyse_library(lib)
        assert plan.drivers == ['src/zz_bridge.cpp', 'src/fic_shm_stub.cpp']
        assert (plan.build.directory, plan.build.command) == ('.', './scripts/build.sh')
        assert driver_lib.program_choices(plan) == ['build/zz_bridge', 'build/fic_shm_stub']

    def test_vendor_sdk_without_fic_driver_warns(self, tmp_path):
        lib = str(tmp_path / 'sdk')
        _write(os.path.join(lib, 'Makefile'), 'all:\n')
        _write(os.path.join(lib, 'vendor.cpp'), VENDOR)
        plan = analyse_library(lib)
        assert plan.build.kind == 'make' and not plan.drivers
        warn = [t for lvl, t in plan.messages if lvl == driver_lib.WARN]
        assert warn and 'shared_params.h' in warn[0] and 'Real robot' in warn[0]
        assert guess_program(plan) == ''

    def test_nothing_to_build_warns(self, tmp_path):
        lib = str(tmp_path / 'prebuilt')
        _write(os.path.join(lib, 'bridge.cpp'), BRIDGE)
        plan = analyse_library(lib)
        assert not plan.builds
        assert any('No build script' in t for _, t in plan.messages)

    def test_python_driver(self, tmp_path):
        lib = str(tmp_path / 'pydrv')
        _write(os.path.join(lib, 'pyproject.toml'), '[project]\nname = "pydrv"\n')
        _write(os.path.join(lib, 'pydrv', 'bridge.py'),
               'SEG = "/robot_feedback_memory"\nif __name__ == "__main__":\n    pass\n')
        plan = analyse_library(lib)
        assert plan.drivers == ['pydrv/bridge.py'] and plan.build.kind == 'python'
        assert guess_program(plan) == 'pydrv/bridge.py'
        assert (driver_command('pydrv', 'pydrv/bridge.py', '--ip 10.0.0.2')
                == 'cd {robot_lib}/pydrv/pydrv && python3 bridge.py --ip 10.0.0.2')

    def test_zip_with_top_folder_keeps_exec_bits(self, tmp_path):
        lib = make_kinova_like(str(tmp_path / 'src'))
        zpath = str(tmp_path / 'Kinova.zip')
        with zipfile.ZipFile(zpath, 'w') as zf:
            for dirpath, _, files in os.walk(lib):
                for f in files:
                    full = os.path.join(dirpath, f)
                    info = zipfile.ZipInfo.from_file(full, os.path.relpath(full, tmp_path / 'src'))
                    with open(full, 'rb') as fh:
                        zf.writestr(info, fh.read())
        plan = analyse_library(zpath)
        try:
            assert plan.name == 'Kinova' and os.path.basename(plan.root) == 'Kinova'
            script = os.path.join(plan.root, 'api_cpp', 'examples', 'scripts', 'build-gcc.sh')
            assert os.stat(script).st_mode & stat.S_IXUSR
            assert plan.build.command == './scripts/build-gcc.sh'
            temp = plan.temp_dir
        finally:
            plan.cleanup()
        assert not os.path.exists(temp)

    def test_unsafe_and_bad_zips_refused(self, tmp_path):
        evil = str(tmp_path / 'evil.zip')
        with zipfile.ZipFile(evil, 'w') as zf:
            zf.writestr('../outside.txt', 'x')
        with pytest.raises(LibraryError, match='unsafe'):
            analyse_library(evil)
        assert not (tmp_path / 'outside.txt').exists()
        bad = _write(str(tmp_path / 'bad.zip'), 'not a zip')
        with pytest.raises(LibraryError, match='not a valid zip'):
            analyse_library(bad)
        with pytest.raises(LibraryError, match='folder'):
            analyse_library(_write(str(tmp_path / 'notes.txt'), 'x'))

    def test_library_already_in_robot_library(self, tmp_path):
        lib_root = str(tmp_path / 'robot_library')
        lib = make_cmake_lib(os.path.join(lib_root, 'Vendor'), 'sdk')
        plan = analyse_library(lib, lib_root=lib_root)
        assert plan.in_place and plan.name == os.path.join('Vendor', 'sdk')
        assert any('nothing copied' in t for _, t in plan.messages)
        assert install_library(plan, lib_root) == lib
        with pytest.raises(LibraryError, match='robot_library itself'):
            analyse_library(lib_root, lib_root=lib_root)

    def test_too_big_a_folder_refused(self, tmp_path, monkeypatch):
        monkeypatch.setattr(driver_lib, 'MAX_FILES', 3)
        with pytest.raises(LibraryError, match='too much'):
            analyse_library(make_cmake_lib(str(tmp_path)))


# ── Copying it in ─────────────────────────────────────────────────────────

class TestInstall:

    def test_copies_through_staging_and_refuses_a_clash(self, tmp_path):
        lib_root = str(tmp_path / 'robot_library')
        plan = analyse_library(make_cmake_lib(str(tmp_path / 'src')))
        seen = []
        target = install_library(plan, lib_root, progress=seen.append)
        assert target == os.path.join(lib_root, 'acme_driver')
        assert os.path.isfile(os.path.join(target, 'examples', 'acme_bridge.cpp'))
        assert os.listdir(lib_root) == ['acme_driver']          # no staging folder left
        assert seen and seen[-1] == plan.size_bytes
        with pytest.raises(LibraryError, match='already exists'):
            install_library(plan, lib_root)
        _write(os.path.join(target, 'stale.txt'), 'old')
        install_library(plan, lib_root, replace=True)
        assert not os.path.exists(os.path.join(target, 'stale.txt'))

    def test_cancelled_copy_leaves_nothing(self, tmp_path):
        lib_root = str(tmp_path / 'robot_library')
        plan = analyse_library(make_cmake_lib(str(tmp_path / 'src')))

        def stop(_n):
            raise driver_lib.Cancelled('Copy cancelled.')
        with pytest.raises(driver_lib.Cancelled):
            install_library(plan, lib_root, progress=stop)
        assert os.listdir(lib_root) == []

    def test_expand(self):
        assert (driver_lib.expand('{robot_lib}/x/{robot_name}', '/lib', 'R1')
                == '/lib/x/R1')


# ── Finding the program a build made ──────────────────────────────────────

class TestPrograms:

    def test_named_after_the_driver_wins(self, tmp_path):
        lib = str(tmp_path)
        old = time.time() - 3600
        _fake_program(os.path.join(lib, 'build', 'examples', 'vendor_demo'))
        _fake_program(os.path.join(lib, 'build', 'examples', 'acme_bridge'), mtime=old)
        _fake_program(os.path.join(lib, 'build', 'CMakeFiles', 'x', 'a.out'))
        _write(os.path.join(lib, 'build', 'not_a_program'), 'text', 0o755)
        drivers = ['examples/acme_bridge.cpp']
        assert find_programs(lib, drivers) == ['build/examples/acme_bridge',
                                               'build/examples/vendor_demo']
        assert pick_program(lib, drivers) == 'build/examples/acme_bridge'
        # Older than the build: not from it.
        assert find_programs(lib, drivers, since=old + 60) == ['build/examples/vendor_demo']

    def test_only_one_unnamed_program_is_trusted(self, tmp_path):
        lib = str(tmp_path)
        _fake_program(os.path.join(lib, 'build', 'robot_node'))
        assert pick_program(lib, ['src/bridge.cpp']) == 'build/robot_node'
        _fake_program(os.path.join(lib, 'build', 'other_node'))
        assert pick_program(lib, ['src/bridge.cpp']) == ''

    def test_name_problem_and_users(self):
        assert driver_lib.name_problem('') and driver_lib.name_problem('a/b')
        assert driver_lib.name_problem('.hidden') and not driver_lib.name_problem('lib-2.0_x')
        configs = {
            'A': {'driver_command': 'cd {robot_lib}/Kinova/api_cpp && ./x'},
            'B': {'build_directory': '{robot_lib}/Kinova'},
            'C': {'build_directory': '{robot_lib}/Kinova2/build'},
            'D': {'driver_command': None},
        }
        assert driver_lib.users_of('Kinova', configs) == ['A', 'B']


@pytest.mark.skipif(not (shutil.which('cmake') and shutil.which('make')
                         and (shutil.which('c++') or shutil.which('g++'))),
                    reason='needs cmake, make and a C++ compiler')
def test_real_cmake_build_then_the_driver_runs(tmp_path):
    """Copy, build with the planned command, find the program, run the driver command."""
    lib_root = str(tmp_path / 'robot_library')
    plan = analyse_library(make_cmake_lib(str(tmp_path / 'src')))
    lib_dir = install_library(plan, lib_root)
    build_dir = driver_lib.expand(plan.build.build_directory(plan.name), lib_root)
    os.makedirs(build_dir)
    since = time.time() - 1
    out = subprocess.run(['bash', '-c', plan.build.command], cwd=build_dir,
                         capture_output=True, text=True, timeout=240)
    assert out.returncode == 0, out.stdout + out.stderr
    # The guess (build/acme_bridge) was wrong; the build put it in build/examples.
    assert not os.path.exists(os.path.join(lib_dir, guess_program(plan)))
    program = pick_program(lib_dir, plan.drivers, since)
    assert program == 'build/examples/acme_bridge'
    cmd = driver_lib.expand(driver_command(plan.name, program), lib_root)
    assert subprocess.run(['bash', '-c', cmd], timeout=30).returncode == 0


@pytest.mark.skipif(not os.path.isdir(REPO_LIB), reason='no robot_library in this checkout')
class TestShippedLibraries:
    """The repository's own libraries read back as their shipped settings (read only)."""

    def test_kinova(self):
        from fic_studio.mixins.robot_config_mixin import DEFAULT_ROBOT_CONFIGS
        if not os.path.isdir(os.path.join(REPO_LIB, 'Kinova')):
            pytest.skip('Kinova SDK not present')
        cfg = DEFAULT_ROBOT_CONFIGS['KinovaGen3_7DOF']
        plan = analyse_library(os.path.join(REPO_LIB, 'Kinova'), lib_root=REPO_LIB)
        assert plan.in_place and plan.name == 'Kinova'
        assert plan.build.build_directory(plan.name) == cfg['build_directory']
        assert plan.build.command == cfg['build_command']
        if plan.programs:            # built in this checkout
            assert driver_command(plan.name, guess_program(plan)) == cfg['driver_command']

    def test_cspm(self):
        if not os.path.isdir(os.path.join(REPO_LIB, 'CSPM')):
            pytest.skip('CSPM bridge not present')
        plan = analyse_library(os.path.join(REPO_LIB, 'CSPM'), lib_root=REPO_LIB)
        assert plan.build.command == './scripts/build.sh'
        assert plan.drivers[0] == 'src/cspm_fic_bridge.cpp'
        assert (driver_command(plan.name, guess_program(plan))
                == 'cd {robot_lib}/CSPM/build && ./cspm_fic_bridge')
