"""Set up a robot's driver library: the vendor SDK and FIC's bridge program.

A real robot runs through a driver: a program built from the vendor's SDK
that talks to the robot and, through FIC's shared memory
(``fic_library/include/fic/shared_params.h``: the ``/robot_feedback_memory``
segment), to the FIC hardware node. The shipped robots keep theirs in
``robot_library/<name>`` -- libfranka, Kinova's api_cpp, the CSPM bridge.

``analyse_library(source)`` looks at a dropped folder or ``.zip``: how it
builds (a build script, a CMake project, a Makefile, a Python package),
which sources speak FIC's shared memory (those are the driver), and what
the build and driver commands should be. ``install_library`` copies it into
``robot_library``; after the first build, ``find_programs`` finds the
driver program so the driver command can point at it.

Commands use ``{robot_lib}`` for the robot_library folder, as the Studio's
Compile and API buttons expand it.
"""
from dataclasses import dataclass, field
import os
import re
import shutil
import tempfile
from typing import List, Optional
import zipfile

#: What a source using FIC's shared memory mentions.
_FIC_MARKERS = (b'shared_params.h', b'/robot_feedback_memory', b'SharedFeedback')
_SOURCE_EXTS = ('.cpp', '.cc', '.cxx', '.c', '.h', '.hpp', '.py')
_SKIP_DIRS = {'.git', '.svn', '.hg', '__pycache__', 'node_modules', '.idea', '.vscode'}
#: Folders a build writes into; searched for programs, not for sources.
_BUILD_DIR_RE = re.compile(r'^(build|cmake-build|out)([-_].*)?$', re.I)
_MAIN_RE = re.compile(rb'\bint\s+main\s*\(|__name__\s*==\s*[\'"]__main__[\'"]')
#: Programs named like these are tools around a driver, not the driver.
_TOOL_RE = re.compile(r'stub|mock|test|sim|example|demo', re.I)
_JOBS = '-j$(sysctl -n hw.ncpu 2>/dev/null || nproc)'   # as the shipped robots' commands
#: A folder bigger than this is not one driver library (a home folder, say).
MAX_FILES = 100_000
MAX_BYTES = 5 * 10**9

INFO, WARN, ERROR = 'info', 'warn', 'error'


class LibraryError(Exception):
    """The library cannot be set up; the message is written for the user."""


class Cancelled(LibraryError):
    """The user stopped the copy."""


@dataclass
class BuildPlan:
    """One way the library can be built."""

    kind: str                 # 'script' | 'cmake' | 'make' | 'python'
    directory: str            # where the command runs, relative to the library root
    command: str
    label: str                # how the review screen names it
    covers_driver: bool = False   # the driver's source is in what it builds

    def build_directory(self, name):
        rel = self.directory.strip('./') if self.directory not in ('.', '') else ''
        return '{robot_lib}/%s%s' % (name, '/' + rel if rel else '')


@dataclass
class LibraryPlan:
    source: str               # what was dropped
    root: str                 # the library's top folder (inside the zip, if one)
    name: str                 # its folder name in robot_library
    builds: List[BuildPlan] = field(default_factory=list)
    fic_sources: List[str] = field(default_factory=list)    # relative to root
    drivers: List[str] = field(default_factory=list)        # FIC sources with a main()
    programs: List[str] = field(default_factory=list)       # executables already built
    size_bytes: int = 0
    files: int = 0
    in_place: bool = False    # already in robot_library: nothing to copy
    messages: List[tuple] = field(default_factory=list)
    temp_dir: Optional[str] = None

    @property
    def build(self):
        return self.builds[0] if self.builds else None

    def cleanup(self):
        if self.temp_dir:
            shutil.rmtree(self.temp_dir, ignore_errors=True)
            self.temp_dir = None


# ── Looking at it ─────────────────────────────────────────────────────────

def _unpack(source):
    """(root folder, temp dir or None) for a dropped folder or zip."""
    source = os.path.abspath(os.path.expanduser(source))
    if os.path.isdir(source):
        return source, None
    if not (os.path.isfile(source) and source.lower().endswith('.zip')):
        raise LibraryError('Drop the driver library as a folder, or as a .zip of that folder.')
    temp = tempfile.mkdtemp(prefix='fic_driver_lib_')
    try:
        with zipfile.ZipFile(source) as zf:
            real = os.path.realpath(temp) + os.sep
            for member in zf.namelist():
                if not os.path.realpath(os.path.join(temp, member)).startswith(real):
                    raise LibraryError('The zip file contains an unsafe path: %s' % member)
            zf.extractall(temp)
            # zipfile drops the executable bits; build scripts need them back.
            for info in zf.infolist():
                mode = (info.external_attr >> 16) & 0o777
                if mode & 0o111:
                    path = os.path.join(temp, info.filename)
                    if os.path.isfile(path):
                        os.chmod(path, mode)
    except zipfile.BadZipFile:
        shutil.rmtree(temp, ignore_errors=True)
        raise LibraryError('%s is not a valid zip file.' % os.path.basename(source))
    except LibraryError:
        shutil.rmtree(temp, ignore_errors=True)
        raise
    entries = [e for e in os.listdir(temp) if not e.startswith(('.', '__MACOSX'))]
    if len(entries) == 1 and os.path.isdir(os.path.join(temp, entries[0])):
        return os.path.join(temp, entries[0]), temp
    return temp, temp


def _is_program(path):
    """Tell whether ``path`` is an ELF executable (not one of CMake's probes)."""
    if '%sCMakeFiles%s' % (os.sep, os.sep) in path:
        return False
    try:
        if not os.access(path, os.X_OK) or os.path.islink(path):
            return False
        with open(path, 'rb') as fh:
            head = fh.read(4)
    except OSError:
        return False
    return head == b'\x7fELF'


def _mentions_fic(path):
    try:
        with open(path, 'rb') as fh:
            data = fh.read(2_000_000)
    except OSError:
        return False, False
    return any(m in data for m in _FIC_MARKERS), bool(_MAIN_RE.search(data))


def _cmake_project(path):
    try:
        with open(path, errors='replace') as fh:
            return re.search(r'^\s*project\s*\(', fh.read(), re.I | re.M) is not None
    except OSError:
        return False


def analyse_library(source, lib_root=None, name=None):
    """Work out how the library at ``source`` builds and which program drives the robot."""
    root, temp = _unpack(source)
    try:
        plan = LibraryPlan(source=os.path.abspath(source), root=root,
                           name=name or _safe_name(os.path.basename(root.rstrip(os.sep))),
                           temp_dir=temp)
        if lib_root and not temp:
            # Already somewhere in robot_library: use it where it is.
            real, real_lib = os.path.realpath(root), os.path.realpath(lib_root)
            if real == real_lib:
                raise LibraryError('That is robot_library itself. Drop one library from it '
                                   '(Kinova, libfranka...), not the whole folder.')
            if real.startswith(real_lib + os.sep):
                plan.in_place = True
                plan.name = os.path.relpath(real, real_lib)
        scripts, cmake_roots, makefiles, pythons = [], [], [], []
        for dirpath, dirnames, filenames in os.walk(root):
            rel_dir = os.path.relpath(dirpath, root)
            in_build = any(_BUILD_DIR_RE.match(p) for p in rel_dir.split(os.sep) if p != '.')
            dirnames[:] = [d for d in dirnames if d not in _SKIP_DIRS]
            for f in filenames:
                path = os.path.join(dirpath, f)
                rel = os.path.normpath(os.path.join(rel_dir, f))
                try:
                    plan.size_bytes += os.lstat(path).st_size
                except OSError:
                    pass
                plan.files += 1
                if plan.files > MAX_FILES or plan.size_bytes > MAX_BYTES:
                    raise LibraryError(
                        '%s holds more than %d files or %d GB: too much for one driver '
                        "library. Drop the library's own folder."
                        % (os.path.basename(root.rstrip(os.sep)) or root, MAX_FILES,
                           MAX_BYTES // 10**9))
                if in_build:
                    if _is_program(path):
                        plan.programs.append(rel)
                    continue
                low = f.lower()
                if low.endswith(_SOURCE_EXTS):
                    fic, main = _mentions_fic(path)
                    if fic:
                        plan.fic_sources.append(rel)
                        if main:
                            plan.drivers.append(rel)
                elif low.startswith('build') and low.endswith('.sh'):
                    scripts.append(rel)
                elif f == 'CMakeLists.txt' and _cmake_project(path):
                    cmake_roots.append(rel_dir)
                elif f in ('Makefile', 'makefile', 'GNUmakefile'):
                    makefiles.append(rel_dir)
                elif f in ('setup.py', 'pyproject.toml'):
                    pythons.append(rel_dir)
        plan.drivers.sort(key=lambda d: (bool(_TOOL_RE.search(os.path.basename(d))), d))
        plan.builds = _build_plans(scripts, cmake_roots, makefiles, pythons, plan.drivers
                                   or plan.fic_sources)
        _review(plan)
        return plan
    except Exception:
        if temp:
            shutil.rmtree(temp, ignore_errors=True)
        raise


def _safe_name(name):
    return re.sub(r'[^A-Za-z0-9_.-]+', '_', name).strip('_.') or 'driver'


def _inside(path, directory):
    directory = os.path.normpath(directory)
    return directory in ('.', '') or path == directory or path.startswith(directory + os.sep)


def _build_plans(scripts, cmake_roots, makefiles, pythons, driver_sources):
    plans = []
    for s in scripts:
        # A script in scripts/ is run from the folder above it (Kinova's
        # api_cpp/examples runs ./scripts/build-gcc.sh), one beside the
        # project from its own folder.
        folder = os.path.dirname(s)
        run_in = os.path.dirname(folder) if os.path.basename(folder) == 'scripts' else folder
        command = './' + os.path.relpath(s, run_in or '.')
        plans.append(BuildPlan('script', run_in or '.', command,
                               'Build script: %s' % s))
    # The top CMake projects only; subprojects are built by their parent.
    tops = [c for c in cmake_roots
            if not any(o != c and _inside(c, o) for o in cmake_roots)]
    for c in sorted(tops, key=lambda p: (p.count(os.sep), p)):
        plans.append(BuildPlan('cmake', os.path.join(c, 'build') if c != '.' else 'build',
                               'cmake .. -DCMAKE_BUILD_TYPE=Release && make %s' % _JOBS,
                               'CMake project: %s' % (c if c != '.' else 'the top folder')))
    for m in makefiles:
        if not any(_inside(m, c) for c in cmake_roots):
            plans.append(BuildPlan('make', m, 'make %s' % _JOBS,
                                   'Makefile: %s' % (m if m != '.' else 'the top folder')))
    for p in pythons:
        plans.append(BuildPlan('python', p, 'python3 -m pip install --user -e .',
                               'Python package: %s' % (p if p != '.' else 'the top folder')))
    order = {'script': 0, 'cmake': 1, 'make': 2, 'python': 3}
    for plan in plans:
        base = plan.directory if plan.kind != 'cmake' else os.path.dirname(plan.directory)
        sources = [s for s in driver_sources if s.endswith('.py') == (plan.kind == 'python')]
        plan.covers_driver = any(_inside(s, base or '.') for s in sources)
    # What builds the driver first; a script before the CMake it wraps;
    # shallow before deep.
    return sorted(plans, key=lambda p: (not p.covers_driver, order[p.kind],
                                        p.directory.count(os.sep), p.directory))


def _review(plan):
    msgs = []
    if not plan.builds:
        msgs.append((WARN, 'No build script, CMake project, Makefile or Python package was '
                     'found. Fill in the build command yourself, or leave it empty if the '
                     'driver is already built.'))
    if plan.drivers:
        msgs.append((INFO, "Driver program%s using FIC's shared memory: %s."
                     % ('' if len(plan.drivers) == 1 else 's',
                        ', '.join(plan.drivers[:3]) + ('…' if len(plan.drivers) > 3 else ''))))
    elif plan.fic_sources:
        msgs.append((INFO, "FIC's shared memory is used in %s."
                     % ', '.join(plan.fic_sources[:3])))
    else:
        msgs.append((WARN, "Nothing here uses FIC's shared memory (fic/shared_params.h). A "
                     'vendor SDK alone cannot drive the robot from FIC: its driver program '
                     'must read commanded torques from, and write joint states to, '
                     '/robot_feedback_memory. See Adding a Robot → Real robot in the docs.'))
    if plan.build and not plan.build.covers_driver and plan.drivers:
        msgs.append((WARN, 'The chosen build does not contain %s; check that it builds the '
                     'driver.' % plan.drivers[0]))
    if plan.in_place:
        msgs.append((INFO, 'Already in robot_library: used where it is, nothing copied.'))
    plan.messages = msgs


# ── Commands ──────────────────────────────────────────────────────────────

def guess_program(plan, build=None, source=None):
    """Guess the program the build will make for a driver ('' if none).

    ``source`` is one of ``plan.drivers`` (the first by default). CMake
    names targets freely, so this is a guess (the source's stem, in the
    build folder); ``find_programs`` settles it after the first build.
    """
    build = build or plan.build
    src = source or (plan.drivers[0] if plan.drivers else '')
    if not src:
        return ''
    stem = os.path.splitext(os.path.basename(src))[0].lower()
    built = [p for p in plan.programs if os.path.basename(p).lower().endswith(stem)]
    if built:                         # built already: no guessing
        return built[0]
    stem = os.path.splitext(os.path.basename(src))[0]
    if src.endswith('.py'):
        return src
    if build is None:
        return stem
    if build.kind == 'cmake':
        return os.path.normpath(os.path.join(build.directory, stem))
    if build.kind == 'script':
        return os.path.normpath(os.path.join(build.directory, 'build', stem))
    return os.path.normpath(os.path.join(os.path.dirname(src), stem))


def program_choices(plan, build=None):
    """Programs to offer as the driver: a guess per driver source, then built ones."""
    out = []
    for src in plan.drivers or ['']:
        guess = guess_program(plan, build, src)
        if guess and guess not in out:
            out.append(guess)
    for p in plan.programs:
        if p not in out:
            out.append(p)
    return out


def driver_command(name, program, args=''):
    """``cd {robot_lib}/<name>/<folder> && ./<program> args`` (python for .py)."""
    if not program:
        return ''
    folder, exe = os.path.split(os.path.normpath(program))
    where = '{robot_lib}/%s%s' % (name, '/' + folder if folder else '')
    run = ('python3 %s' % exe) if exe.endswith('.py') else './%s' % exe
    return ('cd %s && %s %s' % (where, run, args)).strip()


def _stems(drivers):
    return [os.path.splitext(os.path.basename(d))[0].lower() for d in drivers]


def pick_program(lib_dir, drivers=(), since=None):
    """Return the program a build made for the driver, or '' when unclear.

    One named after a driver source wins; otherwise only a build that made
    a single program is trusted (libfranka's makes twenty examples).
    """
    progs = find_programs(lib_dir, drivers, since)
    stems = _stems(drivers)
    named = [p for p in progs if any(os.path.basename(p).lower().endswith(s) for s in stems)]
    if named:
        return named[0]
    return progs[0] if len(progs) == 1 else ''


def find_programs(lib_dir, drivers=(), since=None):
    """List the executables under ``lib_dir``, the likeliest driver first.

    A program whose name ends with a driver source's stem ranks first
    (Kinova's build names 00-Torque_Controller.cpp's program
    01-torque-control_00-Torque_Controller), then newer ones -- ``since``
    (a timestamp) drops anything older, i.e. not from the last build.
    """
    stems = _stems(drivers)
    found = []
    for dirpath, dirnames, filenames in os.walk(lib_dir):
        dirnames[:] = [d for d in dirnames if d not in _SKIP_DIRS]
        for f in filenames:
            path = os.path.join(dirpath, f)
            if not _is_program(path):
                continue
            mtime = os.path.getmtime(path)
            if since is not None and mtime < since:
                continue
            rel = os.path.relpath(path, lib_dir)
            match = next((k for k, s in enumerate(stems) if f.lower().endswith(s)), len(stems))
            found.append((match, -mtime, rel.count(os.sep), rel))
    return [rel for *_, rel in sorted(found)]


# ── Copying it in ─────────────────────────────────────────────────────────

def install_library(plan, lib_root, replace=False, progress=None):
    """Copy the library into ``lib_root/<plan.name>``; return that folder.

    Copied into a hidden folder first and renamed into place, so an
    interrupted copy never looks like a library. ``progress(bytes_done)``
    is called as files arrive (from the copying thread, if one); raising
    from it stops the copy and removes what was copied.
    """
    if plan.in_place:
        return plan.root
    os.makedirs(lib_root, exist_ok=True)
    target = os.path.join(lib_root, plan.name)
    if os.path.exists(target) and not replace:
        raise LibraryError('robot_library/%s already exists. Pick another name, or replace it.'
                           % plan.name)
    staging = os.path.join(lib_root, '.%s.partial' % plan.name)
    shutil.rmtree(staging, ignore_errors=True)
    done = [0]

    def copy(src, dst):
        shutil.copy2(src, dst)
        done[0] += os.path.getsize(dst)
        if progress:
            progress(done[0])
    try:
        shutil.copytree(plan.root, staging, symlinks=True, copy_function=copy,
                        ignore=shutil.ignore_patterns(*_SKIP_DIRS - {'.git'}))
        if os.path.exists(target):
            shutil.rmtree(target)
        os.rename(staging, target)
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return target


def name_problem(name):
    """Why ``name`` cannot be a robot_library folder, or ''."""
    if not name:
        return 'Give the library a folder name.'
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*', name):
        return 'A folder name uses letters, digits, _ . and - only.'
    return ''


def users_of(name, configs):
    """List the robots whose driver or build lives in ``robot_library/<name>``."""
    pat = re.compile(r'\{robot_lib\}/%s(/|\s|$)' % re.escape(name))
    return [rid for rid, cfg in configs.items()
            if any(pat.search(str(cfg.get(k) or ''))
                   for k in ('driver_command', 'build_directory', 'build_command'))]


def expand(path, lib_root, robot_name=''):
    """Expand a ``{robot_lib}`` path as the Studio's buttons do."""
    return os.path.expanduser(path.replace('{robot_lib}', lib_root)
                              .replace('{robot_name}', robot_name))
