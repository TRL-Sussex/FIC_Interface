"""Find the mesh files a URDF refers to, wherever they actually are.

URDFs name meshes three ways: ``package://pkg/path``, a path relative to the
URDF, or an absolute path. Exported robots rarely arrive with the ROS package
installed, so after the obvious places this searches the folders the user gave
us, by path suffix and finally by file name (case-insensitively: CAD
exporters write ``LINK1.STL``).
"""
from dataclasses import dataclass
import os
from typing import Dict, List, Optional

#: Formats MuJoCo can load directly.
MUJOCO_EXTS = ('.stl', '.obj', '.msh')
#: Formats the importer can convert for MuJoCo.
CONVERTIBLE_EXTS = ('.dae',)

_MAX_INDEXED_FILES = 50000
_SKIP_DIRS = {'.git', 'build', 'install', 'log', '__pycache__', 'node_modules'}


@dataclass
class ResolvedMesh:
    ref: object            # urdf_info.MeshRef
    path: Optional[str]    # None when not found

    @property
    def ext(self):
        return os.path.splitext(self.path or self.ref.uri)[1].lower()

    @property
    def found(self):
        return self.path is not None


class MeshFinder:
    """Resolves mesh URIs against a URDF location and extra search folders."""

    def __init__(self, urdf_path, search_roots=(), packages=None):
        self.urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
        self.packages = dict(packages or {})     # ROS package name -> folder
        roots = [os.path.abspath(r) for r in search_roots if r and os.path.isdir(r)]
        # The URDF's own folder and its parent cover the usual
        # <pkg>/urdf/robot.urdf + <pkg>/meshes/ layout.
        for extra in (self.urdf_dir, os.path.dirname(self.urdf_dir)):
            if extra not in roots:
                roots.append(extra)
        self.roots = roots
        self._index: Optional[Dict[str, List[str]]] = None
        self._share_cache: Dict[str, Optional[str]] = {}

    # ── Search index ─────────────────────────────────────────────────────
    def _build_index(self):
        index: Dict[str, List[str]] = {}
        count = 0
        for root in self.roots:
            for dirpath, dirnames, filenames in os.walk(root):
                dirnames[:] = [d for d in dirnames if d not in _SKIP_DIRS]
                for name in filenames:
                    index.setdefault(name.lower(), []).append(os.path.join(dirpath, name))
                    count += 1
                    if count >= _MAX_INDEXED_FILES:
                        self._index = index
                        return
        self._index = index

    def _by_name(self, relpath):
        """Files matching ``relpath``'s name, best path-suffix match first."""
        if self._index is None:
            self._build_index()
        name = os.path.basename(relpath).lower()
        hits = self._index.get(name, [])
        want = [p for p in relpath.replace('\\', '/').lower().split('/') if p and p != '..']

        def suffix_len(path):
            have = path.replace('\\', '/').lower().split('/')
            n = 0
            while n < len(want) and n < len(have) and want[-1 - n] == have[-1 - n]:
                n += 1
            return n
        return sorted(dict.fromkeys(hits), key=lambda p: (-suffix_len(p), len(p)))

    # ── Resolution ───────────────────────────────────────────────────────
    def resolve(self, uri):
        """Absolute path of the mesh ``uri`` names, or None."""
        if uri.startswith('file://'):
            uri = uri[len('file://'):]
        if uri.startswith('package://'):
            pkg, _, rel = uri[len('package://'):].partition('/')
            for candidate in self._package_candidates(pkg, rel):
                if os.path.isfile(candidate):
                    return os.path.abspath(candidate)
            hits = self._by_name(rel)
            return hits[0] if hits else None
        if os.path.isabs(uri):
            if os.path.isfile(uri):
                return uri
            # Written on another machine: .../share/<pkg>/meshes/x.stl. Try
            # the same file in that package here, deepest package name first.
            parts = [p for p in uri.split('/') if p]
            for i in range(len(parts) - 2, -1, -1):
                share = self._share_dir(parts[i])
                if share:
                    candidate = os.path.join(share, *parts[i + 1:])
                    if os.path.isfile(candidate):
                        return candidate
            hits = self._by_name(uri)
            return hits[0] if hits else None
        local = os.path.normpath(os.path.join(self.urdf_dir, uri))
        if os.path.isfile(local):
            return local
        hits = self._by_name(uri)
        return hits[0] if hits else None

    def _share_dir(self, pkg):
        """Folder of ROS package ``pkg``: one we were given, else an installed one."""
        if pkg in self.packages:
            return self.packages[pkg]
        if pkg not in self._share_cache:
            try:
                from ament_index_python.packages import get_package_share_directory
                self._share_cache[pkg] = get_package_share_directory(pkg)
            except Exception:
                self._share_cache[pkg] = None
        return self._share_cache[pkg]

    def _package_candidates(self, pkg, rel):
        share = self._share_dir(pkg)
        if share:
            yield os.path.join(share, rel)
        # A folder named like the package, above the URDF or in the roots.
        seen = set()
        for start in [self.urdf_dir] + self.roots:
            cur = start
            for _ in range(8):
                if os.path.basename(cur) == pkg and cur not in seen:
                    seen.add(cur)
                    yield os.path.join(cur, rel)
                candidate = os.path.join(cur, pkg)
                if os.path.isdir(candidate) and candidate not in seen:
                    seen.add(candidate)
                    yield os.path.join(candidate, rel)
                parent = os.path.dirname(cur)
                if parent == cur:
                    break
                cur = parent

    def resolve_all(self, refs):
        return [ResolvedMesh(ref, self.resolve(ref.uri)) for ref in refs]
