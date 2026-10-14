"""ExtensionLoader: an extension that imports ``Extensions.<pkg>`` loads on its own.

The Studio runs from fic_ws/, so the repository root is not on sys.path. Before the
fix, ``from Extensions._shared... import`` only worked if another extension that
adds the repository root had already been loaded (the VR extensions, Teach Mode,
Tracker and others silently failed to appear).
"""
import subprocess
import sys
import textwrap
from pathlib import Path

STUDIO_SRC = Path(__file__).resolve().parents[1]


def test_package_style_import_loads_without_help(tmp_path):
    ext_root = tmp_path / "repo" / "Extensions"
    (ext_root / "_shared").mkdir(parents=True)
    (ext_root / "_shared" / "__init__.py").write_text("")
    (ext_root / "_shared" / "helper.py").write_text("VALUE = 42\n")
    demo = ext_root / "demo"
    demo.mkdir()
    (demo / "__init__.py").write_text("")
    (demo / "manifest.json").write_text('{"id": "demo", "name": "Demo"}')
    (demo / "extension.py").write_text(textwrap.dedent("""
        from Extensions._shared.helper import VALUE
        from fic_studio.extensions.base import Extension as _Base

        class Extension(_Base):
            def __init__(self, host):
                self.value = VALUE

            def build_page(self):
                return None

            def teardown(self):
                pass
    """))
    elsewhere = tmp_path / "fic_ws"
    elsewhere.mkdir()
    script = textwrap.dedent(f"""
        import sys
        sys.path.insert(0, {str(STUDIO_SRC)!r})
        from pathlib import Path
        from fic_studio.extensions.manifest import ExtensionManifest
        from fic_studio.extensions.loader import ExtensionLoader
        root = Path({str(ext_root)!r})
        m = ExtensionManifest.from_file(root / "demo" / "manifest.json")
        loaded = ExtensionLoader(root).load_enabled([m], ["demo"], host_factory=lambda eid: None)
        print(loaded["demo"].value)
    """)
    out = subprocess.run([sys.executable, "-c", script], cwd=elsewhere,
                         capture_output=True, text=True, timeout=60)
    assert out.returncode == 0, out.stderr
    assert out.stdout.strip().splitlines()[-1] == "42", out.stdout + out.stderr
