# config.py
import os
from pathlib import Path

# ---------------------- CONFIG ----------------------
# Project root resolution: find the `fic_ws` workspace directory in this file's
# path; the repo root is its parent. This works whether fic_studio is imported
# from the source tree (fic_ws/src/fic_studio/fic_studio/config.py) OR from a
# colcon copy-install tree (fic_ws/install/fic_studio/lib/pythonX/site-packages/
# fic_studio/config.py) — the latter happens on macOS/RoboStack where
# `colcon --symlink-install` isn't supported by modern setuptools. A fixed
# parents[N] walk would land inside install/ and break. Override with FIC_ROOT.
def _resolve_repo_root() -> Path:
    here = Path(__file__).resolve()
    parts = here.parts
    if "fic_ws" in parts:
        idx = parts.index("fic_ws")   # first occurrence = the real workspace
        if idx > 0:
            return Path(*parts[:idx])
    return here.parents[4]            # fallback: original source-layout walk

_DEFAULT_ROOT = _resolve_repo_root()
FIC_INTERFACE_DIR = Path(os.environ.get("FIC_ROOT", str(_DEFAULT_ROOT))).resolve()
FIC_ROOT = FIC_INTERFACE_DIR

# Subdirectory roots — strings, since most callers feed them to os.path / subprocess.
WS_PATH          = str(FIC_INTERFACE_DIR / "fic_ws")
ROBOT_LIB        = str(FIC_INTERFACE_DIR / "robot_library")
FIC_API_DIR      = str(FIC_INTERFACE_DIR / "fic_api")
FIC_DOCS         = str(FIC_INTERFACE_DIR / "fic_docs")
EXPERIMENTS_ROOT = str(FIC_INTERFACE_DIR / "experiments")
FIC_LIBRARY      = str(FIC_INTERFACE_DIR / "fic_library")
FIC_CONFIG_DIR   = str(FIC_INTERFACE_DIR / "fic_library" / "config")

XML_FILE_PATH = FIC_INTERFACE_DIR / "fic_library" / "config" / "config.xml"
