from pathlib import Path
from typing import List

from .manifest import ExtensionManifest


def discover_extensions(extensions_root: Path) -> List[ExtensionManifest]:
    # Folders without manifest.json are skipped — they may be runtime helpers,
    # not GUI extensions.
    results: List[ExtensionManifest] = []
    if not extensions_root.exists():
        return results
    for child in sorted(extensions_root.iterdir()):
        if not child.is_dir() or child.name.startswith(("_", ".")):
            continue                      # _template, _shared, …: not extensions
        manifest_path = child / "manifest.json"
        if not manifest_path.exists():
            continue
        try:
            results.append(ExtensionManifest.from_file(manifest_path))
        except Exception as e:
            print(f"[extensions] Skipping {child.name}: {e}")
    return results
