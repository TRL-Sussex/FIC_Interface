import importlib
import sys
from pathlib import Path
from typing import Callable, Dict, List

from .base import Extension
from .manifest import ExtensionManifest


class ExtensionLoadError(Exception):
    pass


class LoadedExtension:
    def __init__(self, manifest: ExtensionManifest, instance: Extension):
        self.manifest = manifest
        self.instance = instance


class ExtensionLoader:
    def __init__(self, extensions_root: Path):
        self._root = extensions_root
        self._loaded: Dict[str, LoadedExtension] = {}

    def _import_entry(self, manifest: ExtensionManifest):
        # Extensions import each other two ways: by folder name ("gaze.push_turn",
        # needs Extensions/ on the path) and as a package ("Extensions._shared...",
        # needs the folder above it). The Studio runs from fic_ws/, so add both;
        # otherwise the package form only worked when another extension that
        # happened to add the repository root had loaded first.
        for path in (str(self._root.parent), str(self._root)):
            if path not in sys.path:
                sys.path.insert(0, path)
        return importlib.import_module(manifest.entry_module)

    def load(
        self,
        manifest: ExtensionManifest,
        host_factory: Callable[[str], object],
    ) -> Extension:
        if manifest.id in self._loaded:
            return self._loaded[manifest.id].instance
        try:
            module = self._import_entry(manifest)
        except Exception as e:
            raise ExtensionLoadError(f"failed to import {manifest.id}: {e}") from e
        ExtClass = getattr(module, "Extension", None)
        if (
            ExtClass is None
            or not isinstance(ExtClass, type)
            or not issubclass(ExtClass, Extension)
        ):
            raise ExtensionLoadError(
                f"{manifest.id}: {manifest.entry_module} must export class Extension(Extension)"
            )
        host = host_factory(manifest.id)
        try:
            instance = ExtClass(host)
        except Exception as e:
            raise ExtensionLoadError(f"failed to instantiate {manifest.id}: {e}") from e
        self._loaded[manifest.id] = LoadedExtension(manifest, instance)
        return instance

    def load_enabled(
        self,
        manifests: List[ExtensionManifest],
        enabled_ids: List[str],
        host_factory: Callable[[str], object],
    ) -> Dict[str, Extension]:
        loaded: Dict[str, Extension] = {}
        for m in manifests:
            if m.id not in enabled_ids:
                continue
            try:
                loaded[m.id] = self.load(m, host_factory)
            except ExtensionLoadError as e:
                print(f"[extensions] {e}")
        return loaded

    def teardown_all(self) -> None:
        for loaded in list(self._loaded.values()):
            try:
                loaded.instance.teardown()
            except Exception as e:
                print(f"[extensions] teardown failed for {loaded.manifest.id}: {e}")
        self._loaded.clear()

    def loaded(self) -> Dict[str, LoadedExtension]:
        return dict(self._loaded)
