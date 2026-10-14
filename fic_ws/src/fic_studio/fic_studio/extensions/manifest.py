import json
from dataclasses import dataclass, field
from pathlib import Path


@dataclass(frozen=True)
class ExtensionManifest:
    id: str
    name: str
    description: str
    entry_module: str
    folder: Path
    sidebar_icon: str = ""
    sidebar_section: str = "Extensions"
    #: The extension's own web page, if it has one: ``{"page": "page.js",
    #: "css": "page.css", "poll_ms": 250, "local_only": [action, ...]}``, files
    #: relative to the extension's ``web/`` folder. Empty: the web UI mirrors
    #: its Qt page instead.
    web: dict = field(default_factory=dict, compare=False, hash=False)

    @classmethod
    def from_file(cls, manifest_path: Path) -> "ExtensionManifest":
        data = json.loads(manifest_path.read_text())
        folder = manifest_path.parent
        return cls(
            id=data["id"],
            name=data["name"],
            description=data.get("description", ""),
            entry_module=data.get("entry_module", f"{folder.name}.extension"),
            folder=folder,
            sidebar_icon=data.get("sidebar_icon", ""),
            sidebar_section=data.get("sidebar_section", "Extensions"),
            web=_web_block(data.get("web")),
        )


def _web_block(raw):
    """Check and normalise a manifest's ``web`` block; {} when there is none."""
    if not raw:
        return {}
    if not isinstance(raw, dict) or not isinstance(raw.get("page"), str) or not raw["page"]:
        raise ValueError('"web" needs at least {"page": "page.js"}')
    out = {"page": raw["page"]}
    if isinstance(raw.get("css"), str) and raw["css"]:
        out["css"] = raw["css"]
    try:
        out["poll_ms"] = max(100, int(raw.get("poll_ms", 250)))
    except (TypeError, ValueError):
        out["poll_ms"] = 250
    out["local_only"] = [str(a) for a in (raw.get("local_only") or [])]
    return out
