from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

from PyQt5.QtWidgets import QWidget

if TYPE_CHECKING:
    from .host import ExtensionHost


class Extension(ABC):
    # Class-level metadata. The same values must also appear in manifest.json so
    # discovery can list disabled extensions without importing them.
    ID: str = ""
    NAME: str = ""
    DESCRIPTION: str = ""
    SIDEBAR_ICON: str = ""
    SIDEBAR_SECTION: str = "Extensions"

    def __init__(self, host: "ExtensionHost"):
        self.host = host

    def build_page(self) -> QWidget:
        """The extension's page in the classic Qt window.

        Optional when the extension has a web page (manifest ``web``): the
        classic window then says where to find it.
        """
        from PyQt5.QtCore import Qt
        from PyQt5.QtWidgets import QLabel
        label = QLabel('%s has a web page.\n\nOpen it in the web interface '
                       '(start the Studio with --web-ui, or use the web remote).'
                       % (self.NAME or self.ID))
        label.setAlignment(Qt.AlignCenter)
        label.setWordWrap(True)
        return label

    @abstractmethod
    def teardown(self) -> None:
        ...

    def on_activate(self) -> None:
        pass

    def on_emergency_stop(self) -> None:
        pass

    # ── Web page (manifest "web") ─────────────────────────────────────────
    def web_state(self) -> dict:
        """The live values the extension's web page shows (JSON-able).

        Called on the Qt thread, as often as the manifest's ``poll_ms`` while
        the page is open. Keep it quick: read, do not compute or wait.
        """
        return {}

    def web_action(self, name: str, args: dict):
        """Run one action the web page asked for; return a JSON-able result.

        Called on the Qt thread, only for a client that holds the control lease
        (and, for actions the manifest lists in ``web.local_only``, only from
        the Studio's own window). Raise ``ValueError("why")`` to refuse it.
        """
        raise ValueError('No such action: %s' % name)
