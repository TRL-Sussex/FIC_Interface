"""A small web browser for the container's desktop.

The Studio opens its documentation (Help) and some links with the system
browser. The container has none, so $BROWSER points here: one window with
back, forward, reload and an address bar, on Qt WebEngine.

Local pages (a path, or a file:// URL) are served over HTTP from their folder
by a small server inside this process, because pages that load their content
with fetch(), like the FIC documentation, do not work from file:// URLs.
Folders are opened in the file manager instead.
"""

import functools
import http.server
import os
import subprocess
import sys
import threading
from urllib.parse import quote, unquote, urlsplit

# Software rendering inside the container; the Chromium sandbox needs kernel
# features Docker does not grant, and the container is the sandbox anyway.
os.environ.setdefault(
    "QTWEBENGINE_CHROMIUM_FLAGS",
    "--no-sandbox --disable-gpu --disable-gpu-compositing --disable-dev-shm-usage",
)
os.environ["QT_XCB_GL_INTEGRATION"] = "xcb_glx"

from PyQt5.QtCore import Qt, QCoreApplication, QUrl  # noqa: E402
from PyQt5.QtGui import QIcon  # noqa: E402
from PyQt5.QtWebEngineWidgets import QWebEnginePage, QWebEngineProfile, QWebEngineView  # noqa: E402
from PyQt5.QtWidgets import (  # noqa: E402
    QAction, QApplication, QLineEdit, QMainWindow, QStyle, QToolBar,
)


class _QuietHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *args):
        pass


def serve_folder(folder):
    """Serve `folder` on a free local port; return the base URL."""
    handler = functools.partial(_QuietHandler, directory=folder)
    server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return f"http://127.0.0.1:{server.server_address[1]}/"


def local_path(text):
    """The filesystem path `text` refers to, or None if it is not local."""
    if text.startswith("file://"):
        return unquote(urlsplit(text).path)
    if "://" not in text and os.path.exists(text):
        return os.path.abspath(text)
    return None


def to_url(text):
    text = text.strip()
    path = local_path(text)
    if path and os.path.isfile(path):
        folder, name = os.path.split(path)
        fragment = urlsplit(text).fragment if text.startswith("file://") else ""
        return QUrl(serve_folder(folder) + quote(name) + (f"#{fragment}" if fragment else ""))
    if "://" in text or text.startswith("about:"):
        return QUrl(text)
    return QUrl("http://" + text)


class Browser(QMainWindow):
    def __init__(self, url):
        super().__init__()
        self.resize(1200, 850)
        # A private, in-memory profile: several browser windows can be open
        # at once without competing for one on-disk cookie store.
        self.profile = QWebEngineProfile(self)
        self.view = QWebEngineView(self)
        self.view.setPage(QWebEnginePage(self.profile, self.view))
        self.setCentralWidget(self.view)

        bar = QToolBar(self)
        bar.setMovable(False)
        self.addToolBar(bar)
        style = self.style()
        for icon, tip, slot in (
            (QStyle.SP_ArrowBack, "Back", self.view.back),
            (QStyle.SP_ArrowForward, "Forward", self.view.forward),
            (QStyle.SP_BrowserReload, "Reload", self.view.reload),
        ):
            action = QAction(style.standardIcon(icon), tip, self)
            action.triggered.connect(slot)
            bar.addAction(action)

        self.address = QLineEdit(self)
        self.address.returnPressed.connect(
            lambda: self.view.setUrl(to_url(self.address.text())))
        bar.addWidget(self.address)

        self.view.urlChanged.connect(lambda u: self.address.setText(u.toString()))
        self.view.titleChanged.connect(
            lambda t: self.setWindowTitle(f"{t} — FIC Browser" if t else "FIC Browser"))
        self.view.setUrl(url)


def main():
    target = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.environ.get("FIC_ROOT", "/opt/fic"), "fic_docs", "index.html")

    path = local_path(target)
    if path and os.path.isdir(path):
        subprocess.Popen(["pcmanfm", path], start_new_session=True)
        return

    QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
    app = QApplication(sys.argv[:1])
    app.setApplicationName("FIC Browser")
    app.setDesktopFileName("fic-browser")
    app.setWindowIcon(QIcon.fromTheme("help-browser"))
    window = Browser(to_url(target))
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
