# qt_webengine.py — QtWebEngine start-up helpers shared by the entry points
"""
QtWebEngine has two start-up rules and one common install trap.

* ``Qt::AA_ShareOpenGLContexts`` must be set, and ``QtWebEngineWidgets``
  imported, before the ``QApplication`` exists. Doing it later gives the
  warning "Qt WebEngine seems to be initialized from a plugin".
* It has to be the build that matches the system Qt: on Ubuntu 24.04 with ROS
  Jazzy that is the apt package ``python3-pyqt5.qtwebengine``. A pip
  ``PyQtWebEngine`` in ``~/.local`` shadows it and fails with an
  "undefined symbol" error.

This module imports no PyQt at load time, so the splash is not slowed.
"""

APT_PACKAGE = 'python3-pyqt5.qtwebengine'
# Platforms with no OpenGL (CI, tests): sharing GL contexts aborts QtWebEngine there, so only the early import is done.
HEADLESS_PLATFORMS = ('offscreen', 'minimal', 'vnc', 'linuxfb')
PIP_WHEELS = 'PyQtWebEngine PyQtWebEngine-Qt5 PyQt5 PyQt5-Qt5 PyQt5-sip'


def prepare():
    """Call before ``QApplication(...)``. Returns True if QtWebEngine imported."""
    import os
    from PyQt5.QtCore import Qt, QCoreApplication
    if os.environ.get('QT_QPA_PLATFORM', '') not in HEADLESS_PLATFORMS:
        QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
    try:
        import PyQt5.QtWebEngineWidgets  # noqa: F401
    except ImportError:
        return False
    return True


def fix_instructions():
    """The exact commands that repair a broken QtWebEngine."""
    return ('sudo apt install %s\n'
            'pip uninstall %s     # the pip wheels in ~/.local shadow the system Qt'
            % (APT_PACKAGE, PIP_WHEELS))


def error_text(exc):
    return ('FIC Studio could not load its web interface (QtWebEngine):\n\n%s\n\n'
            'To fix it, run:\n\n%s\n\nThen start FIC Studio again. '
            'You can open the classic interface for now.' % (exc, fix_instructions()))
