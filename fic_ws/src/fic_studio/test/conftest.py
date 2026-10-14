"""Pytest fixtures for headless Qt tests (no pytest-qt available)."""
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication  # noqa: E402
import pytest  # noqa: E402


@pytest.fixture(scope='session')
def qapp():
    """Return a process-wide offscreen QApplication."""
    app = QApplication.instance() or QApplication([])
    yield app
