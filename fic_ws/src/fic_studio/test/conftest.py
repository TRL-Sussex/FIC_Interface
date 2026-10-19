"""Pytest fixtures for headless Qt tests (no pytest-qt available)."""
import os
import tempfile

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
# Tests never write to the machine's real audit log (~/.fic_studio/audit.log).
os.environ['FIC_STUDIO_AUDIT'] = os.path.join(tempfile.mkdtemp(prefix='fic_audit_'), 'audit.log')

from PyQt5.QtWidgets import QApplication  # noqa: E402
import pytest  # noqa: E402


@pytest.fixture(scope='session')
def qapp():
    """Return a process-wide offscreen QApplication."""
    app = QApplication.instance() or QApplication([])
    yield app
