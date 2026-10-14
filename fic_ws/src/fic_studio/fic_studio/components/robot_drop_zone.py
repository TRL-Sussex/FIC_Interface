"""Drop zones for the Robot Configuration page.

``RobotDropZone`` is where a URDF is dropped to add a robot. A URDF is
required: a ``.urdf`` / ``.xacro`` file, a folder holding one (a ROS
description package, say) or a ``.zip`` of that folder. The robot's own
MuJoCo model (``.xml``) is welcome too: dropped along with the URDF, or kept
in the same folder. Emits ``dropped(path)`` for one item and
``dropped_many(paths)`` for several dropped together.

``LibraryDropZone`` (Hardware tab) takes a robot's driver library: a folder
or a ``.zip`` of one. It emits ``dropped(path)``.
"""
import os

from fic_studio import theme
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtWidgets import (
    QFileDialog, QFrame, QHBoxLayout, QLabel, QMenu, QToolButton, QVBoxLayout,
)

ACCEPTED_EXTS = ('.urdf', '.xacro', '.zip', '.xml')


def accepts(path):
    """Return True for the things the importer can take."""
    return os.path.isdir(path) or path.lower().endswith(ACCEPTED_EXTS)


def accepts_library(path):
    """Return True for a driver library: a folder or a .zip."""
    return os.path.isdir(path) or path.lower().endswith('.zip')


class _DropZone(QFrame):
    """Dashed drop target with a Browse menu; subclasses say what it takes."""

    dropped = pyqtSignal(str)
    dropped_many = pyqtSignal(list)

    ICON = '⤓'
    TITLE = ''
    HINT = ''
    MULTIPLE = True           # several items dropped together are welcome

    def __init__(self, parent=None, compact=False):
        """Build the zone; it highlights while something acceptable hovers."""
        super().__init__(parent)
        self.setAcceptDrops(True)
        self.setObjectName('RobotDropZone')
        self._hover = False

        self._icon = QLabel(self.ICON)
        self._icon.setAlignment(Qt.AlignCenter)
        self._title = QLabel(self.TITLE)
        self._hint = QLabel(self.HINT)
        self._hint.setWordWrap(True)
        self.browse_btn = QToolButton()
        self.browse_btn.setText('Browse…')
        self.browse_btn.setCursor(Qt.PointingHandCursor)
        self.browse_btn.setPopupMode(QToolButton.InstantPopup)
        menu = QMenu(self.browse_btn)
        for label, slot in self._menu_actions():
            menu.addAction(label, slot)
        self.browse_btn.setMenu(menu)

        if compact:
            # One row: icon, title over hint, Browse.
            lay = QHBoxLayout(self)
            lay.setContentsMargins(12, 8, 12, 8)
            lay.setSpacing(10)
            text = QVBoxLayout()
            text.setSpacing(1)
            text.addWidget(self._title)
            text.addWidget(self._hint)
            lay.addWidget(self._icon, 0, Qt.AlignVCenter)
            lay.addLayout(text, 1)
            lay.addWidget(self.browse_btn, 0, Qt.AlignVCenter)
        else:
            lay = QVBoxLayout(self)
            lay.setContentsMargins(12, 14, 12, 12)
            lay.setSpacing(4)
            for w in (self._title, self._hint):
                w.setAlignment(Qt.AlignCenter)
            for w in (self._icon, self._title, self._hint):
                lay.addWidget(w)
            lay.addWidget(self.browse_btn, 0, Qt.AlignHCenter)

        theme.bind(self._icon, lambda t: 'color: %s; font-size: %dpx; border: none;'
                   % (t['accent_blue'], 18 if compact else 22))
        theme.bind(self._title, lambda t: 'color: %s; font-size: %dpx; font-weight: 700;'
                   ' border: none;' % (t['text_primary'], 12 if compact else 13))
        theme.bind(self._hint, lambda t: 'color: %s; font-size: 11px; border: none;'
                   % t['text_secondary'])
        theme.bind(self.browse_btn, lambda t: (
            'QToolButton { color: %s; background: transparent; border: 1px solid %s;'
            ' border-radius: 5px; padding: 3px 12px; font-size: 11px; }'
            'QToolButton:hover { background: %s; }'
            'QToolButton::menu-indicator { image: none; }'
        ) % (t['accent_blue'], t['border_medium'], t['bg_hover']))
        theme.bind(self, self._style)

    # ── What it takes ────────────────────────────────────────────────────
    def accepts(self, path):
        """Return True for something this zone takes."""
        raise NotImplementedError

    def _menu_actions(self):
        return []

    def _browse_default(self):
        """Browse for what the zone takes (a click on it does this)."""

    # ── Styling ──────────────────────────────────────────────────────────
    def _style(self, t):
        return ('QFrame#RobotDropZone { background: %s; border: 2px dashed %s;'
                ' border-radius: 10px; }'
                % (t['bg_hover'] if self._hover else t['bg_raised'],
                   t['accent_blue'] if self._hover else t['border_medium']))

    def _set_hover(self, on):
        if on != self._hover:
            self._hover = on
            self.setStyleSheet(self._style(theme.current()))

    # ── Drag and drop ────────────────────────────────────────────────────
    def _paths_from(self, mime):
        """Return the dropped local paths, if every one is something this zone takes."""
        if not mime.hasUrls():
            return []
        paths = [(u.toLocalFile().rstrip('/') or '/') for u in mime.urls() if u.isLocalFile()]
        if not self.MULTIPLE and len(paths) > 1:
            return []
        return paths if paths and all(self.accepts(p) for p in paths) else []

    def dragEnterEvent(self, event):
        """Accept what the zone takes; ignore the rest."""
        if self._paths_from(event.mimeData()):
            event.acceptProposedAction()
            self._set_hover(True)
        else:
            event.ignore()

    def dragLeaveEvent(self, event):
        """Drop the highlight."""
        self._set_hover(False)
        super().dragLeaveEvent(event)

    def dropEvent(self, event):
        """Emit ``dropped`` with the path, or ``dropped_many`` with several."""
        self._set_hover(False)
        paths = self._paths_from(event.mimeData())
        if not paths:
            return
        event.acceptProposedAction()
        self._emit(paths)

    def _emit(self, paths):
        if len(paths) == 1:
            self.dropped.emit(paths[0])
        elif paths:
            self.dropped_many.emit(paths)

    def mouseReleaseEvent(self, event):
        """Open the file dialog on a click anywhere on the zone."""
        if (event.button() == Qt.LeftButton and self.isEnabled()
                and self.rect().contains(event.pos())):
            self._browse_default()
        super().mouseReleaseEvent(event)


class RobotDropZone(_DropZone):
    """Where a URDF (and, optionally, the robot's MuJoCo model) is dropped."""

    TITLE = 'Add a robot'
    HINT = ('Drop its URDF here (required): a .urdf or .xacro, its folder, or a .zip. '
            'Have a MuJoCo model? Drop its .xml with it, or keep it in the same folder.')

    def accepts(self, path):
        """Return True for a URDF, xacro, MuJoCo .xml, zip or folder."""
        return accepts(path)

    def _menu_actions(self):
        return [('URDF file (and MuJoCo model)…', self._browse_file),
                ('Folder…', self._browse_folder),
                ('Zip file…', self._browse_zip)]

    def _browse_default(self):
        self._browse_file()

    def _browse_file(self):
        paths, _ = QFileDialog.getOpenFileNames(
            self, 'Choose the URDF (and its MuJoCo model, if you have one)',
            os.path.expanduser('~'),
            'Robot descriptions (*.urdf *.xacro *.zip *.xml);;All files (*)')
        self._emit(paths)

    def _browse_folder(self):
        path = QFileDialog.getExistingDirectory(
            self, 'Choose the folder holding the URDF', os.path.expanduser('~'))
        if path:
            self.dropped.emit(path)

    def _browse_zip(self):
        path, _ = QFileDialog.getOpenFileName(
            self, 'Choose a zip of the robot description', os.path.expanduser('~'),
            'Zip files (*.zip)')
        if path:
            self.dropped.emit(path)


class LibraryDropZone(_DropZone):
    """Where a robot's driver library (vendor SDK + FIC bridge) is dropped."""

    ICON = '+'
    TITLE = 'Add the driver library'
    HINT = ("Drop the robot's driver folder or its .zip here: it is copied into "
            'robot_library and the commands below are filled in.')
    MULTIPLE = False

    def __init__(self, parent=None, compact=True):
        """Build the zone (one row by default: it sits inside a card)."""
        super().__init__(parent, compact=compact)

    def accepts(self, path):
        """Return True for a folder or a .zip."""
        return accepts_library(path)

    def _menu_actions(self):
        return [('Folder…', self._browse_folder), ('Zip file…', self._browse_zip)]

    def _browse_default(self):
        self._browse_folder()

    def _browse_folder(self):
        path = QFileDialog.getExistingDirectory(
            self, "Choose the robot's driver library folder", os.path.expanduser('~'))
        if path:
            self.dropped.emit(path)

    def _browse_zip(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Choose a zip of the robot's driver library", os.path.expanduser('~'),
            'Zip files (*.zip)')
        if path:
            self.dropped.emit(path)
