"""DriverLibraryDialog -- the one screen between dropping a driver library and using it.

Shows where the library will go in robot_library, how it builds and which
program drives the robot -- all worked out by ``driver_lib.analyse_library``
-- and lets the user change any of it. It copies and builds nothing itself:
on accept, ``settings`` says what to do and the Robot Configuration page
does it.
"""
import os

from fic_studio import theme
from fic_studio.components.robot_import_dialog import notes_widget
from fic_studio.robot_import import driver_lib
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QCheckBox, QComboBox, QDialog, QFrame, QGridLayout, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QScrollArea, QVBoxLayout, QWidget,
)

NO_BUILD = 'None: it is already built'


def _muted(text, size=11):
    lbl = QLabel(text)
    lbl.setWordWrap(True)
    theme.bind(lbl, lambda t: 'color: %s; font-size: %dpx;' % (t['text_secondary'], size))
    return lbl


def _heading(text):
    lbl = QLabel(text.upper())
    theme.bind(lbl, lambda t: 'color: %s; font-size: 10px; font-weight: 700;'
               % t['text_disabled'])
    return lbl


def _size(n):
    for unit in ('bytes', 'KB', 'MB', 'GB'):
        if n < 1000 or unit == 'GB':
            return ('%d %s' if unit == 'bytes' else '%.1f %s') % (n, unit)
        n /= 1000.0


class DriverLibraryDialog(QDialog):
    """Review a ``LibraryPlan`` for one robot.

    ``configs`` (robot ID -> config) lets replacing a library another robot
    relies on say so. On accept, ``settings`` holds: name, replace,
    build_directory, build_command, program (relative to the library),
    args, driver_command, compile_now.
    """

    def __init__(self, plan, lib_root, robot_id, current=None, configs=None, parent=None):
        """Build the screen for ``plan``; ``current`` is the robot's driver settings now."""
        super().__init__(parent)
        self.plan = plan
        self.lib_root = lib_root
        self.robot_id = robot_id
        self.current = {k: v for k, v in (current or {}).items() if v}
        self.configs = configs or {}
        self.settings = None
        self._name = plan.name
        self.setWindowTitle('Set up the driver library')
        self.resize(760, 640)
        self.setMinimumSize(620, 480)
        theme.bind(self, lambda t: 'QDialog { background: %s; }' % t['bg_base'])

        outer = QVBoxLayout(self)
        outer.setContentsMargins(20, 18, 20, 16)
        outer.setSpacing(10)
        title = QLabel('Set up the driver library')
        theme.bind(title, lambda t: 'color: %s; font-size: 18px; font-weight: 700;'
                   % t['text_primary'])
        outer.addWidget(title)
        outer.addWidget(_muted('For %s. From %s (%d files, %s).'
                               % (robot_id, plan.source, plan.files, _size(plan.size_bytes))))

        body = QWidget()
        lay = QVBoxLayout(body)
        lay.setContentsMargins(0, 0, 6, 0)
        lay.setSpacing(10)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setWidget(body)
        outer.addWidget(scroll, 1)

        form = QGridLayout()
        form.setHorizontalSpacing(12)
        form.setVerticalSpacing(6)
        self.name_edit = QLineEdit(plan.name)
        self.name_edit.setToolTip('The folder it gets in robot_library')
        self.name_edit.setEnabled(not plan.in_place)
        self.name_edit.textChanged.connect(self._on_name)
        self.build_combo = QComboBox()
        for b in plan.builds:
            self.build_combo.addItem(b.label)
        self.build_combo.addItem(NO_BUILD)
        self.build_combo.currentIndexChanged.connect(self._on_build)
        self.build_dir_edit = QLineEdit()
        self.build_dir_edit.setToolTip('Where the build command runs; {robot_lib} is '
                                       'robot_library')
        self.build_cmd_edit = QLineEdit()
        self.program_combo = QComboBox()
        self.program_combo.setEditable(True)
        self.program_combo.setToolTip('The driver program, relative to the library folder')
        self.program_combo.editTextChanged.connect(self._refresh_command)
        self.args_edit = QLineEdit()
        self.args_edit.setPlaceholderText("What the driver needs, e.g. the robot's IP address")
        self.args_edit.textChanged.connect(self._refresh_command)
        rows = (('Folder in robot_library', self.name_edit),
                ('Build', self.build_combo),
                ('Build directory', self.build_dir_edit),
                ('Build command', self.build_cmd_edit),
                ('Driver program', self.program_combo),
                ('Driver arguments', self.args_edit))
        for r, (label, widget) in enumerate(rows):
            lbl = QLabel(label)
            theme.bind(lbl, lambda t: 'color: %s; font-size: 12px; font-weight: 600;'
                       % t['text_primary'])
            form.addWidget(lbl, r, 0)
            form.addWidget(widget, r, 1)
        form.setColumnStretch(1, 1)
        lay.addLayout(form)

        self.name_note = QLabel('')
        self.name_note.setWordWrap(True)
        theme.bind(self.name_note, lambda t: 'color: %s; font-size: 11px;' % t['accent_red'])
        lay.addWidget(self.name_note)
        self.replace_cb = QCheckBox()
        self.replace_cb.toggled.connect(self._refresh_buttons)
        lay.addWidget(self.replace_cb)

        lay.addWidget(_heading('The robot will run'))
        self.command_lbl = QLabel('')
        self.command_lbl.setWordWrap(True)
        self.command_lbl.setTextInteractionFlags(Qt.TextSelectableByMouse)
        theme.bind(self.command_lbl, lambda t: (
            'QLabel { color: %s; background: %s; border: 1px solid %s; border-radius: 6px;'
            ' padding: 6px 8px; font-family: monospace; font-size: 12px; }'
        ) % (t['text_primary'], t['bg_raised'], t['border_subtle']))
        lay.addWidget(self.command_lbl)
        self.program_note = _muted('')
        lay.addWidget(self.program_note)
        if self.current:
            lay.addWidget(_muted(
                "This replaces %s's driver settings now: %s"
                % (robot_id, '; '.join(self.current[k] for k in
                                       ('driver_command', 'build_directory', 'build_command')
                                       if k in self.current))))

        lay.addWidget(_heading('Notes'))
        lay.addWidget(notes_widget(plan.messages))
        lay.addStretch()

        self.compile_cb = QCheckBox('Build it now (the output is shown as it builds)')
        outer.addWidget(self.compile_cb)
        buttons = QHBoxLayout()
        buttons.addStretch()
        self.cancel_btn = QPushButton('Cancel')
        self.cancel_btn.clicked.connect(self.reject)
        self.ok_btn = QPushButton('Set up')
        self.ok_btn.setObjectName('BtnTeal')
        self.ok_btn.setDefault(True)
        self.ok_btn.clicked.connect(self._accept)
        buttons.addWidget(self.cancel_btn)
        buttons.addWidget(self.ok_btn)
        outer.addLayout(buttons)

        self._on_build(0)
        self._on_name(self.name_edit.text())

    # ── Reacting to edits ─────────────────────────────────────────────────
    def _build(self):
        i = self.build_combo.currentIndex()
        return self.plan.builds[i] if 0 <= i < len(self.plan.builds) else None

    def _on_build(self, _index):
        build = self._build()
        name = self.name_edit.text().strip()
        self.build_dir_edit.setText(build.build_directory(name) if build else '')
        self.build_cmd_edit.setText(build.command if build else '')
        choices = driver_lib.program_choices(self.plan, build)
        self.program_combo.blockSignals(True)
        self.program_combo.clear()
        self.program_combo.addItems(choices)
        self.program_combo.blockSignals(False)
        # Build now unless the driver is built already; Python packages run
        # from source, and installing them is left to the user.
        built = bool(choices) and choices[0] in self.plan.programs
        self.compile_cb.setChecked(build is not None and build.kind != 'python' and not built)
        self.compile_cb.setEnabled(build is not None)
        self._refresh_command()

    def _on_name(self, text):
        # Keep the user's own edits: only swap the library's folder in them.
        new = text.strip()
        old_prefix, new_prefix = '{robot_lib}/%s' % self._name, '{robot_lib}/%s' % new
        for edit in (self.build_dir_edit, self.build_cmd_edit):
            value = edit.text()
            if value == old_prefix or value.startswith(old_prefix + '/'):
                edit.setText(new_prefix + value[len(old_prefix):])
        self._name = new
        self._refresh_command()
        self._refresh_buttons()

    def _program(self):
        return self.program_combo.currentText().strip()

    def _refresh_command(self, *_args):
        name = self.name_edit.text().strip()
        program = self._program()
        cmd = driver_lib.driver_command(name, program, self.args_edit.text().strip())
        self.command_lbl.setText(cmd or '(no driver program)')
        if not program:
            note = ('No driver program was found. Type the path of the program the build '
                    'makes, relative to the library folder.')
        elif (program in self.plan.programs
              or os.path.isfile(os.path.join(self.plan.root, program))):
            note = 'The driver program is already built.'
        else:
            note = ('The program is named after its source: the build decides the real name. '
                    'It is looked for again after the first build.')
        self.program_note.setText(note)

    def _refresh_buttons(self, *_args):
        name = self.name_edit.text().strip()
        problem = '' if self.plan.in_place else driver_lib.name_problem(name)
        target = os.path.join(self.lib_root, name)
        clash = not problem and not self.plan.in_place and os.path.exists(target)
        self.replace_cb.setVisible(clash)
        if clash:
            users = driver_lib.users_of(name, self.configs)
            self.replace_cb.setText(
                'robot_library/%s already exists%s. Replace it.'
                % (name, ' (used by %s)' % ', '.join(users) if users else ''))
        self.name_note.setText(problem)
        self.name_note.setVisible(bool(problem))
        self.ok_btn.setEnabled(not problem and (not clash or self.replace_cb.isChecked()))

    # ── Done ──────────────────────────────────────────────────────────────
    def _accept(self):
        name = self.name_edit.text().strip()
        program = self._program()
        args = self.args_edit.text().strip()
        build_cmd = self.build_cmd_edit.text().strip()
        self.settings = {
            'name': name,
            'replace': not self.replace_cb.isHidden() and self.replace_cb.isChecked(),
            'build_directory': self.build_dir_edit.text().strip() if build_cmd else '',
            'build_command': build_cmd,
            'program': program,
            'args': args,
            'driver_command': driver_lib.driver_command(name, program, args),
            'compile_now': self.compile_cb.isChecked() and bool(build_cmd),
        }
        self.accept()
