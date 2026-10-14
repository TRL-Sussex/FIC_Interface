"""RobotImportDialog -- the one screen between dropping a URDF and having a robot.

Shows everything the importer filled in from the robot's files (joints,
limits, speeds, end effector, home pose, gripper, safety values) and where
each came from, lets the user fix the few things that are guesses, then
builds the robot in place and turns into a "done" view offering to show it
in MuJoCo or run it.
"""
import math
import os

from fic_studio import theme
from fic_studio.robot_import import importer, mechanism
from fic_studio.robot_import.mjcf import MjcfError
from fic_studio.robot_import.urdf_info import limits_deg
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDialog, QDoubleSpinBox, QFileDialog, QFrame,
    QGridLayout, QHBoxLayout, QHeaderView, QLabel, QLineEdit, QPushButton, QScrollArea,
    QStackedWidget, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

_ICON = {importer.ERROR: '✕', importer.WARN: '!', importer.INFO: 'ℹ'}
_COLOR = {importer.ERROR: 'accent_red', importer.WARN: 'accent_orange',
          importer.INFO: 'text_secondary'}


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


def notes_widget(messages):
    """Build a column of icon + text rows, coloured by level."""
    box = QWidget()
    lay = QVBoxLayout(box)
    lay.setContentsMargins(0, 0, 0, 0)
    lay.setSpacing(4)
    for level, text in messages:
        row = QHBoxLayout()
        row.setSpacing(8)
        icon = QLabel(_ICON.get(level, '•'))
        icon.setFixedWidth(16)
        icon.setAlignment(Qt.AlignTop | Qt.AlignHCenter)
        body = QLabel(text)
        body.setWordWrap(True)
        body.setTextInteractionFlags(Qt.TextSelectableByMouse)
        key = _COLOR.get(level, 'text_secondary')
        theme.bind(icon, lambda t, k=key: 'color: %s; font-size: 12px; font-weight: 700;' % t[k])
        theme.bind(body, lambda t, k=key: 'color: %s; font-size: 12px;'
                   % (t['text_primary'] if k == 'text_secondary' else t[k]))
        row.addWidget(icon)
        row.addWidget(body, 1)
        lay.addLayout(row)
    return box


def found_widget(rows):
    """Build a what / value / where-from grid of what the importer filled in."""
    box = QWidget()
    grid = QGridLayout(box)
    grid.setContentsMargins(0, 0, 0, 0)
    grid.setHorizontalSpacing(12)
    grid.setVerticalSpacing(3)
    for r, (what, value, source) in enumerate(rows):
        a, b, c = QLabel(what), QLabel(value), QLabel(source)
        for lbl in (b, c):
            lbl.setWordWrap(True)
        theme.bind(a, lambda t: 'color: %s; font-size: 12px; font-weight: 600;'
                   % t['text_primary'])
        theme.bind(b, lambda t: 'color: %s; font-size: 12px;' % t['text_primary'])
        theme.bind(c, lambda t: 'color: %s; font-size: 11px;' % t['text_secondary'])
        grid.addWidget(a, r, 0, Qt.AlignTop)
        grid.addWidget(b, r, 1, Qt.AlignTop)
        grid.addWidget(c, r, 2, Qt.AlignTop)
    grid.setColumnStretch(1, 2)
    grid.setColumnStretch(2, 3)
    return box


class RobotImportDialog(QDialog):
    """Review an ImportPlan, create the robot, then offer what to do next.

    ``register(result)`` is called with the ``CreateResult`` as soon as the
    files exist, so the robot is in the Studio before the user picks a next
    step. ``next_action`` is then ``'simulate'``, ``'viewer'`` or None.
    ``plan`` is replaced when another description file is picked; the
    caller cleans up ``dialog.plan``.
    """

    def __init__(self, plan, existing_ids, model_root, register, parent=None):
        """Build the review screen for ``plan``."""
        super().__init__(parent)
        self.plan = plan
        self.existing = list(existing_ids)
        self.existing_ids = {i.lower() for i in existing_ids}
        self.model_root = model_root
        self.register = register
        self.result = None
        self.next_action = None
        self.setWindowTitle('Add a robot from its URDF')
        self.resize(860, 780)
        self.setMinimumSize(720, 600)
        theme.bind(self, lambda t: 'QDialog { background: %s; }' % t['bg_base'])

        self._pages = QStackedWidget()
        root = QVBoxLayout(self)
        root.setContentsMargins(20, 18, 20, 16)
        root.addWidget(self._pages)
        self._pages.addWidget(self._build_review())
        self._done_page = QWidget()
        self._pages.addWidget(self._done_page)
        self._refresh()

    # ── Review page ───────────────────────────────────────────────────────
    def _build_review(self):
        page = QWidget()
        outer = QVBoxLayout(page)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(10)

        title = QLabel('Add a robot from its URDF')
        theme.bind(title, lambda t: 'color: %s; font-size: 18px; font-weight: 700;'
                   % t['text_primary'])
        outer.addWidget(title)

        src_row = QHBoxLayout()
        src_row.setSpacing(8)
        self.source_lbl = _muted('')
        src_row.addWidget(self.source_lbl, 1)
        self.file_combo = QComboBox()
        self.file_combo.setToolTip('The folder holds several robot descriptions; pick the '
                                   'one to import')
        self.file_combo.setMinimumWidth(260)
        self.file_combo.currentIndexChanged.connect(self._on_file)
        src_row.addWidget(self.file_combo)
        outer.addLayout(src_row)

        mj_row = QHBoxLayout()
        mj_row.setSpacing(8)
        self.mjcf_lbl = _muted('')
        mj_row.addWidget(self.mjcf_lbl, 1)
        self.mjcf_btn = QPushButton('')
        self.mjcf_btn.clicked.connect(self._pick_mjcf)
        self.mjcf_clear = QPushButton("Don't use it")
        self.mjcf_clear.setToolTip('Build the simulation model from the URDF instead')
        self.mjcf_clear.clicked.connect(lambda: self._set_mjcf(''))
        mj_row.addWidget(self.mjcf_btn)
        mj_row.addWidget(self.mjcf_clear)
        outer.addLayout(mj_row)

        # Everything between the header and the buttons scrolls, so a long
        # list of notes never pushes the buttons off a small screen.
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
        form.setVerticalSpacing(4)
        self.name_edit = QLineEdit()
        self.name_edit.textChanged.connect(self._on_name)
        self.id_edit = QLineEdit()
        self.id_edit.textChanged.connect(self._on_id)
        self.ee_combo = QComboBox()
        self.ee_combo.currentTextChanged.connect(self._on_ee)
        self.ee_combo.setToolTip('The link Cartesian control moves')
        self.elbow_combo = QComboBox()
        self.elbow_combo.currentTextChanged.connect(self._on_elbow)
        self.elbow_combo.setToolTip('A link near the middle of the arm, used to steer the '
                                    'elbow of redundant arms. Optional.')
        self.ee_source = _muted('')
        self.elbow_source = _muted('the link after the middle joint')
        for col, (label, widget) in enumerate((('Name', self.name_edit),
                                               ('Robot ID', self.id_edit))):
            form.addWidget(self._label(label), 0, col * 2)
            form.addWidget(widget, 0, col * 2 + 1)
        for col, (label, widget, note) in enumerate((
                ('End effector', self.ee_combo, self.ee_source),
                ('Elbow', self.elbow_combo, self.elbow_source))):
            form.addWidget(self._label(label), 1, col * 2)
            form.addWidget(widget, 1, col * 2 + 1)
            form.addWidget(note, 2, col * 2 + 1)
        form.setColumnStretch(1, 1)
        form.setColumnStretch(3, 1)
        lay.addLayout(form)
        self.id_note = QLabel('')
        theme.bind(self.id_note, lambda t: 'color: %s; font-size: 11px;' % t['accent_red'])
        lay.addWidget(self.id_note)
        self.replace_cb = QCheckBox()
        self.replace_cb.toggled.connect(self._refresh_buttons)
        self.replace_cb.hide()
        lay.addWidget(self.replace_cb)

        self.summary = QLabel()
        theme.bind(self.summary, lambda t: 'color: %s; font-size: 12px; font-weight: 600;'
                   % t['text_primary'])
        lay.addWidget(self.summary)

        self.table = QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels(
            ['Driven', 'Joint', 'Range (°)', 'Torque (Nm)', 'Speed (°/s)', 'Home (°)'])
        self.table.itemChanged.connect(self._on_driven_toggled)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSelectionMode(QTableWidget.NoSelection)
        self.table.setFocusPolicy(Qt.NoFocus)
        self.table.verticalHeader().setDefaultSectionSize(28)
        hdr = self.table.horizontalHeader()
        hdr.setSectionResizeMode(1, QHeaderView.Stretch)
        for c in (0, 2, 3, 4, 5):
            hdr.setSectionResizeMode(c, QHeaderView.ResizeToContents)
        theme.bind(self.table, lambda t: (
            'QTableWidget { background: %s; color: %s; border: 1px solid %s;'
            ' border-radius: 6px; gridline-color: %s; font-size: 12px; }'
            'QHeaderView::section { background: %s; color: %s; border: none;'
            ' padding: 4px 8px; font-size: 11px; font-weight: 700; }'
        ) % (t['bg_raised'], t['text_primary'], t['border_subtle'], t['border_subtle'],
             t['bg_overlay'], t['text_secondary']))
        lay.addWidget(self.table)
        lay.addWidget(_muted("Every value here can be changed after import on the robot's "
                             'Joints, Frames and Safety tabs. The robot is added for '
                             'simulation; hardware settings stay off until you set them.'))

        lay.addWidget(self._build_loops())

        lay.addWidget(_heading("Also filled in from the robot's files"))
        self.found_holder = QVBoxLayout()
        lay.addLayout(self.found_holder)
        lay.addWidget(_heading('Notes'))
        self.notes_holder = QVBoxLayout()
        lay.addLayout(self.notes_holder)
        lay.addStretch()

        self.status = _muted('')
        outer.addWidget(self.status)
        buttons = QHBoxLayout()
        buttons.addStretch()
        self.cancel_btn = QPushButton('Cancel')
        self.cancel_btn.clicked.connect(self.reject)
        self.create_btn = QPushButton('Create robot')
        self.create_btn.setObjectName('BtnTeal')
        self.create_btn.setDefault(True)
        self.create_btn.clicked.connect(self._create)
        buttons.addWidget(self.cancel_btn)
        buttons.addWidget(self.create_btn)
        outer.addLayout(buttons)
        return page

    def _build_loops(self):
        """Build the Loops box: what closes the mechanism, and adding a loop by hand."""
        box = QFrame()
        box.setObjectName('LoopsBox')
        theme.bind(box, lambda t: 'QFrame#LoopsBox { background: %s; border: 1px solid %s;'
                   ' border-radius: 8px; }' % (t['bg_raised'], t['border_subtle']))
        lay = QVBoxLayout(box)
        lay.setContentsMargins(12, 10, 12, 10)
        lay.setSpacing(6)
        head = QHBoxLayout()
        head.addWidget(_heading('Loops (closed-loop robots)'))
        head.addStretch()
        self.add_loop_btn = QPushButton('Add a loop…')
        self.add_loop_btn.setToolTip('For a parallel mechanism whose URDF has the loop cut')
        self.add_loop_btn.clicked.connect(lambda: self.loop_form.setVisible(
            not self.loop_form.isVisible()))
        head.addWidget(self.add_loop_btn)
        lay.addLayout(head)
        self.loops_hint = _muted('')
        lay.addWidget(self.loops_hint)
        self.loops_table = QTableWidget(0, 5)
        self.loops_table.setHorizontalHeaderLabels(
            ['Link A', 'Point in A (m)', 'Link B', 'From', ''])
        self.loops_table.verticalHeader().setVisible(False)
        self.loops_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.loops_table.setSelectionMode(QTableWidget.NoSelection)
        h = self.loops_table.horizontalHeader()
        for c in range(5):
            h.setSectionResizeMode(c, QHeaderView.ResizeToContents)
        h.setSectionResizeMode(3, QHeaderView.Stretch)
        lay.addWidget(self.loops_table)

        self.loop_form = QWidget()
        form = QGridLayout(self.loop_form)
        form.setContentsMargins(0, 4, 0, 0)
        self.loop_a = QComboBox()
        self.loop_b = QComboBox()
        self.loop_xyz = []
        for _ in range(3):
            sb = QDoubleSpinBox()
            sb.setRange(-5.0, 5.0)
            sb.setDecimals(5)
            sb.setSingleStep(0.001)
            self.loop_xyz.append(sb)
        form.addWidget(self._label('Link A'), 0, 0)
        form.addWidget(self.loop_a, 0, 1)
        form.addWidget(self._label('point on A, in its frame (m)'), 0, 2)
        for k, sb in enumerate(self.loop_xyz):
            form.addWidget(sb, 0, 3 + k)
        form.addWidget(self._label('is held to link B'), 1, 0)
        form.addWidget(self.loop_b, 1, 1)
        add = QPushButton('Add loop')
        add.clicked.connect(self._add_loop)
        form.addWidget(add, 1, 3, 1, 3)
        form.addWidget(_muted("The point on B is where the point on A is in the URDF's zero "
                              'pose: a URDF is cut with its loops still assembled.'),
                       2, 0, 1, 6)
        self.loop_form.hide()
        lay.addWidget(self.loop_form)
        return box

    def _pick_mjcf(self):
        start = os.path.dirname(self.plan.mjcf.path if self.plan.mjcf else self.plan.urdf_path)
        path, _ = QFileDialog.getOpenFileName(
            self, "Choose the robot's MuJoCo model", start, 'MuJoCo model (*.xml)')
        if path:
            self._set_mjcf(path)

    def _set_mjcf(self, path):
        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            importer.set_mjcf(self.plan, path)
        finally:
            QApplication.restoreOverrideCursor()
        self._refresh()

    def _add_loop(self):
        a, b = self.loop_a.currentText(), self.loop_b.currentText()
        if not a or not b or a == b:
            self._show_notes([(importer.ERROR, 'Pick two different links for the loop.')]
                             + self.plan.messages)
            return
        loop = mechanism.closure_from_point(self.plan.info, a,
                                            [sb.value() for sb in self.loop_xyz], b)
        loop.name = 'loop%d' % (len(self.plan.closures) + 1)
        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            importer.set_closures(self.plan, self.plan.closures + [loop])
        finally:
            QApplication.restoreOverrideCursor()
        self.loop_form.hide()
        self._refresh()

    def _remove_loop(self, index):
        loops = [c for k, c in enumerate(self.plan.closures) if k != index]
        importer.set_closures(self.plan, loops)
        self._refresh()

    def _on_driven_toggled(self, item):
        if item.column() != 0 or not self.plan.closed_loop:
            return
        names = [self.table.item(r, 1).text() for r in range(self.table.rowCount())
                 if self.table.item(r, 0) is not None
                 and self.table.item(r, 0).checkState() == Qt.Checked]
        importer.set_driven(self.plan, names)
        self._refresh()

    @staticmethod
    def _label(text):
        lbl = QLabel(text)
        theme.bind(lbl, lambda t: 'color: %s; font-size: 12px;' % t['text_secondary'])
        return lbl

    def _ordered_links(self):
        """Links in tree order, so the arm reads top to bottom."""
        info = self.plan.info
        return sorted(info.links,
                      key=lambda link: (len(info.chain_to(link)), info.links.index(link)))

    def _source_root(self):
        return self.plan.roots[0] if self.plan.roots else os.path.dirname(self.plan.urdf_path)

    # ── Reactions ─────────────────────────────────────────────────────────
    def _on_name(self, text):
        self.plan.display_name = text.strip()

    def _on_id(self, text):
        self.plan.robot_id = text.strip()
        self._refresh_buttons()

    def _on_ee(self, link):
        if link and link != self.plan.ee_link:
            self.plan.ee_source = 'picked by you'
            importer.set_end_effector(self.plan, link)
            self._refresh()

    def _on_elbow(self, text):
        self.plan.elbow_link = '' if text == '(none)' else text

    def _on_file(self, index):
        path = self.file_combo.itemData(index)
        if not path or path == self.plan.urdf_path:
            return
        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            new = importer.reanalyse(self.plan, path, self.existing)
        except importer.ImportError_ as exc:
            QApplication.restoreOverrideCursor()
            self._show_notes([(importer.ERROR, 'Could not read %s: %s'
                               % (os.path.basename(path), exc))])
            self.file_combo.blockSignals(True)
            self.file_combo.setCurrentIndex(self.file_combo.findData(self.plan.urdf_path))
            self.file_combo.blockSignals(False)
            return
        QApplication.restoreOverrideCursor()
        self.plan = new
        self._refresh()

    def _refresh(self):
        plan = self.plan
        combos = (self.name_edit, self.id_edit, self.ee_combo, self.elbow_combo,
                  self.file_combo)
        for w in combos:
            w.blockSignals(True)
        self.name_edit.setText(plan.display_name)
        self.id_edit.setText(plan.robot_id)
        many = len(plan.urdf_candidates) > 1
        self.source_lbl.setText('From %s' % (plan.source if plan.temp_dir or many
                                             else plan.urdf_path))
        self.file_combo.clear()
        root = self._source_root()
        for path in plan.urdf_candidates:
            self.file_combo.addItem(os.path.relpath(path, root), path)
        self.file_combo.setCurrentIndex(self.file_combo.findData(plan.urdf_path))
        self.file_combo.setVisible(many)
        self.ee_combo.clear()
        self.ee_combo.addItems(self._ordered_links())
        self.ee_combo.setCurrentText(plan.ee_link)
        self.ee_source.setText(plan.ee_source)
        self.elbow_combo.clear()
        self.elbow_combo.addItem('(none)')
        self.elbow_combo.addItems([j.child for j in plan.info.arm_joints(plan.ee_link)][:-1])
        self.elbow_combo.setCurrentText(plan.elbow_link or '(none)')
        for w in combos:
            w.blockSignals(False)

        self._fill_model_row()
        self._fill_joints()
        self._fill_loops()

        found, total = plan.mesh_counts
        parts = ['%d joint%s' % (len(plan.joints), '' if len(plan.joints) == 1 else 's'),
                 'ends at %s' % plan.ee_link]
        if total:
            parts.append('%d of %d meshes found' % (found, total))
        if plan.gripper:
            parts.append('gripper driven in simulation')
        if plan.closed_loop:
            m = plan.mechanism
            parts.append('closed loop%s' % (
                ': %d DOF%s' % (m['ee_dof'], ', turns only' if m.get('orientation_only') else '')
                if 'ee_dof' in m else ''))
        self.summary.setText(' · '.join(parts))

        while self.found_holder.count():
            w = self.found_holder.takeAt(0).widget()
            if w:
                w.deleteLater()
        rows = [r for r in plan.found if r[0] not in ('Joints', 'End effector', 'Elbow')]
        if rows:
            self.found_holder.addWidget(found_widget(rows))
        self._show_notes(plan.messages)
        self._refresh_buttons()

    def _fill_model_row(self):
        plan = self.plan
        if plan.mjcf is not None:
            self.mjcf_lbl.setText("MuJoCo model: %s, used for simulation with FIC's motors and "
                                  'sensors added.' % os.path.basename(plan.mjcf.path))
            self.mjcf_btn.setText('Use another…')
            self.mjcf_clear.show()
        else:
            self.mjcf_lbl.setText('MuJoCo model: none, so one is built from the URDF. Have your '
                                  "own? It keeps your tuning, and a closed loop's constraints.")
            self.mjcf_btn.setText('Add yours…')
            self.mjcf_clear.hide()

    def _fill_joints(self):
        """Arm joints; for a closed loop every mechanism joint, with a Driven tick."""
        plan, info = self.plan, self.plan.info
        driven = {j.name: j for j in plan.joints}
        if plan.closed_loop:
            names = [n for n in plan.mechanism_joints if info.joints[n].movable]
        else:
            names = list(driven)
        self.table.blockSignals(True)
        self.table.setColumnHidden(0, not plan.closed_loop)
        self.table.setRowCount(len(names))
        for r, name in enumerate(names):
            j = driven.get(name)
            if j is not None:
                rng = 'continuous' if j.continuous else '%.0f … %.0f' % (j.lower_deg,
                                                                         j.upper_deg)
                tau = '%g%s' % (j.effort, '' if j.effort_from_urdf else ' (default)')
                speed = '%.0f' % math.degrees(j.velocity) if j.velocity else '—'
                home = '%.1f' % j.home_deg
            else:
                lo, hi = limits_deg(info.joints[name])
                rng = 'continuous' if info.joints[name].continuous else '%.0f … %.0f' % (lo, hi)
                tau, speed, home = 'passive', '—', '—'
            tick = QTableWidgetItem('')
            tick.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
            tick.setCheckState(Qt.Checked if j is not None else Qt.Unchecked)
            self.table.setItem(r, 0, tick)
            for c, text in enumerate((name, rng, tau, speed, home), start=1):
                item = QTableWidgetItem(text)
                if c > 1:
                    item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                self.table.setItem(r, c, item)
        self.table.blockSignals(False)
        rows_shown = min(max(len(names), 1), 8)
        self.table.setFixedHeight(self.table.horizontalHeader().sizeHint().height()
                                  + self.table.verticalHeader().defaultSectionSize() * rows_shown
                                  + 2 * self.table.frameWidth() + 2)

    def _fill_loops(self):
        plan = self.plan
        own = plan.mjcf is not None
        self.add_loop_btn.setVisible(not own)
        if not own and self.loop_a.count() != len(plan.info.links):
            for combo in (self.loop_a, self.loop_b):
                combo.clear()
                combo.addItems(self._ordered_links())
        if not own:
            self.loop_form.setVisible(self.loop_form.isVisible())
        else:
            self.loop_form.hide()
        loops = plan.closures
        if loops:
            source = sorted({c.source for c in loops if c.source})
            self.loops_hint.setText('%d loop%s, from %s. The controllers close them with their '
                                    'closed-loop solver; the simulator with constraints.'
                                    % (len(loops), '' if len(loops) == 1 else 's',
                                       ' and '.join(source) or 'you'))
        elif own:
            self.loops_hint.setText('None in the MuJoCo model: an open-chain robot.')
        else:
            self.loops_hint.setText('None: an open-chain robot. For a parallel mechanism whose '
                                    "URDF has its loops cut, add them here, or add the robot's "
                                    'MuJoCo model above.')
        self.loops_table.setVisible(bool(loops))
        self.loops_table.setRowCount(len(loops))
        for r, c in enumerate(loops):
            for col, text in enumerate((c.body_a, '%.4f  %.4f  %.4f' % tuple(c.pos_a), c.body_b,
                                        c.source.replace('the ', '', 1))):
                self.loops_table.setItem(r, col, QTableWidgetItem(text))
            if c.source == 'added by you':
                rm = QPushButton('Remove')
                rm.clicked.connect(lambda _=False, i=r: self._remove_loop(i))
                self.loops_table.setCellWidget(r, 4, rm)
            else:
                self.loops_table.setCellWidget(r, 4, None)
        self.loops_table.setFixedHeight(
            self.loops_table.horizontalHeader().sizeHint().height()
            + self.loops_table.verticalHeader().defaultSectionSize() * max(1, len(loops))
            + 2 * self.loops_table.frameWidth() + 2)

    def _show_notes(self, messages):
        while self.notes_holder.count():
            w = self.notes_holder.takeAt(0).widget()
            if w:
                w.deleteLater()
        self.notes_holder.addWidget(
            notes_widget(messages) if messages else _muted('Nothing to report.'))

    def _id_problem(self):
        rid = self.plan.robot_id
        if not importer.valid_robot_id(rid):
            return 'Start with a letter; use letters, digits and _ only.'
        if rid.lower() in self.existing_ids:
            return 'A robot called %s already exists. Pick another ID.' % rid
        return ''

    def _refresh_buttons(self):
        problem = self._id_problem()
        folder = os.path.join(self.model_root, self.plan.robot_id)
        clash = not problem and os.path.exists(folder)
        self.replace_cb.setVisible(clash)
        if clash:
            self.replace_cb.setText('menagerie/model/%s already exists (from an earlier '
                                    'import?). Replace it.' % self.plan.robot_id)
        self.id_note.setText(problem)
        self.id_note.setVisible(bool(problem))
        ok = (not problem and not self.plan.errors
              and (not clash or self.replace_cb.isChecked()))
        self.create_btn.setEnabled(ok)

    # ── Create ────────────────────────────────────────────────────────────
    def _say(self, text):
        self.status.setText(text + '…')
        QApplication.processEvents()

    def _create(self):
        widgets = (self.create_btn, self.cancel_btn, self.name_edit, self.id_edit,
                   self.ee_combo, self.elbow_combo, self.file_combo)
        for w in widgets:
            w.setEnabled(False)
        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            self.result = importer.create_robot(
                self.plan, self.model_root, overwrite=self.replace_cb.isChecked(),
                progress=self._say)
            self.register(self.result)
        except Exception as exc:          # every failure is shown, none is fatal
            QApplication.restoreOverrideCursor()
            self.status.setText('')
            known = isinstance(exc, (importer.ImportError_, MjcfError))
            text = str(exc) if known else '%s: %s' % (type(exc).__name__, exc)
            self._show_notes([(importer.ERROR, 'The robot was not created. ' + text)]
                             + self.plan.messages)
            for w in widgets[1:]:
                w.setEnabled(True)
            self._refresh_buttons()
            return
        QApplication.restoreOverrideCursor()
        self._show_done()

    # ── Done page ─────────────────────────────────────────────────────────
    def _show_done(self):
        res = self.result
        lay = QVBoxLayout(self._done_page)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(10)
        head = QLabel('✓  %s is ready' % (res.config.get('display_name') or res.robot_id))
        theme.bind(head, lambda t: 'color: %s; font-size: 18px; font-weight: 700;'
                   % t['accent_green'])
        lay.addWidget(head)
        stats = res.stats
        lay.addWidget(_muted(
            'Saved to menagerie/model/%s and added to the robot list: %d joints, %.2f kg, '
            'reaching %.2f m at home. It held its home pose under gravity compensation '
            '(drift %.2f°).'
            % (res.robot_id, res.config['num_joints'], stats['mass_kg'], stats['reach_m'],
               stats['hold_drift_deg']), 12))
        worth = [m for m in res.messages if m[0] != importer.INFO]
        if worth:
            box = QFrame()
            box.setObjectName('WorthBox')      # by name: QLabels are QFrames too
            bl = QVBoxLayout(box)
            bl.setContentsMargins(12, 10, 12, 10)
            theme.bind(box, lambda t: 'QFrame#WorthBox { background: %s; border: 1px solid %s;'
                       ' border-radius: 8px; }' % (t['bg_raised'], t['border_subtle']))
            bl.addWidget(_muted('Worth checking before you rely on it:'))
            bl.addWidget(notes_widget(worth))
            lay.addWidget(box)
        lay.addWidget(_muted(
            'Next: Try in simulation starts the MuJoCo simulator and the FIC controller for '
            'it. To drive the real robot, fill in the Hardware tab.'))
        lay.addStretch()
        row = QHBoxLayout()
        row.addStretch()
        viewer = QPushButton('Open in MuJoCo viewer')
        viewer.clicked.connect(lambda: self._finish('viewer'))
        simulate = QPushButton('▶  Try in simulation')
        simulate.setObjectName('BtnTeal')
        simulate.clicked.connect(lambda: self._finish('simulate'))
        close = QPushButton('Done')
        close.clicked.connect(lambda: self._finish(None))
        for b in (viewer, simulate, close):
            row.addWidget(b)
        lay.addLayout(row)
        self._pages.setCurrentWidget(self._done_page)
        simulate.setFocus()

    def _finish(self, action):
        self.next_action = action
        self.accept()
