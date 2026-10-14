# widgets.py — Reusable widgets for Robot Selector GUI

import math
import subprocess
import os
from PyQt5.QtCore import Qt, QTimer, QPointF, QProcess, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QBrush, QPen, QFont, QTextCursor
from PyQt5.QtWidgets import (
    QWidget, QLabel, QPushButton, QFrame, QDialog, QVBoxLayout, QHBoxLayout,
    QProgressBar, QTextEdit, QSizePolicy,
)


class StatusLED(QFrame):
    """LED indicator widget for status display."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(10, 10)
        self._status = 'idle'
        self._update_style()

    def set_status(self, status: str):
        self._status = status
        self._update_style()

    def _update_style(self):
        # Semantic status colours from Warmth theme tokens.
        from fic_studio import theme
        t = theme.current()
        tok = {
            'connected': 'accent_green', 'disconnected': 'accent_red',
            'warning': 'accent_orange', 'idle': 'text_disabled',
            'active': 'accent_blue',
        }
        color = t[tok.get(self._status, 'text_disabled')]
        self.setStyleSheet(
            'QFrame { background-color: %s; border-radius: 5px; '
            'border: 1px solid %s; }' % (color, t['bg_base']))


def _status_dot(color: str = None) -> QLabel:
    """Create a small colored dot label for status indicators."""
    if color is None:
        from fic_studio import theme
        color = theme.current()['accent_green']
    dot = QLabel()
    dot.setObjectName("StatusDot")
    dot.setStyleSheet(
        f"background-color:{color};border-radius:3px;"
        "min-width:7px;max-width:7px;min-height:7px;max-height:7px;"
    )
    return dot


class VirtualJoystick(QWidget):
    """
    A virtual joystick widget that emits (dx, dy) values from -1.0 to 1.0.
    
    Signals:
        joystickMoved(float, float): Emitted continuously while grabbed,
                                      with normalized x and y displacement.
    """
    joystickMoved = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(100, 100)
        self.is_grabbed = False
        self.max_radius = 40
        self.stick_radius = 15
        # We calculate the center dynamically in paintEvent,
        # but store the stick offset here.
        self.stick_offset = QPointF(0, 0)
        # Timer to continuously emit values while held off-center
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.emit_position)
        self.timer.start(50)  # 20Hz update rate

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        center = QPointF(self.width() / 2, self.height() / 2)
        # Draw Base
        from fic_studio import theme
        t = theme.current()
        painter.setBrush(QBrush(QColor(t['bg_overlay'])))
        painter.setPen(QPen(QColor(t['border_subtle']), 2))
        painter.drawEllipse(center, self.max_radius, self.max_radius)
        # Draw Stick
        stick_color = QColor(t['accent_teal']) if self.is_grabbed else QColor(t['text_disabled'])
        painter.setBrush(QBrush(stick_color))
        painter.setPen(Qt.NoPen)
        stick_pos = center + self.stick_offset
        painter.drawEllipse(stick_pos, self.stick_radius, self.stick_radius)

    def mousePressEvent(self, event):
        center = QPointF(self.width() / 2, self.height() / 2)
        if (event.pos() - center).manhattanLength() < self.max_radius:
            self.is_grabbed = True
            self.update_stick(event.pos(), center)

    def mouseMoveEvent(self, event):
        if self.is_grabbed:
            center = QPointF(self.width() / 2, self.height() / 2)
            self.update_stick(event.pos(), center)

    def mouseReleaseEvent(self, event):
        self.is_grabbed = False
        self.stick_offset = QPointF(0, 0)
        self.update()
        self.joystickMoved.emit(0.0, 0.0)  # Stop command

    def update_stick(self, pos, center):
        vec = pos - center
        dist = math.hypot(vec.x(), vec.y())
        # Clamp to max radius
        if dist > self.max_radius:
            vec.setX(vec.x() * self.max_radius / dist)
            vec.setY(vec.y() * self.max_radius / dist)
        self.stick_offset = vec
        self.update()

    def emit_position(self):
        if not self.is_grabbed:
            return
        # Normalize to -1.0 to +1.0
        dx = self.stick_offset.x() / self.max_radius
        dy = -self.stick_offset.y() / self.max_radius  # Invert Y so UP is positive
        # Deadzone
        if abs(dx) < 0.05:
            dx = 0.0
        if abs(dy) < 0.05:
            dy = 0.0
        if dx != 0.0 or dy != 0.0:
            self.joystickMoved.emit(dx, dy)


class BuildDialog(QDialog):
    """
    Modal dialog that runs a shell build command, showing a progress bar
    and live stdout/stderr output. Replaces the gnome-terminal popup.
    """

    def __init__(self, title: str, command: str, work_dir: str, parent=None,
                 pre_command: str = None):
        """
        Args:
            title:       Dialog window title (e.g. "Compile Workspace").
            command:     The shell command to execute.
            work_dir:    Working directory for the process.
            pre_command: Optional blocking command to run first (e.g. rm -rf).
        """
        super().__init__(parent)
        self.setWindowTitle(title)
        self.resize(720, 420)
        self.setMinimumSize(520, 300)
        self._finished = False
        #: The build's exit code once it ends (-1 if it crashed or was
        #: cancelled); None while it runs.
        self.exit_code = None

        from fic_studio import theme
        self._tok = theme.current()
        t = self._tok

        layout = QVBoxLayout(self)
        layout.setSpacing(10)

        # ── Status label ──
        self._status_label = QLabel("Preparing...")
        self._status_label.setStyleSheet("font-size: 13px; font-weight: 600;")
        layout.addWidget(self._status_label)

        # ── Progress bar (indeterminate while running) ──
        self._progress = QProgressBar()
        self._progress.setRange(0, 0)  # indeterminate pulse
        self._progress.setFixedHeight(6)
        self._progress.setTextVisible(False)
        self._progress.setStyleSheet(
            'QProgressBar { background: %s; border: none; border-radius: 3px; }'
            'QProgressBar::chunk { background: qlineargradient(x1:0, y1:0, x2:1, y2:0,'
            ' stop:0 %s, stop:1 %s); border-radius: 3px; }'
            % (t['bg_overlay'], t['accent_blue'], t['accent_orange']))
        layout.addWidget(self._progress)

        # ── Output console ──
        self._output = QTextEdit()
        self._output.setReadOnly(True)
        self._output.setFont(QFont("Source Code Pro, Consolas, monospace", 10))
        self._output.setStyleSheet(
            'QTextEdit { background: %s; color: %s; '
            'border: 1px solid %s; border-radius: 6px; padding: 8px; }'
            % (t['bg_base'], t['text_primary'], t['border_subtle']))
        layout.addWidget(self._output, 1)

        # ── Button row ──
        btn_row = QHBoxLayout()
        btn_row.addStretch()

        self._cancel_btn = QPushButton("Cancel")
        self._cancel_btn.setFixedWidth(100)
        self._cancel_btn.clicked.connect(self._on_cancel)
        btn_row.addWidget(self._cancel_btn)

        self._close_btn = QPushButton("Close")
        self._close_btn.setFixedWidth(100)
        self._close_btn.setEnabled(False)
        self._close_btn.clicked.connect(self.accept)
        btn_row.addWidget(self._close_btn)

        layout.addLayout(btn_row)

        # ── Run pre-command synchronously if provided ──
        if pre_command:
            self._status_label.setText("Cleaning...")
            self._append_text(f"$ {pre_command}\n", self._tok['text_secondary'])
            try:
                result = subprocess.run(
                    pre_command, shell=True, cwd=work_dir,
                    capture_output=True, text=True, timeout=60)
                if result.stdout:
                    self._append_text(result.stdout)
                if result.stderr:
                    self._append_text(result.stderr, self._tok['accent_orange'])
            except Exception as e:
                self._append_text(f"Pre-command error: {e}\n", self._tok['accent_red'])

        # ── Start the main build process ──
        self._status_label.setText("Building...")
        self._append_text(f"$ {command}\n", self._tok['text_secondary'])

        self._process = QProcess(self)
        self._process.setWorkingDirectory(work_dir)
        self._process.setProcessChannelMode(QProcess.MergedChannels)
        self._process.readyReadStandardOutput.connect(self._on_stdout)
        self._process.finished.connect(self._on_finished)

        self._elapsed_timer = QTimer(self)
        self._elapsed_seconds = 0
        self._elapsed_timer.timeout.connect(self._tick_elapsed)
        self._elapsed_timer.start(1000)

        self._process.start("bash", ["-c", command])

    # ── Internal helpers ──

    def _append_text(self, text: str, color: str = None):
        cursor = self._output.textCursor()
        cursor.movePosition(QTextCursor.End)
        if color:
            cursor.insertHtml(
                f"<span style='color:{color}'>{text.replace(chr(10), '<br>')}</span>")
        else:
            cursor.insertText(text)
        self._output.setTextCursor(cursor)
        self._output.ensureCursorVisible()

    def _tick_elapsed(self):
        self._elapsed_seconds += 1
        m, s = divmod(self._elapsed_seconds, 60)
        self._status_label.setText(f"Building... ({m}:{s:02d})")

    def _on_stdout(self):
        data = self._process.readAllStandardOutput()
        text = bytes(data).decode("utf-8", errors="replace")
        # Strip ANSI escape codes for clean display
        import re
        text = re.sub(r'\x1b\[[0-9;]*m', '', text)
        self._append_text(text)

    def _on_finished(self, exit_code, exit_status):
        self._finished = True
        self.exit_code = exit_code if exit_status == QProcess.NormalExit else -1
        self._elapsed_timer.stop()
        m, s = divmod(self._elapsed_seconds, 60)

        self._progress.setRange(0, 100)

        if exit_code == 0:
            self._progress.setValue(100)
            self._progress.setStyleSheet(
                'QProgressBar { background: %s; border: none; border-radius: 3px; }'
                'QProgressBar::chunk { background: %s; border-radius: 3px; }'
                % (self._tok['bg_overlay'], self._tok['accent_green']))
            self._status_label.setText(f"Build succeeded ({m}:{s:02d})")
            self._status_label.setStyleSheet(
                'font-size: 13px; font-weight: 600; color: %s;'
                % self._tok['accent_green'])
            self._append_text(f"\n--- Build completed successfully ({m}:{s:02d}) ---\n",
                              self._tok['accent_green'])
        else:
            self._progress.setValue(100)
            self._progress.setStyleSheet(
                'QProgressBar { background: %s; border: none; border-radius: 3px; }'
                'QProgressBar::chunk { background: %s; border-radius: 3px; }'
                % (self._tok['bg_overlay'], self._tok['accent_red']))
            self._status_label.setText(f"Build failed (exit code {exit_code})")
            self._status_label.setStyleSheet(
                'font-size: 13px; font-weight: 600; color: %s;'
                % self._tok['accent_red'])
            self._append_text(
                f"\n--- Build failed with exit code {exit_code} ({m}:{s:02d}) ---\n",
                self._tok['accent_red'])

        self._cancel_btn.setEnabled(False)
        self._close_btn.setEnabled(True)
        self._close_btn.setFocus()

    def _on_cancel(self):
        if not self._finished and self._process.state() != QProcess.NotRunning:
            self._process.kill()
            self._append_text("\n--- Build cancelled by user ---\n",
                              self._tok['accent_orange'])
            self._status_label.setText("Cancelled")
            self._status_label.setStyleSheet(
                'font-size: 13px; font-weight: 600; color: %s;'
                % self._tok['accent_orange'])
            self._elapsed_timer.stop()
            self._progress.setRange(0, 100)
            self._progress.setValue(0)
            self._cancel_btn.setEnabled(False)
            self._close_btn.setEnabled(True)
            self._finished = True

    def closeEvent(self, event):
        if not self._finished and self._process.state() != QProcess.NotRunning:
            self._process.kill()
            self._process.waitForFinished(3000)
        event.accept()