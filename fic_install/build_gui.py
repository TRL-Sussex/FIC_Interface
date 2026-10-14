#!/usr/bin/env python3
"""
build_gui.py — the installer's build window for FIC Control Studio (PyQt5),
in the web interface's dark theme (installer_theme.py).

Usage:
    python3 build_gui.py <work_dir> <command> [--title TITLE] [--pre-command CMD]

Exit codes:
    0  Build succeeded
    1  Build failed
    2  Cancelled by user
    3  GUI could not start (fallback to terminal)
"""

import sys
import os
import re
import subprocess

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from installer_theme import COLORS as C, MONO, SANS, pick_family  # noqa: E402


def _bar_style(color):
    return ("QProgressBar { background: %s; border: none; border-radius: 2px; }"
            "QProgressBar::chunk { background: %s; border-radius: 2px; }" % (C['rule'], color))


def _status_style(color):
    return "font-size: 13px; color: %s;" % color


def run_gui(work_dir, command, title="Building...", pre_command=None):
    """Launch the PyQt5 build dialog. Returns exit code."""
    try:
        from PyQt5.QtCore import Qt, QTimer, QProcess
        from PyQt5.QtGui import QFont, QTextCursor
        from PyQt5.QtWidgets import (
            QApplication, QDialog, QVBoxLayout, QHBoxLayout,
            QLabel, QPushButton, QProgressBar, QTextEdit,
        )
    except ImportError:
        return 3  # signal caller to fall back

    app = QApplication(sys.argv)
    from PyQt5.QtGui import QFontDatabase
    families = QFontDatabase().families()
    app.setFont(QFont(pick_family(families, SANS, app.font().family()), 10))
    mono_family = pick_family(families, MONO, 'monospace')

    class BuildDialog(QDialog):
        def __init__(self):
            super().__init__()
            self.setWindowTitle(title)
            self.resize(720, 440)
            self.setMinimumSize(520, 300)
            self._finished = False
            self._exit_code = 1

            layout = QVBoxLayout(self)
            layout.setContentsMargins(24, 20, 24, 16)
            layout.setSpacing(10)

            # ── Title and status ──
            heading = QLabel(title)
            heading.setStyleSheet("font-size: 17px; font-weight: 600; color: %s;" % C['ink_strong'])
            layout.addWidget(heading)
            self._status = QLabel("Preparing…")
            self._status.setStyleSheet(_status_style(C['ink_muted']))
            layout.addWidget(self._status)

            # ── Progress bar ──
            self._bar = QProgressBar()
            self._bar.setRange(0, 0)
            self._bar.setFixedHeight(4)
            self._bar.setTextVisible(False)
            self._bar.setStyleSheet(_bar_style(C['accent']))
            layout.addWidget(self._bar)

            # ── Output console ──
            self._output = QTextEdit()
            self._output.setReadOnly(True)
            self._output.setFont(QFont(mono_family, 10))
            self._output.setStyleSheet(
                "QTextEdit { background: %s; color: %s; border: 1px solid %s; "
                "border-radius: 8px; padding: 8px; }" % (C['sunk'], C['ink'], C['rule']))
            layout.addWidget(self._output, 1)

            # ── Buttons, under a hairline ──
            from PyQt5.QtWidgets import QFrame
            rule = QFrame()
            rule.setFixedHeight(1)
            rule.setStyleSheet("background: %s;" % C['rule'])
            layout.addSpacing(6)
            layout.addWidget(rule)
            btn_row = QHBoxLayout()
            btn_row.addStretch()
            self._cancel_btn = QPushButton("Cancel")
            self._cancel_btn.clicked.connect(self._on_cancel)
            btn_row.addWidget(self._cancel_btn)
            self._close_btn = QPushButton("Close")
            self._close_btn.setObjectName("primary")
            self._close_btn.setEnabled(False)
            self._close_btn.clicked.connect(self.accept)
            btn_row.addWidget(self._close_btn)
            layout.addLayout(btn_row)

            # The web interface's dark theme: quiet buttons, one solid primary
            self.setStyleSheet("""
                QDialog { background: %(canvas)s; }
                QLabel { color: %(ink)s; }
                QPushButton {
                    background: transparent; color: %(ink)s; border: none;
                    border-radius: 6px; padding: 7px 16px; min-width: 64px;
                }
                QPushButton:hover { background: %(hover)s; }
                QPushButton:disabled { color: %(ink_faint)s; background: transparent; }
                QPushButton#primary { background: %(accent)s; color: %(on_accent)s; font-weight: 600; }
                QPushButton#primary:hover { background: %(accent_hover)s; }
                QPushButton#primary:disabled { background: transparent; color: %(ink_faint)s; }
            """ % C)

            # ── Pre-command ──
            if pre_command:
                self._status.setText("Cleaning…")
                self._append(f"$ {pre_command}\n", C['ink_muted'])
                try:
                    result = subprocess.run(
                        pre_command, shell=True, cwd=work_dir,
                        capture_output=True, text=True, timeout=120)
                    if result.stdout:
                        self._append(result.stdout)
                    if result.stderr:
                        self._append(result.stderr, C['warn'])
                except Exception as e:
                    self._append(f"Error: {e}\n", C['bad'])

            # ── Start build ──
            self._status.setText("Building…")
            self._append(f"$ {command}\n", C['ink_muted'])

            self._proc = QProcess(self)
            self._proc.setWorkingDirectory(work_dir)
            self._proc.setProcessChannelMode(QProcess.MergedChannels)
            self._proc.readyReadStandardOutput.connect(self._on_stdout)
            self._proc.finished.connect(self._on_finished)

            self._elapsed = 0
            self._timer = QTimer(self)
            self._timer.timeout.connect(self._tick)
            self._timer.start(1000)

            self._proc.start("bash", ["-c", command])

        def _append(self, text, color=None):
            cur = self._output.textCursor()
            cur.movePosition(QTextCursor.End)
            if color:
                cur.insertHtml(
                    f"<span style='color:{color}'>"
                    f"{text.replace(chr(10), '<br>')}</span>")
            else:
                cur.insertText(text)
            self._output.setTextCursor(cur)
            self._output.ensureCursorVisible()

        def _tick(self):
            self._elapsed += 1
            m, s = divmod(self._elapsed, 60)
            self._status.setText(f"Building… ({m}:{s:02d})")

        def _on_stdout(self):
            data = self._proc.readAllStandardOutput()
            text = bytes(data).decode("utf-8", errors="replace")
            text = re.sub(r'\x1b\[[0-9;]*m', '', text)
            self._append(text)

        def _on_finished(self, exit_code, _exit_status):
            self._finished = True
            self._exit_code = exit_code
            self._timer.stop()
            m, s = divmod(self._elapsed, 60)
            self._bar.setRange(0, 100)
            self._bar.setValue(100)

            if exit_code == 0:
                self._bar.setStyleSheet(_bar_style(C['ok']))
                self._status.setText(f"Build succeeded ({m}:{s:02d})")
                self._status.setStyleSheet(_status_style(C['ok']))
                self._append(f"\n--- Build completed successfully ({m}:{s:02d}) ---\n", C['ok'])
                # Auto-close 2 s after success so the installer keeps moving
                # without making the user click. Failures stay open so they
                # can read the log.
                self._cancel_btn.setEnabled(False)
                self._close_btn.setEnabled(True)
                self._close_btn.setFocus()
                QTimer.singleShot(2000, self.accept)
                return
            else:
                self._bar.setStyleSheet(_bar_style(C['bad']))
                self._status.setText(f"Build failed (exit code {exit_code})")
                self._status.setStyleSheet(_status_style(C['bad']))
                self._append(f"\n--- Build failed with exit code {exit_code} ({m}:{s:02d}) ---\n", C['bad'])

            self._cancel_btn.setEnabled(False)
            self._close_btn.setEnabled(True)
            self._close_btn.setFocus()

        def _on_cancel(self):
            if not self._finished and self._proc.state() != QProcess.NotRunning:
                self._proc.kill()
                self._append("\n--- Build cancelled by user ---\n", C['warn'])
                self._status.setText("Cancelled")
                self._status.setStyleSheet(_status_style(C['warn']))
                self._timer.stop()
                self._bar.setRange(0, 100)
                self._bar.setValue(0)
                self._cancel_btn.setEnabled(False)
                self._close_btn.setEnabled(True)
                self._finished = True
                self._exit_code = 2  # cancelled

        def closeEvent(self, event):
            if not self._finished and self._proc.state() != QProcess.NotRunning:
                self._proc.kill()
                self._proc.waitForFinished(3000)
                self._exit_code = 2
            event.accept()

    dlg = BuildDialog()
    dlg.show()
    app.exec_()
    return dlg._exit_code


def main():
    import argparse
    parser = argparse.ArgumentParser(description="FIC Control Studio Build GUI")
    parser.add_argument("work_dir", help="Working directory for the build")
    parser.add_argument("command", help="Shell command to run")
    parser.add_argument("--title", default="Building...", help="Dialog title")
    parser.add_argument("--pre-command", default=None, help="Command to run before build (e.g. clean)")
    args = parser.parse_args()

    if not os.path.isdir(args.work_dir):
        print(f"Error: directory not found: {args.work_dir}", file=sys.stderr)
        sys.exit(1)

    code = run_gui(args.work_dir, args.command,
                   title=args.title, pre_command=args.pre_command)
    sys.exit(code)


if __name__ == "__main__":
    main()
