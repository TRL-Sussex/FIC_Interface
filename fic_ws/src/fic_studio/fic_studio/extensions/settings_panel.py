import os
import sys
from typing import Callable, List

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QCheckBox, QFrame, QHBoxLayout, QLabel, QMessageBox,
    QPushButton, QScrollArea, QVBoxLayout, QWidget,
)

from .manifest import ExtensionManifest


class ExtensionsSettingsPanel(QWidget):
    def __init__(
        self,
        manifests: List[ExtensionManifest],
        enabled_ids: List[str],
        on_apply: Callable[[List[str]], None],
        parent: QWidget = None,
    ):
        super().__init__(parent)
        self._manifests = manifests
        self._initial_enabled = set(enabled_ids)
        self._on_apply = on_apply
        self._checkboxes = {}
        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setAlignment(Qt.AlignTop)

        header = QLabel(
            "Extensions add optional pages to the Studio. Toggle below and click "
            "APPLY — a restart is required for changes to take effect."
        )
        header.setWordWrap(True)
        layout.addWidget(header)

        if not self._manifests:
            empty = QLabel(
                "No extensions found.\n\n"
                "Each extension lives in Extensions/<name>/ with a manifest.json "
                "and an extension.py exporting class Extension."
            )
            empty.setObjectName("HintLabel")
            empty.setWordWrap(True)
            layout.addWidget(empty)
            layout.addStretch(1)
            return

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        inner = QWidget()
        inner_layout = QVBoxLayout(inner)
        inner_layout.setAlignment(Qt.AlignTop)

        for m in self._manifests:
            row = QFrame()
            row.setObjectName("ExtensionRow")
            row_layout = QHBoxLayout(row)
            row_layout.setContentsMargins(8, 6, 8, 6)
            cb = QCheckBox(f"{m.sidebar_icon}  {m.name}".strip())
            cb.setChecked(m.id in self._initial_enabled)
            self._checkboxes[m.id] = cb
            row_layout.addWidget(cb)
            desc = QLabel(m.description or "")
            desc.setObjectName("HintLabel")
            desc.setWordWrap(True)
            row_layout.addWidget(desc, 1)
            inner_layout.addWidget(row)

        inner_layout.addStretch(1)
        scroll.setWidget(inner)
        layout.addWidget(scroll, 1)

        button_row = QHBoxLayout()
        button_row.addStretch(1)
        apply_btn = QPushButton("APPLY")
        apply_btn.setObjectName("BtnPrimary")
        apply_btn.clicked.connect(self._apply)
        button_row.addWidget(apply_btn)
        layout.addLayout(button_row)

    def _apply(self):
        new_enabled = sorted(eid for eid, cb in self._checkboxes.items() if cb.isChecked())
        if set(new_enabled) == self._initial_enabled:
            QMessageBox.information(self, "Extensions", "No changes to apply.")
            return

        self._on_apply(new_enabled)
        self._initial_enabled = set(new_enabled)

        reply = QMessageBox.question(
            self,
            "Restart required",
            "Extension changes are saved but only take effect after restart.\n\n"
            "Restart now?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.Yes,
        )
        if reply == QMessageBox.Yes:
            os.execv(sys.executable, [sys.executable] + sys.argv)
