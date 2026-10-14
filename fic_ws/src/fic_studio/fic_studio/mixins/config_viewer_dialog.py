# config_viewer_dialog.py — Reusable popup for viewing/editing filtered config files
# Supports both XML (config.xml) and YAML (robots_config.yaml) with syntax highlighting

import os
import re
import yaml
import xml.etree.ElementTree as ET

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont, QSyntaxHighlighter, QTextCharFormat, QColor
from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QTextEdit, QMessageBox,
)

from fic_studio import theme


class XMLHighlighter(QSyntaxHighlighter):
    """Syntax highlighter for XML content."""
    def highlightBlock(self, text):
        # Tags
        fmt = QTextCharFormat()
        fmt.setForeground(QColor("#C56A42"))
        for m in re.finditer(r'</?[\w:]+', text):
            self.setFormat(m.start(), m.end() - m.start(), fmt)
        for m in re.finditer(r'/?>', text):
            self.setFormat(m.start(), m.end() - m.start(), fmt)
        # Attribute names
        attr_fmt = QTextCharFormat()
        attr_fmt.setForeground(QColor("#D29A3C"))
        for m in re.finditer(r'[\w:]+(?==)', text):
            self.setFormat(m.start(), m.end() - m.start(), attr_fmt)
        # Attribute values
        val_fmt = QTextCharFormat()
        val_fmt.setForeground(QColor("#B5764F"))
        for m in re.finditer(r'"[^"]*"', text):
            self.setFormat(m.start(), m.end() - m.start(), val_fmt)
        # Comments
        cmt_fmt = QTextCharFormat()
        cmt_fmt.setForeground(QColor("#8A9A52"))
        for m in re.finditer(r'<!--.*?-->', text):
            self.setFormat(m.start(), m.end() - m.start(), cmt_fmt)
        # Text content between tags
        txt_fmt = QTextCharFormat()
        txt_fmt.setForeground(QColor("#B3A897"))
        for m in re.finditer(r'>([^<]+)<', text):
            self.setFormat(m.start(1), m.end(1) - m.start(1), txt_fmt)


class YAMLHighlighter(QSyntaxHighlighter):
    """Syntax highlighter for YAML content."""
    def highlightBlock(self, text):
        # Keys (before colon)
        key_fmt = QTextCharFormat()
        key_fmt.setForeground(QColor("#C56A42"))
        for m in re.finditer(r'^(\s*)([\w_-]+)(\s*:)', text):
            self.setFormat(m.start(2), m.end(2) - m.start(2), key_fmt)
        # Numbers
        num_fmt = QTextCharFormat()
        num_fmt.setForeground(QColor("#D98A5E"))
        for m in re.finditer(r':\s+(-?[\d.]+)\s*$', text):
            self.setFormat(m.start(1), m.end(1) - m.start(1), num_fmt)
        # Booleans
        bool_fmt = QTextCharFormat()
        bool_fmt.setForeground(QColor("#C56A42"))
        bool_fmt.setFontWeight(QFont.Bold)
        for m in re.finditer(r':\s+(true|false)\s*$', text, re.IGNORECASE):
            self.setFormat(m.start(1), m.end(1) - m.start(1), bool_fmt)
        # Strings in quotes
        str_fmt = QTextCharFormat()
        str_fmt.setForeground(QColor("#B5764F"))
        for m in re.finditer(r"'[^']*'|\"[^\"]*\"", text):
            self.setFormat(m.start(), m.end() - m.start(), str_fmt)
        # Comments
        cmt_fmt = QTextCharFormat()
        cmt_fmt.setForeground(QColor("#8A9A52"))
        for m in re.finditer(r'#.*$', text):
            self.setFormat(m.start(), m.end() - m.start(), cmt_fmt)
        # List dashes
        dash_fmt = QTextCharFormat()
        dash_fmt.setForeground(QColor("#B3A897"))
        dash_fmt.setFontWeight(QFont.Bold)
        for m in re.finditer(r'^\s*-\s', text):
            self.setFormat(m.start(), m.end() - m.start(), dash_fmt)


class ConfigViewerDialog(QDialog):
    """Resizable popup dialog for viewing/editing a single robot's config section."""

    def __init__(self, parent=None, file_path="", robot_name="", file_type="xml"):
        super().__init__(parent)
        self.file_path = file_path
        self.robot_name = robot_name
        self.file_type = file_type
        self._original_text = ""
        self._has_unsaved_changes = False

        self.setWindowTitle(f"{os.path.basename(file_path)} \u2014 {robot_name}")
        self.setMinimumSize(700, 500)
        self.resize(850, 650)
        self.setSizeGripEnabled(True)

        self._setup_ui()
        self._load_section()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)

        # Header
        header = QHBoxLayout()
        file_label = QLabel(f"{os.path.basename(self.file_path)}")
        theme.bind(
            file_label,
            lambda t: "font-size: 14px; font-weight: 600; color: %s;" % t["text_primary"]
        )
        header.addWidget(file_label)
        header.addStretch()
        robot_label = QLabel(f"Robot: {self.robot_name}")
        theme.bind(
            robot_label,
            lambda t: (
                "font-size: 12px; color: %s; font-weight: 600; "
                "background-color: %s; padding: 4px 12px; border-radius: 3px;"
            ) % (t["accent_blue"], t["bg_overlay"])
        )
        header.addWidget(robot_label)
        layout.addLayout(header)

        # Info bar
        self._info_label = QLabel("")
        theme.bind(
            self._info_label,
            lambda t: "color: %s; font-size: 11px;" % t["text_secondary"]
        )
        layout.addWidget(self._info_label)

        # Editor
        self.editor = QTextEdit()
        self.editor.setFont(QFont("Consolas", 11))
        self.editor.setLineWrapMode(QTextEdit.NoWrap)
        theme.bind(
            self.editor,
            lambda t: (
                "QTextEdit {"
                " background-color: %s;"
                " color: %s;"
                " border: 1px solid %s;"
                " border-radius: 4px;"
                " padding: 8px;"
                " selection-background-color: %s;"
                "}"
            ) % (
                t["bg_base"], t["text_primary"],
                t["border_subtle"], t["bg_selected"]
            )
        )
        self.editor.textChanged.connect(self._on_text_changed)

        if self.file_type == "xml":
            self._highlighter = XMLHighlighter(self.editor.document())
        else:
            self._highlighter = YAMLHighlighter(self.editor.document())

        layout.addWidget(self.editor, 1)

        # Validation status
        self._validation_label = QLabel("")
        self._validation_label.setStyleSheet("font-size: 11px;")
        layout.addWidget(self._validation_label)

        # Button row
        btn_row = QHBoxLayout()

        validate_btn = QPushButton("Validate")
        validate_btn.setToolTip("Check syntax without saving")
        validate_btn.clicked.connect(self._validate)
        btn_row.addWidget(validate_btn)

        revert_btn = QPushButton("Revert")
        revert_btn.setToolTip("Discard changes and reload from disk")
        revert_btn.clicked.connect(self._revert)
        btn_row.addWidget(revert_btn)

        btn_row.addStretch()

        self._save_btn = QPushButton("Save")
        theme.bind(
            self._save_btn,
            lambda t: (
                "background-color: %s; border: 1px solid %s; "
                "color: %s; font-weight: 600; padding: 6px 20px;"
            ) % (t["bg_overlay"], t["border_medium"], t["accent_blue"])
        )
        self._save_btn.setToolTip("Validate and save changes back to file")
        self._save_btn.clicked.connect(self._save)
        btn_row.addWidget(self._save_btn)

        close_btn = QPushButton("Close")
        close_btn.clicked.connect(self._close_dialog)
        btn_row.addWidget(close_btn)

        layout.addLayout(btn_row)

        theme.bind(
            self,
            lambda t: (
                "QDialog {"
                " background-color: %s;"
                " color: %s;"
                "}"
                " QPushButton {"
                " background-color: %s;"
                " border: 1px solid %s;"
                " border-radius: 4px;"
                " color: %s;"
                " padding: 6px 16px;"
                " min-height: 28px;"
                "}"
                " QPushButton:hover {"
                " background-color: %s;"
                " border-color: %s;"
                "}"
                " QLabel {"
                " color: %s;"
                "}"
            ) % (
                t["bg_base"], t["text_primary"],
                t["bg_raised"], t["border_subtle"], t["text_primary"],
                t["bg_hover"], t["border_medium"],
                t["text_primary"],
            )
        )

    # ─────────────────────────────────────────────────────────────
    #  Load
    # ─────────────────────────────────────────────────────────────

    def _load_section(self):
        if not os.path.exists(self.file_path):
            self.editor.setPlainText(f"# File not found: {self.file_path}")
            return

        if self.file_type == "xml":
            self._load_xml_section()
        else:
            self._load_yaml_section()

        self._original_text = self.editor.toPlainText()
        self._has_unsaved_changes = False
        self._update_info()

    def _load_xml_section(self):
        try:
            tree = ET.parse(self.file_path)
            root = tree.getroot()

            for robot_elem in root.findall('.//robot'):
                if robot_elem.get('name') == self.robot_name:
                    ET.indent(robot_elem, space="  ")
                    section = ET.tostring(robot_elem, encoding='unicode')
                    self.editor.setPlainText(section)
                    self._set_validation_ok()
                    return

            self.editor.setPlainText(
                f"<!-- Robot '{self.robot_name}' not found in {os.path.basename(self.file_path)} -->")
        except ET.ParseError as e:
            self.editor.setPlainText(f"<!-- XML Parse Error: {e} -->")
            self._set_validation_error(str(e))

    def _load_yaml_section(self):
        try:
            with open(self.file_path, 'r') as f:
                all_configs = yaml.safe_load(f)

            robots = all_configs
            if isinstance(all_configs, dict) and 'robots' in all_configs:
                robots = all_configs['robots']

            if isinstance(robots, dict) and self.robot_name in robots:
                section = {self.robot_name: robots[self.robot_name]}
                self.editor.setPlainText(
                    yaml.dump(section, default_flow_style=False, sort_keys=False))
                self._set_validation_ok()
            else:
                self.editor.setPlainText(
                    f"# Robot '{self.robot_name}' not found")
        except Exception as e:
            self.editor.setPlainText(f"# YAML Error: {e}")
            self._set_validation_error(str(e))

    # ─────────────────────────────────────────────────────────────
    #  Validation
    # ─────────────────────────────────────────────────────────────

    def _validate(self) -> bool:
        text = self.editor.toPlainText()
        if self.file_type == "xml":
            return self._validate_xml(text)
        else:
            return self._validate_yaml(text)

    def _validate_xml(self, text) -> bool:
        try:
            ET.fromstring(text)
            self._set_validation_ok()
            return True
        except ET.ParseError as e:
            self._set_validation_error(f"XML error: {e}")
            return False

    def _validate_yaml(self, text) -> bool:
        try:
            parsed = yaml.safe_load(text)
            if not isinstance(parsed, dict):
                self._set_validation_error("Root must be a YAML mapping")
                return False
            self._set_validation_ok()
            return True
        except yaml.YAMLError as e:
            self._set_validation_error(f"YAML error: {e}")
            return False

    def _set_validation_ok(self):
        self._validation_label.setText("Syntax OK")
        theme.bind(
            self._validation_label,
            lambda t: "color: %s; font-size: 11px;" % t["accent_green"]
        )

    def _set_validation_error(self, msg):
        self._validation_label.setText(f"Error: {msg}")
        theme.bind(
            self._validation_label,
            lambda t: "color: %s; font-size: 11px;" % t["accent_red"]
        )

    # ─────────────────────────────────────────────────────────────
    #  Save
    # ─────────────────────────────────────────────────────────────

    def _save(self):
        if not self._validate():
            QMessageBox.warning(self, "Cannot Save",
                "Fix the syntax errors before saving.")
            return

        reply = QMessageBox.question(self, "Save Changes?",
            f"Save changes to {os.path.basename(self.file_path)}?\n\n"
            f"This will update the {self.robot_name} section only.\n"
            "Other robots are not affected.",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

        if reply != QMessageBox.Yes:
            return

        if self.file_type == "xml":
            self._save_xml_section()
        else:
            self._save_yaml_section()

    def _save_xml_section(self):
        text = self.editor.toPlainText()
        try:
            edited_elem = ET.fromstring(text)
            tree = ET.parse(self.file_path)
            root = tree.getroot()

            replaced = False
            for parent in root.iter():
                for i, child in enumerate(list(parent)):
                    if child.tag == 'robot' and child.get('name') == self.robot_name:
                        parent.remove(child)
                        parent.insert(i, edited_elem)
                        replaced = True
                        break
                if replaced:
                    break

            if not replaced:
                QMessageBox.warning(self, "Save Error",
                    f"Could not find robot '{self.robot_name}' in the full file.")
                return

            ET.indent(root, space="  ")
            from fic_studio.xml_manager import write_tree_atomic
            write_tree_atomic(tree, self.file_path, encoding='unicode')   # the controllers may be reading it
            self._original_text = self.editor.toPlainText()
            self._has_unsaved_changes = False
            self._update_info()
            QMessageBox.information(self, "Saved",
                f"config.xml updated for {self.robot_name}.")

        except Exception as e:
            QMessageBox.critical(self, "Save Failed", f"Error saving: {e}")

    def _save_yaml_section(self):
        text = self.editor.toPlainText()
        try:
            edited = yaml.safe_load(text)

            with open(self.file_path, 'r') as f:
                full = yaml.safe_load(f)

            if 'robots' in full and isinstance(full['robots'], dict):
                if self.robot_name in edited:
                    full['robots'][self.robot_name] = edited[self.robot_name]
                else:
                    QMessageBox.warning(self, "Key Mismatch",
                        f"Expected root key '{self.robot_name}' in edited content.")
                    return
            else:
                if self.robot_name in edited:
                    full[self.robot_name] = edited[self.robot_name]
                else:
                    QMessageBox.warning(self, "Key Mismatch",
                        f"Expected root key '{self.robot_name}' in edited content.")
                    return

            with open(self.file_path, 'w') as f:
                yaml.dump(full, f, default_flow_style=False, sort_keys=False)

            self._original_text = self.editor.toPlainText()
            self._has_unsaved_changes = False
            self._update_info()
            QMessageBox.information(self, "Saved",
                f"robots_config.yaml updated for {self.robot_name}.")

        except Exception as e:
            QMessageBox.critical(self, "Save Failed", f"Error saving: {e}")

    # ─────────────────────────────────────────────────────────────
    #  Revert / Close
    # ─────────────────────────────────────────────────────────────

    def _revert(self):
        if self._has_unsaved_changes:
            reply = QMessageBox.question(self, "Revert?",
                "Discard all unsaved changes and reload from disk?",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if reply != QMessageBox.Yes:
                return
        self._load_section()

    def _close_dialog(self):
        if self._has_unsaved_changes:
            reply = QMessageBox.question(self, "Unsaved Changes",
                "You have unsaved changes. Close anyway?",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if reply != QMessageBox.Yes:
                return
        self.accept()

    def closeEvent(self, event):
        if self._has_unsaved_changes:
            reply = QMessageBox.question(self, "Unsaved Changes",
                "You have unsaved changes. Close anyway?",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if reply != QMessageBox.Yes:
                event.ignore()
                return
        event.accept()

    # ─────────────────────────────────────────────────────────────
    #  UI helpers
    # ─────────────────────────────────────────────────────────────

    def _on_text_changed(self):
        self._has_unsaved_changes = (self.editor.toPlainText() != self._original_text)
        self._update_info()

    def _update_info(self):
        text = self.editor.toPlainText()
        lines = text.count('\n') + 1
        chars = len(text)
        modified = " (modified)" if self._has_unsaved_changes else ""
        self._info_label.setText(
            f"{lines} lines, {chars} chars{modified}  \u2014  "
            f"Showing: {self.robot_name} only")
