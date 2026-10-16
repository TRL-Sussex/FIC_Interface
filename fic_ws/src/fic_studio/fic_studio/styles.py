# styles.py -- the structural stylesheet for FIC Studio.
#
# One QSS template, filled from design tokens (theme/design.py derive()) for
# whichever theme is active.  Quiet by default: buttons are outlines or bare
# text, hover and selection are faint washes, one accent carries focus and
# selection, and only safety controls are solid and saturated.
#
# Canonical button object names (setObjectName):
#   (default)    -- secondary: outline
#   BtnPrimary   -- accent outline (the one affirmative action on a page)
#   BtnQuiet     -- bare text, wash on hover
#   BtnTeal / BtnGreen / BtnWarn / BtnRed / BtnStop -- legacy semantic names:
#                   outline in the matching state colour
#   BtnEStop     -- SAFETY: solid red, the only filled button
#   ModeTab / UnitBtn / BtnToggle -- segmented, accent when active
#   SideButton   -- sidebar navigation [active]
#
# Usage: fic_studio.theme.apply_theme(window, 'warmth_dark')

# Legacy key list. The palette slots every theme fills; tokens.py is the
# source of values and test_theme_tokens checks each theme has exactly these.
COLORS_DARK = {
    'bg_base': '#1a1e24', 'bg_raised': '#22262c', 'bg_overlay': '#252a30',
    'bg_hover': '#2a2e36', 'bg_selected': '#1a5fb4',
    'border_subtle': '#2e333a', 'border_medium': '#3a4050',
    'border_strong': '#4a5060',
    'text_primary': '#c0c8d4', 'text_secondary': '#8a9ab0',
    'text_disabled': '#556070', 'text_on_accent': '#ffffff',
    'accent_blue': '#4a90d9', 'accent_teal': '#1a6b5a',
    'accent_green': '#2a6e3a', 'accent_orange': '#c05a10',
    'accent_red': '#a83232',
}

QSS_TEMPLATE = """
/* GLOBAL */
* { font-family: %(font_sans)s; }
QMainWindow, QWidget {
    background-color: %(canvas)s;
    color: %(ink)s;
    font-size: 12px;
}
QLabel, QCheckBox, QRadioButton, QSlider { background: transparent; }
QMainWindow::separator { background-color: %(rule)s; width: 1px; height: 1px; }

/* MENUS */
QMenuBar {
    background-color: %(canvas)s;
    color: %(ink)s;
    border-bottom: 1px solid %(rule)s;
    padding: 2px 0;
}
QMenuBar::item { padding: 5px 12px; background: transparent; border-radius: 3px; }
QMenuBar::item:selected { background-color: %(hover)s; }
QMenuBar::item:pressed { background-color: %(press)s; }
QMenu {
    background-color: %(panel)s;
    color: %(ink)s;
    border: 1px solid %(rule_strong)s;
    border-radius: 5px;
    padding: 4px 0;
}
QMenu::item { padding: 7px 28px 7px 20px; color: %(ink)s; background: transparent; }
QMenu::item:selected { background-color: %(hover)s; color: %(ink_strong)s; }
QMenu::item:disabled { color: %(ink_faint)s; }
QMenu::separator { height: 1px; background-color: %(rule)s; margin: 4px 12px; }

/* TOOLBAR */
QToolBar {
    background-color: %(canvas)s;
    border: none;
    border-bottom: 1px solid %(rule)s;
    padding: 6px 8px;
    spacing: 6px;
}
QToolBar::separator { background-color: %(rule)s; width: 1px; margin: 4px 8px; }
QToolBar QWidget { background: transparent; }
QToolBar QLabel {
    font-size: 11px;
    font-weight: 500;
    color: %(ink_muted)s;
    background: transparent;
}
QToolBar QLabel#BlueLabel { color: %(accent)s; }
QToolBar QComboBox {
    background-color: %(panel)s;
    border: 1px solid %(rule_strong)s;
    color: %(ink)s;
}
QToolBar QComboBox QAbstractItemView {
    background-color: %(panel)s;
    border: 1px solid %(rule_strong)s;
    color: %(ink)s;
    selection-background-color: %(hover)s;
    selection-color: %(ink_strong)s;
    outline: none;
    padding: 4px 0;
}
QToolBar QComboBox QAbstractItemView::item {
    padding: 7px 20px;
    min-height: 22px;
    background-color: transparent;
    border: none;
}
QToolButton {
    background-color: transparent;
    border: 1px solid transparent;
    border-radius: 5px;
    padding: 4px 8px;
    color: %(ink)s;
}
QToolButton:hover { background-color: %(hover)s; }
QToolButton:pressed, QToolButton:checked { background-color: %(press)s; }

/* STATUS BAR */
QStatusBar {
    background-color: %(canvas)s;
    color: %(ink_muted)s;
    border-top: 1px solid %(rule)s;
    font-size: 11px;
    min-height: 26px;
}
QStatusBar::item { border: none; }
QStatusBar QLabel { color: %(ink_muted)s; padding: 2px 8px; }

/* SIDEBAR */
#Sidebar { background-color: %(canvas)s; border-right: 1px solid %(rule)s; }
#SidebarCollapse {
    background: transparent;
    border: none;
    color: %(ink_muted)s;
    font-size: 16px;
    border-radius: 5px;
    padding: 4px;
}
#SidebarCollapse:hover { background-color: %(hover)s; color: %(ink_strong)s; }
#SidebarSearch {
    background-color: %(panel)s;
    border: 1px solid %(rule)s;
    border-radius: 5px;
    color: %(ink)s;
    padding: 6px 10px;
    font-size: 12px;
}
#SidebarSearch:focus { border-color: %(ink_muted)s; }
#SideSection {
    color: %(ink_faint)s;
    font-size: 10px;
    font-weight: 500;
    letter-spacing: 1px;
    padding: 18px 18px 6px 18px;
}
#SideButton, #SideButtonBottom {
    text-align: left;
    padding: 9px 18px;
    border: none;
    border-left: 2px solid transparent;
    background: transparent;
    color: %(ink_muted)s;
    font-size: 12px;
    font-weight: 400;
    border-radius: 0px;
}
#SideButton:hover, #SideButtonBottom:hover {
    background-color: %(hover)s;
    color: %(ink_strong)s;
}
#SideButton[active="true"] {
    background-color: %(selected)s;
    color: %(ink_strong)s;
    font-weight: 500;
    border-left: 2px solid %(accent)s;
}
#Divider {
    background-color: %(rule)s;
    min-height: 1px;
    max-height: 1px;
    border: none;
    margin: 4px 16px;
}

/* DASHBOARD CARDS AND BADGES */
#DashActionBtn {
    background-color: transparent;
    border: 1px solid %(rule_strong)s;
    border-radius: 5px;
    color: %(ink)s;
    font-size: 12px;
    font-weight: 400;
    padding: 7px 14px;
    text-align: left;
}
#DashActionBtn:hover { background-color: %(hover)s; border-color: %(border_strong)s; }
#DashActionBtnDanger {
    background-color: transparent;
    border: 1px solid %(err)s;
    border-radius: 5px;
    color: %(err)s;
    font-size: 12px;
    font-weight: 500;
    padding: 7px 14px;
    text-align: center;
}
#DashActionBtnDanger:hover { background-color: %(err_wash)s; }
#DashCard {
    background-color: %(panel)s;
    border: 1px solid %(rule)s;
    border-radius: 8px;
}
#DashCardTitle {
    color: %(ink_strong)s;
    font-size: 13px;
    font-weight: 600;
    background: transparent;
}
#DashStatBox {
    background-color: %(well)s;
    border: 1px solid %(rule)s;
    border-radius: 5px;
}
#DashModeBadge {
    color: %(ink)s;
    background-color: transparent;
    border: 1px solid %(rule_strong)s;
    border-radius: 10px;
    padding: 3px 10px;
    font-size: 11px;
    font-weight: 500;
    max-width: 250px;
}
#DashInfoBanner {
    background-color: %(accent_wash)s;
    border: 1px solid %(rule)s;
    border-radius: 5px;
}
#DashStatusBadge {
    font-size: 11px;
    padding: 2px 8px;
    border-radius: 8px;
    background: transparent;
}
#MainContent { background-color: %(canvas)s; }
#PageTitle {
    font-family: %(font_serif)s;
    font-size: 24px;
    font-weight: 400;
    color: %(ink_strong)s;
    padding: 12px 0;
}
#PageSubtitle { font-size: 11px; color: %(ink_muted)s; }
#SectionLabel {
    font-size: 10px;
    font-weight: 500;
    color: %(ink_muted)s;
    padding: 8px 0 4px 0;
    letter-spacing: 1px;
}

/* BUTTONS: secondary by default (outline), never a filled block */
QPushButton {
    background-color: transparent;
    border: 1px solid %(rule_strong)s;
    border-radius: 5px;
    color: %(ink)s;
    font-size: 12px;
    font-weight: 500;
    min-height: 28px;
    padding: 4px 14px;
}
QPushButton:hover { background-color: %(hover)s; border-color: %(border_strong)s; color: %(ink_strong)s; }
QPushButton:pressed { background-color: %(press)s; }
QPushButton:focus { border-color: %(ink_muted)s; }
QPushButton:disabled { color: %(ink_faint)s; border-color: %(rule)s; background-color: transparent; }
QPushButton#BtnQuiet {
    background-color: transparent;
    border: 1px solid transparent;
    color: %(ink_muted)s;
}
QPushButton#BtnQuiet:hover { background-color: %(hover)s; color: %(ink_strong)s; }
QPushButton#BtnQuiet:pressed { background-color: %(press)s; }
QPushButton#BtnQuiet:disabled { color: %(ink_faint)s; }
QPushButton#BtnPrimary, QPushButton#BtnTeal {
    background-color: transparent;
    border: 1px solid %(accent)s;
    color: %(accent)s;
}
QPushButton#BtnPrimary:hover, QPushButton#BtnTeal:hover {
    background-color: %(accent_wash)s;
    border-color: %(accent)s;
    color: %(accent)s;
}
QPushButton#BtnPrimary:pressed, QPushButton#BtnTeal:pressed { background-color: %(accent_wash_strong)s; }
QPushButton#BtnPrimary:disabled, QPushButton#BtnTeal:disabled {
    color: %(ink_faint)s;
    border-color: %(rule)s;
    background-color: transparent;
}
QPushButton#BtnGreen {
    background-color: transparent;
    border: 1px solid %(ok)s;
    color: %(ok)s;
}
QPushButton#BtnGreen:hover { background-color: %(ok_wash)s; border-color: %(ok)s; color: %(ok)s; }
QPushButton#BtnGreen:pressed { background-color: %(ok_wash)s; }
QPushButton#BtnGreen:disabled { color: %(ink_faint)s; border-color: %(rule)s; background-color: transparent; }
QPushButton#BtnWarn {
    background-color: transparent;
    border: 1px solid %(warn)s;
    color: %(warn)s;
}
QPushButton#BtnWarn:hover { background-color: %(warn_wash)s; border-color: %(warn)s; color: %(warn)s; }
QPushButton#BtnWarn:pressed { background-color: %(warn_wash)s; }
QPushButton#BtnWarn:disabled { color: %(ink_faint)s; border-color: %(rule)s; background-color: transparent; }
QPushButton#BtnRed, QPushButton#BtnStop {
    background-color: transparent;
    border: 1px solid %(err)s;
    color: %(err)s;
}
QPushButton#BtnRed:hover, QPushButton#BtnStop:hover {
    background-color: %(err_wash)s;
    border-color: %(err)s;
    color: %(err)s;
}
QPushButton#BtnRed:pressed, QPushButton#BtnStop:pressed { background-color: %(err_wash)s; }
QPushButton#BtnRed:disabled { color: %(ink_faint)s; border-color: %(rule)s; background-color: transparent; }

/* SEGMENTED CONTROLS: quiet until active */
QPushButton#ModeTab, QPushButton#UnitBtn {
    background-color: transparent;
    border: 1px solid %(rule)s;
    border-radius: 5px;
    padding: 6px 12px;
    color: %(ink_muted)s;
    font-size: 11px;
    font-weight: 400;
    min-height: 22px;
}
QPushButton#ModeTab:hover, QPushButton#UnitBtn:hover {
    background-color: %(hover)s;
    color: %(ink_strong)s;
}
QPushButton#ModeTab[active_j="true"], QPushButton#ModeTab[active_c="true"],
QPushButton#UnitBtn[active="true"] {
    background-color: %(accent_wash)s;
    border-color: %(accent)s;
    color: %(accent)s;
    font-weight: 500;
}

/* INPUTS */
QLineEdit, QTextEdit, QPlainTextEdit {
    background-color: %(panel)s;
    border: 1px solid %(rule_strong)s;
    border-radius: 5px;
    color: %(ink_strong)s;
    padding: 5px 9px;
    selection-background-color: %(selected)s;
    selection-color: %(ink_strong)s;
}
QLineEdit:hover, QTextEdit:hover, QPlainTextEdit:hover { border-color: %(border_strong)s; }
QLineEdit:focus, QTextEdit:focus, QPlainTextEdit:focus { border-color: %(accent)s; }
QLineEdit:disabled, QTextEdit:disabled, QPlainTextEdit:disabled {
    background-color: %(canvas)s;
    color: %(ink_faint)s;
    border-color: %(rule)s;
}
QComboBox {
    background-color: %(panel)s;
    border: 1px solid %(rule_strong)s;
    border-radius: 5px;
    color: %(ink_strong)s;
    padding: 5px 9px;
    min-height: 20px;
}
QComboBox:hover { border-color: %(border_strong)s; }
QComboBox:focus { border-color: %(accent)s; }
QComboBox:disabled { color: %(ink_faint)s; border-color: %(rule)s; }
QComboBox QAbstractItemView {
    background-color: %(panel)s;
    border: 1px solid %(rule_strong)s;
    color: %(ink)s;
    selection-background-color: %(hover)s;
    selection-color: %(ink_strong)s;
    outline: none;
    padding: 4px 0;
}
QComboBox QAbstractItemView::item {
    padding: 7px 20px;
    min-height: 22px;
    color: %(ink)s;
    background-color: transparent;
    border: none;
}
QComboBox QAbstractItemView::item:selected { background-color: %(hover)s; color: %(ink_strong)s; }
QComboBox QAbstractItemView::item:hover { background-color: %(hover)s; color: %(ink_strong)s; }
QComboBox QListView {
    background-color: %(panel)s;
    border: 1px solid %(rule_strong)s;
    color: %(ink)s;
    selection-background-color: %(hover)s;
    selection-color: %(ink_strong)s;
    outline: none;
}
QSpinBox, QDoubleSpinBox {
    background-color: %(panel)s;
    border: 1px solid %(rule_strong)s;
    border-radius: 5px;
    color: %(ink_strong)s;
    padding: 5px 9px;
    min-height: 20px;
}
QSpinBox:hover, QDoubleSpinBox:hover { border-color: %(border_strong)s; }
QSpinBox:focus, QDoubleSpinBox:focus { border-color: %(accent)s; }
QSpinBox::up-button, QDoubleSpinBox::up-button,
QSpinBox::down-button, QDoubleSpinBox::down-button {
    background-color: transparent;
    border: none;
    width: 18px;
    border-radius: 3px;
}
QSpinBox::up-button:hover, QDoubleSpinBox::up-button:hover,
QSpinBox::down-button:hover, QDoubleSpinBox::down-button:hover { background-color: %(hover)s; }
QSlider { background: transparent; }
QSlider::groove:horizontal { height: 2px; background: %(rule_strong)s; border: none; border-radius: 1px; }
QSlider::handle:horizontal {
    width: 12px;
    height: 12px;
    margin: -6px 0;
    border-radius: 6px;
    background: %(panel)s;
    border: 1px solid %(ink_muted)s;
}
QSlider::handle:horizontal:hover { border-color: %(accent)s; background: %(accent)s; }
QSlider::sub-page:horizontal { background: %(accent)s; border-radius: 1px; }
QSlider::add-page:horizontal { background: %(rule_strong)s; border-radius: 1px; }
QCheckBox { color: %(ink)s; spacing: 8px; font-size: 12px; }
QCheckBox::indicator {
    width: 14px;
    height: 14px;
    border: 1px solid %(border_strong)s;
    border-radius: 3px;
    background: %(panel)s;
}
QCheckBox::indicator:checked { background: %(accent)s; border-color: %(accent)s; }
QCheckBox::indicator:hover { border-color: %(accent)s; }
QCheckBox:disabled { color: %(ink_faint)s; }

/* PANELS */
QGroupBox {
    background-color: transparent;
    border: 1px solid %(rule)s;
    border-radius: 8px;
    margin-top: 14px;
    padding: 16px;
    padding-top: 26px;
    font-weight: 500;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    padding: 2px 12px;
    color: %(ink_muted)s;
    background-color: transparent;
    font-size: 11px;
    font-weight: 500;
    letter-spacing: 1px;
}
QGroupBox QWidget { background: transparent; }
QGroupBox QLineEdit, QGroupBox QTextEdit, QGroupBox QPlainTextEdit,
QGroupBox QSpinBox, QGroupBox QDoubleSpinBox, QGroupBox QComboBox { background-color: %(panel)s; }
QGroupBox QListWidget, QGroupBox QTableWidget, QGroupBox QTreeWidget { background-color: %(panel)s; }

/* TABS: text with an underline */
QTabWidget::pane {
    background-color: transparent;
    border: none;
    border-top: 1px solid %(rule)s;
}
QTabBar::tab {
    background-color: transparent;
    border: none;
    border-bottom: 2px solid transparent;
    color: %(ink_muted)s;
    padding: 8px 18px;
    font-size: 12px;
    font-weight: 400;
    min-width: 64px;
}
QTabBar::tab:selected {
    background-color: transparent;
    color: %(ink_strong)s;
    border-bottom: 2px solid %(accent)s;
    font-weight: 500;
}
QTabBar::tab:hover:!selected { background-color: %(hover)s; color: %(ink_strong)s; }
QScrollArea { border: none; background: transparent; }
QScrollBar:vertical { background-color: transparent; width: 8px; margin: 0; }
QScrollBar::handle:vertical { background-color: %(rule_strong)s; border-radius: 4px; min-height: 40px; margin: 1px; }
QScrollBar::handle:vertical:hover { background-color: %(border_strong)s; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical { background: transparent; }
QScrollBar:horizontal { background-color: transparent; height: 8px; }
QScrollBar::handle:horizontal { background-color: %(rule_strong)s; border-radius: 4px; min-width: 40px; margin: 1px; }
QScrollBar::handle:horizontal:hover { background-color: %(border_strong)s; }
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal { width: 0; }
QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal { background: transparent; }

/* TABLES, TREES, LISTS: no grid, a hairline under the header */
QTableWidget, QTreeWidget, QListWidget {
    background-color: %(panel)s;
    alternate-background-color: %(panel)s;
    border: 1px solid %(rule)s;
    gridline-color: %(rule)s;
    outline: none;
    font-size: 12px;
    border-radius: 5px;
}
QTableWidget::item, QTreeWidget::item, QListWidget::item { padding: 6px 10px; border: none; }
QTableWidget::item:selected, QTreeWidget::item:selected, QListWidget::item:selected {
    background-color: %(selected)s;
    color: %(ink_strong)s;
}
QTableWidget::item:hover, QTreeWidget::item:hover, QListWidget::item:hover { background-color: %(hover)s; }
QHeaderView::section {
    background-color: transparent;
    border: none;
    border-bottom: 1px solid %(rule)s;
    color: %(ink_muted)s;
    font-size: 11px;
    font-weight: 500;
    padding: 6px 10px;
}
QSplitter::handle { background-color: %(rule)s; }
QSplitter::handle:horizontal { width: 1px; }
QSplitter::handle:vertical { height: 1px; }
QSplitter::handle:hover { background-color: %(accent)s; }
QProgressBar { background-color: %(rule)s; border: none; border-radius: 2px; height: 4px; text-align: center; }
QProgressBar::chunk { background-color: %(accent)s; border-radius: 2px; }

/* TOOLTIPS, DIALOGS */
QToolTip {
    background-color: %(panel)s;
    border: 1px solid %(rule_strong)s;
    color: %(ink)s;
    padding: 5px 9px;
    font-size: 11px;
    border-radius: 5px;
}
QDialog { background-color: %(canvas)s; }
QMessageBox { background-color: %(canvas)s; }
QMessageBox QLabel { color: %(ink_strong)s; }
QDialogButtonBox QPushButton { min-width: 72px; }
QDockWidget { color: %(ink)s; font-weight: 500; }
QDockWidget::title { background-color: %(canvas)s; padding: 6px 10px; border-bottom: 1px solid %(rule)s; }
QDockWidget::close-button, QDockWidget::float-button { background: transparent; border: none; }
QDockWidget::close-button:hover, QDockWidget::float-button:hover { background-color: %(hover)s; }

/* LOG, FOOTER, STATUS INDICATORS */
#LogViewer {
    background-color: %(canvas)s;
    border: 1px solid %(rule)s;
    font-family: %(font_mono)s;
    font-size: 11px;
}
#LogViewerHeader { background-color: %(canvas)s; border-bottom: 1px solid %(rule)s; }
#Footer { background-color: %(canvas)s; border-top: 1px solid %(rule)s; }
#SideFooter { border-top: 1px solid %(rule)s; padding: 12px 16px; }
#SideFooterLabel { font-size: 10px; letter-spacing: 1px; color: %(ink_faint)s; padding-bottom: 6px; }
#StatusDot { min-width: 8px; max-width: 8px; min-height: 8px; max-height: 8px; border-radius: 4px; }
#NodePill {
    background-color: transparent;
    border: 1px solid %(rule)s;
    border-radius: 5px;
    padding: 8px 12px;
    margin: 10px 16px;
}
#NodeStatusLabel { font-size: 10px; letter-spacing: 1px; color: %(ink_faint)s; }
#NodeStatusValue { font-size: 12px; color: %(ok)s; font-weight: 500; }
#QuatPreview { font-size: 10px; color: %(ink_muted)s; font-family: %(font_mono)s; }
#HintLabel { color: %(ink_muted)s; font-size: 11px; }
#EStopLabel { color: %(err)s; font-weight: 700; font-size: 13px; }
#LogToggleBtn { font-size: 10px; font-weight: 500; color: %(ink_muted)s; border: none; background: transparent; }
#HelpContent { background-color: %(panel)s; border: none; padding: 8px; }
#AccentValue { color: %(accent)s; font-weight: 500; }
#JoystickLabel { color: %(ink_muted)s; font-size: 9px; font-weight: 500; }
#UnitFrame { border: 1px solid %(rule)s; border-radius: 5px; }
#CodeEditor {
    background-color: %(panel)s;
    color: %(ink)s;
    border: 1px solid %(rule)s;
    border-radius: 5px;
    font-family: %(font_mono)s;
}
#OutputConsole {
    background-color: %(well)s;
    color: %(ink)s;
    font-family: %(font_mono)s;
    font-size: 11px;
    border: 1px solid %(rule)s;
    border-radius: 5px;
}
#ApiReference {
    background-color: %(panel)s;
    color: %(ink)s;
    font-size: 11px;
    border: 1px solid %(rule)s;
    border-radius: 5px;
}

/* PROCESS TOGGLES: outline idle, error-coloured outline while running */
QPushButton#BtnToolHW { background-color: transparent; border: 1px solid %(rule_strong)s; color: %(accent)s; }
QPushButton#BtnToolSim { background-color: transparent; border: 1px solid %(rule_strong)s; color: %(accent)s; }
QPushButton#BtnToolUtil { background-color: transparent; border: 1px solid %(rule_strong)s; color: %(ink)s; }
QPushButton#BtnToolHW:hover, QPushButton#BtnToolSim:hover, QPushButton#BtnToolUtil:hover { background-color: %(hover)s; }
QPushButton#BtnToolHW[running="true"], QPushButton#BtnToolSim[running="true"],
QPushButton#BtnToolUtil[running="true"] { background-color: transparent; border: 1px solid %(err)s; color: %(err)s; }
QPushButton#BtnToolHW[running="true"]:hover, QPushButton#BtnToolSim[running="true"]:hover,
QPushButton#BtnToolUtil[running="true"]:hover { background-color: %(err_wash)s; }

/* SAFETY: the only solid, saturated controls. Never quiet. */
QPushButton#BtnEStop {
    background-color: %(safety)s;
    border: 2px solid %(safety)s;
    color: %(safety_text)s;
    font-weight: 700;
    border-radius: 5px;
}
QPushButton#BtnEStop:hover { background-color: %(safety_hover)s; border-color: %(safety_hover)s; }
QPushButton#BtnToggle {
    background-color: transparent;
    border: 1px solid %(rule)s;
    color: %(ink_muted)s;
    font-size: 10px;
    border-radius: 5px;
}
QPushButton#BtnToggle:hover { background-color: %(hover)s; }
QPushButton#BtnToggle:checked {
    background-color: %(accent_wash)s;
    border: 1px solid %(accent)s;
    color: %(accent)s;
    font-size: 10px;
}
QPushButton#BtnToggle:checked:hover { background-color: %(accent_wash_strong)s; }
"""
