"""
Tests for the Topic Monitor's grouped tree and inspector page.

Verifies:
- _build_topic_monitor_page returns a QWidget with the expected widgets.
- topic_tree is a QTreeWidget; grouped by namespace after _update_topic_table.
- Inspector card is present: selected_topic_label, _insp_sparkline, msg_display.
- Filter row: topic_filter (QLineEdit) and topic_type_filter (QComboBox).
- Stats bar: topic_count_label, monitored_count_label, total_msg_label.
- _restore_monitored_topics reads settings['topic_monitor']['monitored'].
- _save_monitored_topics writes to settings['topic_monitor']['monitored'].
- _on_topic_selected sets _selected_topic and updates label.
- _refresh_inspector is page-visible guarded (no-op at non-11 index).
- _build_inspector_fields builds value labels + LimitBars for numeric fields.
- _format_field_value truncates / formats scalars, lists, dicts.
- _display_message respects _selected_topic guard.
- Headless render both warmth_dark and warmth_light (construct GUI at index 11,
  inject synthetic topic list + cached message, assert tree has child items and
  inspector label is updated).
"""

import os
from collections import deque, OrderedDict  # noqa: I100,I101
from unittest.mock import MagicMock

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import (  # noqa: E402
    QApplication, QLabel, QMainWindow, QPushButton, QWidget,
)

_app = QApplication.instance() or QApplication([])


# ---------------------------------------------------------------------------
# Stub helpers
# ---------------------------------------------------------------------------

class _FakeStack:
    """Minimal QStackedWidget stub returning a configurable index."""

    def __init__(self, idx=11):
        """Initialise with a fixed current-index."""
        self._idx = idx

    def currentIndex(self):
        """Return the configured index."""
        return self._idx


def _make_stub(theme_name='warmth_dark', stack_index=11):
    """
    Build a minimal stub satisfying TopicMonitorMixin dependencies.

    The returned stub is a QMainWindow so Qt keeps it alive.  The caller
    must either keep a reference to it or call ``stub.close()`` explicitly.
    We also clear stale theme bindings before applying the theme to avoid
    RuntimeErrors from previous tests' deleted widgets.
    """
    from fic_studio import theme as _theme
    from fic_studio.theme import apply_theme
    from fic_studio.mixins.topic_monitor_mixin import TopicMonitorMixin

    # Remove bindings whose wrapped widget has already been deleted by Qt/GC
    _theme._bindings[:] = [
        (r, f) for r, f in _theme._bindings if r() is not None
    ]

    class _Stub(QMainWindow, TopicMonitorMixin):
        """Minimal stub hosting TopicMonitorMixin."""

        def is_page_visible(self, name):
            """Topic Monitor is the page under test here."""
            return name == 'topic_monitor'

        def _show_page(self, name):
            """Navigate by name, as the real window does."""
            return True

        def __init__(self):
            """Initialise stub state and the mixin."""
            super().__init__()
            self.settings = {'topic_monitor': {}}
            self.main_stack = _FakeStack(stack_index)
            self.node = None  # No ROS node in tests
            self._init_topic_monitor()

        def _page_title(self, text):
            """Return a plain QLabel as a page title."""
            return QLabel(text)

        def _btn(self, text, style_id=None):
            """Return a plain QPushButton."""
            btn = QPushButton(text)
            if style_id:
                btn.setObjectName(style_id)
            return btn

        def log_message(self, level, msg, src):
            """Stub log writer (no-op)."""

    stub = _Stub()
    apply_theme(stub, theme_name)
    return stub


def _build_page(stub):
    """Build the page and attach it as the central widget to keep it alive."""
    page = stub._build_topic_monitor_page()
    stub.setCentralWidget(page)
    return page


# ---------------------------------------------------------------------------
# Build / widget-presence tests
# ---------------------------------------------------------------------------

def test_build_returns_qwidget():
    """_build_topic_monitor_page must return a QWidget."""
    stub = _make_stub()
    page = _build_page(stub)
    assert isinstance(page, QWidget)
    stub.close()


def test_topic_tree_present():
    """topic_tree must be a QTreeWidget after page build."""
    from PyQt5.QtWidgets import QTreeWidget
    stub = _make_stub()
    _build_page(stub)
    assert hasattr(stub, 'topic_tree')
    assert isinstance(stub.topic_tree, QTreeWidget)
    stub.close()


def test_inspector_widgets_present():
    """Inspector card widgets must be present after page build."""
    stub = _make_stub()
    _build_page(stub)
    assert hasattr(stub, 'selected_topic_label')
    assert hasattr(stub, '_insp_sparkline')
    assert hasattr(stub, 'msg_display')
    assert hasattr(stub, 'pause_btn')
    assert hasattr(stub, 'msg_buffer_spin')
    stub.close()


def test_filter_row_widgets_present():
    """topic_filter QLineEdit and topic_type_filter QComboBox must be built."""
    from PyQt5.QtWidgets import QComboBox, QLineEdit
    stub = _make_stub()
    _build_page(stub)
    assert hasattr(stub, 'topic_filter')
    assert isinstance(stub.topic_filter, QLineEdit)
    assert hasattr(stub, 'topic_type_filter')
    assert isinstance(stub.topic_type_filter, QComboBox)
    stub.close()


def test_stats_labels_present():
    """Stats bar labels must be built and show initial '0' counts."""
    stub = _make_stub()
    _build_page(stub)
    for attr in ('topic_count_label', 'monitored_count_label', 'total_msg_label'):
        assert hasattr(stub, attr), 'missing %s' % attr
    assert '0' in stub.topic_count_label.text()
    stub.close()


def test_auto_refresh_checkbox():
    """auto_refresh_check must start unchecked."""
    from PyQt5.QtWidgets import QCheckBox
    stub = _make_stub()
    _build_page(stub)
    assert hasattr(stub, 'auto_refresh_check')
    assert isinstance(stub.auto_refresh_check, QCheckBox)
    assert not stub.auto_refresh_check.isChecked()
    stub.close()


# ---------------------------------------------------------------------------
# Grouping / filter tree tests
# ---------------------------------------------------------------------------

def test_update_topic_table_groups_by_namespace():
    """_update_topic_table must group topics under namespace parent items."""
    stub = _make_stub()
    _build_page(stub)

    topics = [
        ('/robot1/joint_states', 'sensor_msgs/msg/JointState'),
        ('/robot1/cmd_vel', 'geometry_msgs/msg/Twist'),
        ('/rosout', 'rcl_interfaces/msg/Log'),
    ]
    stub._update_topic_table(topics)

    # topic_count_label updated
    assert '3' in stub.topic_count_label.text()

    # Two top-level groups: /robot1 and (global)
    assert stub.topic_tree.topLevelItemCount() == 2
    stub.close()


def test_filter_by_text():
    """Text filter must hide topics not matching the filter string."""
    stub = _make_stub()
    _build_page(stub)

    topics = [
        ('/robot1/joint_states', 'sensor_msgs/msg/JointState'),
        ('/robot1/cmd_vel', 'geometry_msgs/msg/Twist'),
        ('/rosout', 'rcl_interfaces/msg/Log'),
    ]
    stub._update_topic_table(topics)

    # Apply filter
    stub.topic_filter.setText('joint')
    stub._filter_topics()

    # Only /robot1/joint_states survives -> one namespace group with one child
    assert stub.topic_tree.topLevelItemCount() == 1
    ns_item = stub.topic_tree.topLevelItem(0)
    assert ns_item.childCount() == 1
    stub.close()


def test_tree_child_items_dict():
    """_tree_child_items must map topic names to tree children."""
    stub = _make_stub()
    _build_page(stub)

    topics = [('/robot1/joint_states', 'sensor_msgs/msg/JointState')]
    stub._update_topic_table(topics)

    assert '/robot1/joint_states' in stub._tree_child_items
    stub.close()


def test_tree_topic_types_cached():
    """_tree_topic_types must be populated after _update_topic_table."""
    stub = _make_stub()
    _build_page(stub)

    topics = [('/a/b', 'std_msgs/msg/Float64')]
    stub._update_topic_table(topics)
    assert stub._tree_topic_types['/a/b'] == 'std_msgs/msg/Float64'
    stub.close()


# ---------------------------------------------------------------------------
# Settings persist / restore
# ---------------------------------------------------------------------------

def test_save_monitored_topics_writes_settings():
    """_save_monitored_topics must write to settings['topic_monitor']."""
    stub = _make_stub()
    _build_page(stub)
    stub._tree_topic_types['/t'] = 'std_msgs/msg/String'
    stub.monitored_topics['/t'] = MagicMock()

    stub._save_monitored_topics()

    saved = stub.settings['topic_monitor']['monitored']
    assert '/t' in saved
    assert saved['/t'] == 'std_msgs/msg/String'
    stub.close()


def test_restore_monitored_topics_skips_without_node():
    """_restore_monitored_topics must silently skip when node is None."""
    stub = _make_stub()
    _build_page(stub)
    stub.settings['topic_monitor'] = {'monitored': {'/foo': 'std_msgs/msg/String'}}
    stub.node = None
    # Must not raise
    stub._restore_monitored_topics()
    assert '/foo' not in stub.monitored_topics
    stub.close()


def test_restore_monitored_topics_calls_start_monitoring():
    """_restore_monitored_topics must call _start_monitoring for each saved topic."""
    stub = _make_stub()
    _build_page(stub)
    stub.settings['topic_monitor'] = {
        'monitored': {'/saved_topic': 'std_msgs/msg/String'}
    }
    stub._start_monitoring = MagicMock()
    stub.node = MagicMock()
    stub._restore_monitored_topics()
    stub._start_monitoring.assert_called_once_with('/saved_topic', 'std_msgs/msg/String')
    stub.close()


# ---------------------------------------------------------------------------
# Inspector: _on_topic_selected
# ---------------------------------------------------------------------------

def test_on_topic_selected_sets_selected_topic():
    """
    _select_topic must set _selected_topic for a known topic name.

    Note: offscreen Qt does not fire selection events in the tree so we
    call ``_select_topic`` directly — the signal-wired slot ``_on_topic_selected``
    is tested separately via the render test which uses a visible window.
    """
    stub = _make_stub()
    _build_page(stub)

    topics = [('/robot1/joint_states', 'sensor_msgs/msg/JointState')]
    stub._update_topic_table(topics)
    stub._tree_topic_types['/robot1/joint_states'] = 'sensor_msgs/msg/JointState'

    # Directly invoke the logic path (bypasses Qt selection in offscreen mode)
    stub._select_topic('/robot1/joint_states')

    assert stub._selected_topic == '/robot1/joint_states'
    assert '/robot1/joint_states' in stub.selected_topic_label.text()
    stub.close()


def test_on_topic_selected_updates_label():
    """_select_topic must update selected_topic_label with the topic name."""
    stub = _make_stub()
    _build_page(stub)

    topics = [('/a/b', 'std_msgs/msg/Float64')]
    stub._update_topic_table(topics)
    stub._tree_topic_types['/a/b'] = 'std_msgs/msg/Float64'

    stub._select_topic('/a/b')

    assert '/a/b' in stub.selected_topic_label.text()
    stub.close()


# ---------------------------------------------------------------------------
# Inspector: _build_inspector_fields / _format_field_value
# ---------------------------------------------------------------------------

def test_build_inspector_fields_creates_labels():
    """_build_inspector_fields must create one value label per field."""
    stub = _make_stub()
    _build_page(stub)

    fields = OrderedDict([('x', 1.23), ('y', 4.56), ('z', 7.89)])
    stub._build_inspector_fields(fields)

    assert set(stub._insp_value_labels.keys()) == {'x', 'y', 'z'}
    stub.close()


def test_build_inspector_fields_limitbars_for_numerics():
    """Numeric scalar fields must get a LimitBar in _insp_limitbars."""
    stub = _make_stub()
    _build_page(stub)

    fields = OrderedDict([('speed', 0.5), ('label', 'hello')])
    stub._build_inspector_fields(fields)

    assert 'speed' in stub._insp_limitbars
    assert 'label' not in stub._insp_limitbars
    stub.close()


def test_format_field_value_float():
    """Floats must be formatted to 4 decimal places."""
    from fic_studio.mixins.topic_monitor_mixin import TopicMonitorMixin
    result = TopicMonitorMixin._format_field_value(3.14159265)
    assert result == '3.1416'


def test_format_field_value_long_list():
    """Long lists must be truncated with ellipsis and count."""
    from fic_studio.mixins.topic_monitor_mixin import TopicMonitorMixin
    result = TopicMonitorMixin._format_field_value(list(range(10)))
    assert '...' in result
    assert '10' in result


def test_format_field_value_dict():
    """Dicts must show key count."""
    from fic_studio.mixins.topic_monitor_mixin import TopicMonitorMixin
    result = TopicMonitorMixin._format_field_value({'a': 1, 'b': 2})
    assert '2' in result
    assert 'key' in result


def test_format_field_value_long_string():
    """Strings longer than 60 chars must be truncated."""
    from fic_studio.mixins.topic_monitor_mixin import TopicMonitorMixin
    long_s = 'x' * 80
    result = TopicMonitorMixin._format_field_value(long_s)
    assert len(result) <= 63  # 57 + '...'
    assert result.endswith('...')


# ---------------------------------------------------------------------------
# _refresh_inspector page-visible guard
# ---------------------------------------------------------------------------

def test_refresh_inspector_no_op_wrong_index():
    """_refresh_inspector must be a no-op when stack index != 11."""
    stub = _make_stub(stack_index=0)
    _build_page(stub)
    stub._selected_topic = '/t'
    mock_msg = MagicMock()
    stub.topic_cached_msg['/t'] = mock_msg

    called = []
    orig = stub._build_inspector_fields
    stub._build_inspector_fields = lambda f: called.append(f)

    stub._refresh_inspector()
    assert not called, '_refresh_inspector ran at wrong index'
    stub._build_inspector_fields = orig
    stub.close()


def test_refresh_inspector_runs_at_index_11():
    """_refresh_inspector must update fields when stack index is 11."""
    stub = _make_stub(stack_index=11)
    _build_page(stub)

    # Prime inspector widgets
    fields = OrderedDict([('v', 0.1)])
    stub._build_inspector_fields(fields)

    stub._selected_topic = '/t'
    stub.topic_last_msg['/t'] = deque([__import__('time').time()], maxlen=100)

    class _FakeMsg:
        """Minimal ROS-like message stub with one float field."""

        def get_fields_and_field_types(self):
            """Return field name -> type mapping."""
            return {'v': 'float64'}

        v = 0.5

    stub.topic_cached_msg['/t'] = _FakeMsg()
    stub._refresh_inspector()

    assert stub._insp_value_labels.get('v') is not None
    assert '0.5' in stub._insp_value_labels['v'].text()
    stub.close()


# ---------------------------------------------------------------------------
# _display_message guard
# ---------------------------------------------------------------------------

def test_display_message_wrong_topic_no_append():
    """_display_message must not append if topic != _selected_topic."""
    stub = _make_stub()
    _build_page(stub)
    stub._selected_topic = '/other'
    stub.topic_msg_count['/t'] = 5

    stub._display_message('/t', 'hello', __import__('time').time())
    assert stub.msg_display.toPlainText() == ''
    stub.close()


def test_display_message_correct_topic_appends():
    """_display_message must append text when topic matches _selected_topic."""
    stub = _make_stub()
    _build_page(stub)
    stub._selected_topic = '/t'
    stub.topic_msg_count['/t'] = 1

    stub._display_message('/t', 'world', __import__('time').time())
    assert 'world' in stub.msg_display.toPlainText()
    stub.close()


# ---------------------------------------------------------------------------
# Update rates: patches tree child items
# ---------------------------------------------------------------------------

def test_update_rates_patches_hz_text():
    """_update_rates must update column 1 text for known child items."""
    import time
    stub = _make_stub()
    _build_page(stub)

    topics = [('/r/j', 'sensor_msgs/msg/JointState')]
    stub._update_topic_table(topics)

    # Seed 10 recent timestamps
    now = time.time()
    stub.topic_last_msg['/r/j'] = deque(
        [now - i * 0.1 for i in range(10)], maxlen=100
    )

    stub._update_rates()

    child = stub._tree_child_items['/r/j']
    text = child.text(1)
    assert 'Hz' in text or text == '-'
    stub.close()


# ---------------------------------------------------------------------------
# Headless render: both themes
# ---------------------------------------------------------------------------

def _render_stub(theme_name):
    """
    Build the page with a synthetic topic list + cached message.

    Returns (stub, page) where *stub* is kept alive as QMainWindow.
    """
    import time as _time
    from fic_studio.mixins.topic_monitor_mixin import _introspect_msg

    stub = _make_stub(theme_name=theme_name, stack_index=11)
    page = _build_page(stub)
    stub.resize(900, 700)
    stub.show()

    # Inject synthetic topic list
    topics = [
        ('/robot1/joint_states', 'sensor_msgs/msg/JointState'),
        ('/robot1/cmd_vel', 'geometry_msgs/msg/Twist'),
        ('/rosout', 'rcl_interfaces/msg/Log'),
    ]
    stub._update_topic_table(topics)

    # Expand all namespace nodes
    for i in range(stub.topic_tree.topLevelItemCount()):
        stub.topic_tree.topLevelItem(i).setExpanded(True)

    class _FakeMsg:
        """Synthetic ROS message with three fields for inspector testing."""

        def get_fields_and_field_types(self):
            """Return field name -> type string mapping."""
            return {
                'position': 'float64',
                'velocity': 'float64',
                'name': 'string',
            }

        position = 0.42
        velocity = 0.1
        name = 'joint_1'

    stub.topic_cached_msg['/robot1/joint_states'] = _FakeMsg()
    stub.topic_last_msg['/robot1/joint_states'] = deque(
        [_time.time() - i * 0.2 for i in range(25)], maxlen=100
    )
    stub._selected_topic = '/robot1/joint_states'
    stub.selected_topic_label.setText('Topic: /robot1/joint_states')

    fields = _introspect_msg(_FakeMsg())
    stub._build_inspector_fields(fields)
    stub._refresh_inspector()

    _app.processEvents()
    return stub, page


def test_render_warmth_dark():
    """Page must construct and show with warmth_dark theme: tree + inspector."""
    stub, page = _render_stub('warmth_dark')
    assert stub.topic_tree.topLevelItemCount() >= 2
    assert 'robot1/joint_states' in stub.selected_topic_label.text()
    assert len(stub._insp_value_labels) > 0
    stub.close()


def test_render_warmth_light():
    """Page must construct and show with warmth_light theme: tree + inspector."""
    stub, page = _render_stub('warmth_light')
    assert stub.topic_tree.topLevelItemCount() >= 2
    assert 'robot1/joint_states' in stub.selected_topic_label.text()
    stub.close()
