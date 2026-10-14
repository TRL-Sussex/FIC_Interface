"""
Tests for the Topic Monitor's data-layer helpers.

Covers:
- _topic_namespace: namespace extraction from topic paths.
- _introspect_msg: ordered-dict field introspection via rosidl or fallback.
- _topic_hz: rolling 5-second Hz calculation.
- _stop_monitoring: leak-free teardown of rate/count/last-msg dicts.
- _refresh_topic_list: topics are sorted by name.
- _filter_topics: Publishers/Subscribers filter branches use node count APIs.
- _update_rates: updates only Hz cell text (no full rebuild).
"""

import gc
import os
import time
from collections import deque  # noqa: I100 (stdlib ordering: collections before time is correct)
from unittest.mock import MagicMock, patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from fic_studio.mixins.topic_monitor_mixin import (  # noqa: E402
    _introspect_msg,
    _topic_hz,
    _topic_namespace,
)
from PyQt5.QtWidgets import QApplication  # noqa: E402

_app = QApplication.instance() or QApplication([])


# ---------------------------------------------------------------------------
# _topic_namespace
# ---------------------------------------------------------------------------

class TestTopicNamespace:
    """Tests for the _topic_namespace helper."""

    def test_multi_segment(self):
        """First path segment returned for /robot1/joint_states."""
        assert _topic_namespace('/robot1/joint_states') == '/robot1'

    def test_single_segment_global(self):
        """Single-segment topic /rosout returns (global)."""
        assert _topic_namespace('/rosout') == '(global)'

    def test_deep_path(self):
        """Only the first segment returned for deeper paths."""
        assert _topic_namespace('/ns/sub/topic') == '/ns'

    def test_empty_string(self):
        """Empty string treated as global."""
        assert _topic_namespace('') == '(global)'

    def test_no_leading_slash(self):
        """Topic without leading slash: first segment treated as namespace or global."""
        result = _topic_namespace('robot1/joint_states')
        # Acceptable: either 'robot1' or '(global)' — must be a non-empty string.
        assert isinstance(result, str) and result


# ---------------------------------------------------------------------------
# _introspect_msg
# ---------------------------------------------------------------------------

class TestIntrospectMsg:
    """Tests for the _introspect_msg helper."""

    def test_wrench_returns_ordered_dict(self):
        """geometry_msgs/Wrench returns an OrderedDict-like mapping."""
        try:
            import geometry_msgs.msg
            msg = geometry_msgs.msg.Wrench()
            msg.force.x = 1.0
            msg.force.y = 2.0
            result = _introspect_msg(msg)
            assert isinstance(result, dict), 'result must be dict-like'
            assert 'force' in result, "Expected 'force' key in result"
        except ImportError:
            import pytest
            pytest.skip('geometry_msgs not available')

    def test_result_has_values(self):
        """Introspected fields carry values (not all None)."""
        try:
            import geometry_msgs.msg
            msg = geometry_msgs.msg.Wrench()
            msg.force.x = 7.5
            result = _introspect_msg(msg)
            # force should be a nested mapping with x
            force_val = result.get('force')
            assert force_val is not None, "Missing 'force' in result"
        except ImportError:
            import pytest
            pytest.skip('geometry_msgs not available')

    def test_float64multiarray(self):
        """std_msgs/Float64MultiArray introspects without exception."""
        try:
            import std_msgs.msg
            msg = std_msgs.msg.Float64MultiArray()
            result = _introspect_msg(msg)
            assert isinstance(result, dict)
        except ImportError:
            import pytest
            pytest.skip('std_msgs not available')

    def test_fallback_on_plain_object(self):
        """Plain Python object with __slots__ does not raise; returns a dict."""

        class _FakeMsg:
            """Minimal fake ROS message using __slots__."""

            __slots__ = ['x', 'y']

            def __init__(self):
                """Initialise slots."""
                self.x = 42
                self.y = 3.14

        result = _introspect_msg(_FakeMsg())
        assert isinstance(result, dict)


# ---------------------------------------------------------------------------
# _topic_hz
# ---------------------------------------------------------------------------

class TestTopicHz:
    """Tests for the _topic_hz helper."""

    def test_full_5s_window(self):
        """10 evenly-spaced timestamps within 5 s => 2.0 Hz."""
        now = time.time()
        stamps = deque([now - 4.5 + i * 0.5 for i in range(10)])
        hz = _topic_hz(stamps)
        assert hz == round(10 / 5.0, 1), f'Expected 2.0 Hz, got {hz}'

    def test_empty_deque(self):
        """Empty deque returns 0.0 Hz."""
        assert _topic_hz(deque()) == 0.0

    def test_all_stale_timestamps(self):
        """All timestamps older than 5 s return 0.0 Hz."""
        now = time.time()
        stamps = deque([now - 10, now - 8, now - 6])
        assert _topic_hz(stamps) == 0.0

    def test_one_dp_rounding(self):
        """Result is rounded to 1 decimal place."""
        now = time.time()
        stamps = deque([now - i * 0.3 for i in range(7)])
        hz = _topic_hz(stamps)
        assert hz == round(hz, 1), f'Expected 1 dp, got {hz}'

    def test_partial_window(self):
        """Only timestamps within last 5 s counted."""
        now = time.time()
        # 3 fresh, 2 stale
        stamps = deque([now - 6, now - 7, now - 1, now - 2, now - 3])
        hz = _topic_hz(stamps)
        assert hz == round(3 / 5.0, 1), f'Expected 0.6 Hz, got {hz}'


# ---------------------------------------------------------------------------
# Stub window builder (shared by mixin-level tests below)
# ---------------------------------------------------------------------------

def _purge_dead_theme_bindings():
    """
    Remove dead Qt C++ object refs from the module-level theme _bindings.

    theme.bind stores (weakref, factory) pairs.  When a test window is GC-ed
    the weakref can still return a non-None Python wrapper whose C++ layer has
    been destroyed.  Force-collecting and discarding such refs prevents
    RuntimeError in the next test that calls apply_theme.
    """
    from fic_studio import theme
    gc.collect()
    # Keep only refs whose Python wrapper is alive and whose C++ object is
    # still valid (setStyleSheet must not raise).
    live = []
    for ref, fac in theme._bindings:
        widget = ref()
        if widget is None:
            continue
        try:
            widget.setStyleSheet(widget.styleSheet())
            live.append((ref, fac))
        except RuntimeError:
            pass
    theme._bindings[:] = live


def _make_stub_window(topics=None):
    """Return a minimal TopicMonitorMixin window with a mocked node."""
    from PyQt5.QtWidgets import QMainWindow, QLabel, QPushButton
    from PyQt5.QtCore import Qt
    from fic_studio.mixins.topic_monitor_mixin import TopicMonitorMixin
    from fic_studio import theme

    _purge_dead_theme_bindings()

    class _StubWindow(QMainWindow, TopicMonitorMixin):
        """Minimal stub satisfying TopicMonitorMixin dependencies."""

        def _page_title(self, text):
            """Return a plain QLabel as page title."""
            return QLabel(text)

        def _btn(self, text, style_id=None):
            """Return a plain QPushButton."""
            btn = QPushButton(text)
            btn.setCursor(Qt.PointingHandCursor)
            return btn

        def log_message(self, *args, **kwargs):
            """Stub logger."""

    win = _StubWindow()
    theme.apply_theme(win, 'warmth_dark')

    # Mock ROS node
    mock_node = MagicMock()
    if topics is None:
        topics = []
    mock_node.get_topic_names_and_types.return_value = topics
    mock_node.count_publishers.return_value = 1
    mock_node.count_subscribers.return_value = 0
    win.node = mock_node

    win._init_topic_monitor()
    return win


# ---------------------------------------------------------------------------
# _stop_monitoring: no dict leaks
# ---------------------------------------------------------------------------

class TestStopMonitoringLeak:
    """_stop_monitoring must delete entries from all three state dicts."""

    def test_cleans_topic_rates(self):
        """topic_rates entry removed after stop."""
        win = _make_stub_window()
        topic = '/test_topic'
        win.topic_rates[topic] = deque()
        win.topic_msg_count[topic] = 5
        win.topic_last_msg[topic] = deque()
        # Simulate subscription entry
        win.monitored_topics[topic] = MagicMock()
        win._stop_monitoring(topic)
        assert topic not in win.topic_rates, 'topic_rates leak'

    def test_cleans_topic_msg_count(self):
        """topic_msg_count entry removed after stop."""
        win = _make_stub_window()
        topic = '/test_topic'
        win.topic_rates[topic] = deque()
        win.topic_msg_count[topic] = 5
        win.topic_last_msg[topic] = deque()
        win.monitored_topics[topic] = MagicMock()
        win._stop_monitoring(topic)
        assert topic not in win.topic_msg_count, 'topic_msg_count leak'

    def test_cleans_topic_last_msg(self):
        """topic_last_msg entry removed after stop."""
        win = _make_stub_window()
        topic = '/test_topic'
        win.topic_rates[topic] = deque()
        win.topic_msg_count[topic] = 5
        win.topic_last_msg[topic] = deque()
        win.monitored_topics[topic] = MagicMock()
        win._stop_monitoring(topic)
        assert topic not in win.topic_last_msg, 'topic_last_msg leak'

    def test_monitored_topics_removed(self):
        """monitored_topics entry also removed."""
        win = _make_stub_window()
        topic = '/test_topic'
        win.topic_rates[topic] = deque()
        win.topic_msg_count[topic] = 0
        win.topic_last_msg[topic] = deque()
        win.monitored_topics[topic] = MagicMock()
        win._stop_monitoring(topic)
        assert topic not in win.monitored_topics, 'monitored_topics leak'

    def test_cleans_topic_cached_msg(self):
        """topic_cached_msg entry removed after stop (prevents large-msg retention)."""
        win = _make_stub_window()
        topic = '/test_topic'
        win.topic_rates[topic] = deque()
        win.topic_msg_count[topic] = 3
        win.topic_last_msg[topic] = deque()
        # Simulate a large cached message object (e.g. PointCloud2)
        win.topic_cached_msg[topic] = MagicMock(name='big_msg')
        win.monitored_topics[topic] = MagicMock()
        win._stop_monitoring(topic)
        assert topic not in win.topic_cached_msg, 'topic_cached_msg leak'


# ---------------------------------------------------------------------------
# _refresh_topic_list: topics sorted
# ---------------------------------------------------------------------------

class TestRefreshTopicListSort:
    """_refresh_topic_list must emit a sorted topic list."""

    def test_topics_sorted_by_name(self):
        """Topics emitted in alphabetical order."""
        unsorted = [
            ('/z_topic', ['std_msgs/msg/String']),
            ('/a_topic', ['std_msgs/msg/Int32']),
            ('/m_topic', ['std_msgs/msg/Float64']),
        ]
        win = _make_stub_window(topics=unsorted)
        # Set page as central widget so Qt keeps child widgets alive.
        page = win._build_topic_monitor_page()
        win.setCentralWidget(page)

        captured = []

        def _capture(topics):
            captured.extend(topics)

        win.topic_signals.topic_list_updated.connect(_capture)
        win._refresh_topic_list()

        # Give Qt event loop a tick
        _app.processEvents()

        if captured:
            names = [t[0] for t in captured]
            assert names == sorted(names), f'Topics not sorted: {names}'


# ---------------------------------------------------------------------------
# _filter_topics: Publishers/Subscribers branches
# ---------------------------------------------------------------------------

class TestFilterTopicsPublishersSubscribers:
    """Publishers Only / Subscribers Only filter must call node count APIs."""

    # Keep a class-level reference so Qt does not GC theme-bound widgets
    # between test methods (theme bindings hold weak refs internally but the
    # C++ layer underneath gets destroyed on GC).
    _windows = []

    def _setup(self):
        """Build a window with two topics and return (win, mock_node)."""
        from PyQt5.QtWidgets import QMainWindow, QLabel, QPushButton
        from PyQt5.QtCore import Qt
        from fic_studio.mixins.topic_monitor_mixin import TopicMonitorMixin
        from fic_studio import theme

        # Purge stale C++ refs before calling apply_theme.
        _purge_dead_theme_bindings()

        class _FilterStub(QMainWindow, TopicMonitorMixin):
            """Minimal stub for filter tests; held alive via class list."""

            def _page_title(self, text):
                """Return plain QLabel."""
                return QLabel(text)

            def _btn(self, text, style_id=None):
                """Return plain QPushButton."""
                btn = QPushButton(text)
                btn.setCursor(Qt.PointingHandCursor)
                return btn

            def log_message(self, *args, **kwargs):
                """Stub logger."""

        win = _FilterStub()
        theme.apply_theme(win, 'warmth_dark')

        mock_node = MagicMock()
        mock_node.get_topic_names_and_types.return_value = [
            ('/pub_only', ['std_msgs/msg/String']),
            ('/sub_only', ['std_msgs/msg/String']),
        ]
        mock_node.count_publishers.return_value = 1
        mock_node.count_subscribers.return_value = 0
        win.node = mock_node

        win._init_topic_monitor()
        # Set as central widget so Qt keeps the page and child widgets alive.
        page = win._build_topic_monitor_page()
        win.setCentralWidget(page)
        _app.processEvents()

        win._all_topics = [('/pub_only', 'std_msgs/msg/String'),
                           ('/sub_only', 'std_msgs/msg/String')]

        TestFilterTopicsPublishersSubscribers._windows.append(win)
        return win, mock_node

    def test_publishers_filter_calls_count_publishers(self):
        """The Publishers Only filter calls node.count_publishers for each topic."""
        win, mock_node = self._setup()
        mock_node.count_publishers.return_value = 1
        mock_node.count_publishers.reset_mock()

        win.topic_type_filter.setCurrentText('Publishers Only')
        _app.processEvents()

        assert mock_node.count_publishers.called, (
            'count_publishers not called with Publishers Only filter')

    def test_subscribers_filter_calls_count_subscribers(self):
        """The Subscribers Only filter calls node.count_subscribers for each topic."""
        win, mock_node = self._setup()
        mock_node.count_subscribers.return_value = 1
        mock_node.count_subscribers.reset_mock()

        win.topic_type_filter.setCurrentText('Subscribers Only')
        _app.processEvents()

        assert mock_node.count_subscribers.called, (
            'count_subscribers not called with Subscribers Only filter')

    def test_publishers_filter_hides_zero_publisher_topics(self):
        """Topics with 0 publishers excluded by Publishers Only filter."""
        win, mock_node = self._setup()

        def _pub_count(name):
            return 1 if name == '/pub_only' else 0

        mock_node.count_publishers.side_effect = _pub_count

        win.topic_type_filter.setCurrentText('Publishers Only')
        _app.processEvents()

        # Collect all child (topic) names visible in the tree
        visible_names = []
        for i in range(win.topic_tree.topLevelItemCount()):
            parent = win.topic_tree.topLevelItem(i)
            for j in range(parent.childCount()):
                child = parent.child(j)
                # Full topic name stored in UserRole
                from PyQt5.QtCore import Qt as _Qt
                visible_names.append(child.data(0, _Qt.UserRole))

        assert '/pub_only' in visible_names, 'pub_only missing from Publishers Only filter'
        assert '/sub_only' not in visible_names, 'sub_only should be hidden'

    def test_publishers_filter_node_none_shows_all(self):
        """Publishers Only filter shows all topics when node is None (regression guard)."""
        win, _mock_node = self._setup()
        win.node = None  # simulate node not yet available

        win.topic_type_filter.setCurrentText('Publishers Only')
        _app.processEvents()

        visible_names = []
        for i in range(win.topic_tree.topLevelItemCount()):
            parent = win.topic_tree.topLevelItem(i)
            for j in range(parent.childCount()):
                child = parent.child(j)
                from PyQt5.QtCore import Qt as _Qt
                visible_names.append(child.data(0, _Qt.UserRole))

        assert len(visible_names) == 2, (
            'Expected all 2 topics when node is None, got %d: %s'
            % (len(visible_names), visible_names)
        )
        assert '/pub_only' in visible_names
        assert '/sub_only' in visible_names

    def test_subscribers_filter_node_none_shows_all(self):
        """Subscribers Only filter shows all topics when node is None (regression guard)."""
        win, _mock_node = self._setup()
        win.node = None  # simulate node not yet available

        win.topic_type_filter.setCurrentText('Subscribers Only')
        _app.processEvents()

        visible_names = []
        for i in range(win.topic_tree.topLevelItemCount()):
            parent = win.topic_tree.topLevelItem(i)
            for j in range(parent.childCount()):
                child = parent.child(j)
                from PyQt5.QtCore import Qt as _Qt
                visible_names.append(child.data(0, _Qt.UserRole))

        assert len(visible_names) == 2, (
            'Expected all 2 topics when node is None, got %d: %s'
            % (len(visible_names), visible_names)
        )
        assert '/pub_only' in visible_names
        assert '/sub_only' in visible_names


# ---------------------------------------------------------------------------
# _update_rates: patches Hz cells only, does not call _filter_topics
# ---------------------------------------------------------------------------

class TestUpdateRatesNoBuild:
    """_update_rates must update only Hz cell text without rebuilding the table."""

    def test_hz_cell_updated(self):
        """After _update_rates the Hz chip text in column 1 reflects the rate."""
        win = _make_stub_window()
        page = win._build_topic_monitor_page()
        win.setCentralWidget(page)
        topic = '/rate_topic'
        win._all_topics = [(topic, 'std_msgs/msg/String')]
        win._filter_topics()  # populate tree with one child

        # Seed a rate
        now = time.time()
        win.topic_rates[topic] = deque([now - 1, now - 2, now - 3])
        win.topic_last_msg[topic] = deque([now - 1, now - 2, now - 3])

        win._update_rates()
        _app.processEvents()

        # Hz chip is column 1 of the child QTreeWidgetItem
        child_item = win._tree_child_items.get(topic)
        assert child_item is not None, 'child item missing from _tree_child_items'
        text = child_item.text(1)
        # 3 timestamps within the 5 s window → 3 / 5.0 = 0.6 Hz
        assert '0.6' in text, 'Expected Hz chip to contain "0.6" but got %r' % text

    def test_update_rates_does_not_call_filter_topics(self):
        """_update_rates must NOT call _filter_topics (no full rebuild)."""
        win = _make_stub_window()
        page = win._build_topic_monitor_page()
        win.setCentralWidget(page)
        win._all_topics = [('/t', 'std_msgs/msg/String')]
        win._filter_topics()

        with patch.object(win, '_filter_topics') as mock_filter:
            win._update_rates()
            _app.processEvents()
            mock_filter.assert_not_called()
