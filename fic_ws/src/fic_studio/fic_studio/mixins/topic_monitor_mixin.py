# mixins/topic_monitor_mixin.py — ROS2 Topic Monitor for live topic inspection

import re
import time
from collections import OrderedDict, deque
from datetime import datetime
from functools import partial

from fic_studio import theme


# ---------------------------------------------------------------------------
# Module-level helpers (pure functions, no Qt / ROS deps)
# ---------------------------------------------------------------------------

def _topic_namespace(name: str) -> str:
    """Return the first path segment of *name* as the namespace.

    Examples::

        _topic_namespace('/robot1/joint_states') -> '/robot1'
        _topic_namespace('/rosout')              -> '(global)'
        _topic_namespace('')                     -> '(global)'

    A topic is considered *global* when its path has only one segment
    (no second ``/``) or when *name* is empty.
    """
    if not name:
        return '(global)'
    stripped = name.lstrip('/')
    parts = stripped.split('/', 1)
    if len(parts) >= 2 and parts[1]:
        # Path-style namespace, e.g. /robot1/joint_states -> /robot1
        prefix = '/' if name.startswith('/') else ''
        return prefix + parts[0]
    # Flat FIC naming: detect an embedded robotN token
    # (robot1_target_joint_angles or command_..._robot1 -> robot1).
    match = re.search(r'robot\d+', stripped)
    if match:
        return match.group(0)
    return '(global)'


def _introspect_msg(msg) -> OrderedDict:
    """Return an OrderedDict of field->value for *msg* (one nested level).

    Preference order:

    1. ``rosidl_runtime_py.convert.message_to_ordereddict`` — the canonical
       ROS 2 serialiser that handles arrays, sequences, and nested messages.
    2. Walk ``msg.get_fields_and_field_types()`` for field names, then
       ``getattr`` each field.
    3. Walk ``msg.__slots__`` (strips the leading underscore ROS 2 adds).

    All paths return an :class:`collections.OrderedDict` and must not raise.
    """
    # Strategy 1: rosidl_runtime_py (preferred)
    try:
        from rosidl_runtime_py.convert import message_to_ordereddict as _m2od
        return _m2od(msg)
    except Exception:
        pass

    result = OrderedDict()

    # Strategy 2: get_fields_and_field_types
    try:
        fields = msg.get_fields_and_field_types()
        for field_name in fields:
            try:
                result[field_name] = getattr(msg, field_name)
            except Exception:
                result[field_name] = None
        return result
    except Exception:
        pass

    # Strategy 3: __slots__ fallback
    try:
        for slot in getattr(msg, '__slots__', []):
            key = slot.lstrip('_')  # ROS 2 uses _x for field x
            try:
                result[key] = getattr(msg, slot)
            except Exception:
                result[key] = None
        return result
    except Exception:
        pass

    return result


def _topic_hz(timestamps: deque) -> float:
    """Return the message rate in Hz over the last 5 seconds, to 1 d.p.

    Counts how many entries in *timestamps* (Unix epoch floats) fall within
    the 5-second window ending *now*, then divides by 5.0 and rounds to one
    decimal place.
    """
    if not timestamps:
        return 0.0
    now = time.time()
    window = 5.0
    count = sum(1 for t in timestamps if now - t < window)
    return round(count / window, 1)


from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QHeaderView, QSplitter, QTextEdit, QComboBox, QSpinBox, QCheckBox,
    QScrollArea, QLineEdit, QAbstractItemView,
    QListView, QTreeWidget, QTreeWidgetItem,
)


class TopicSignals(QObject):
    """Signals for thread-safe topic updates."""
    topic_list_updated = pyqtSignal(list)
    message_received = pyqtSignal(str, str, float)  # topic, data, timestamp


class TopicMonitorMixin:
    """Mixin providing ROS2 topic monitoring functionality."""
    
    def _init_topic_monitor(self):
        """Initialize topic monitor state."""
        self.topic_signals = TopicSignals()
        self.topic_signals.topic_list_updated.connect(self._update_topic_table)
        self.topic_signals.message_received.connect(self._display_message)

        self.monitored_topics = {}  # topic_name -> subscription
        self.topic_rates = {}  # topic_name -> deque of timestamps for rate calc
        self.topic_last_msg = {}  # topic_name -> deque of timestamps
        self.topic_msg_count = {}  # topic_name -> message count
        self.topic_cached_msg = {}  # topic_name -> latest raw msg object

        # Currently selected topic name in the tree (None = nothing selected)
        self._selected_topic = None

        # Timer for rate calculation (1 Hz)
        self.rate_timer = QTimer()
        self.rate_timer.timeout.connect(self._update_rates)
        self.rate_timer.start(1000)

        # Timer for ~15 Hz inspector / sparkline refresh (page-visible guarded)
        self._insp_refresh_timer = QTimer()
        self._insp_refresh_timer.timeout.connect(self._refresh_inspector)
        self._insp_refresh_timer.start(67)  # ~15 Hz

        # Timer for topic list auto-refresh
        self.topic_list_timer = QTimer()
        self.topic_list_timer.timeout.connect(self._refresh_topic_list)
    
    def _build_topic_monitor_page(self) -> QWidget:
        """Build the ROS2 Topic Monitor page (grouped tree + inspector).

        Layout (top-to-bottom)::

            filter_row  (QLineEdit + QComboBox + Refresh + Auto-refresh)
            QSplitter(Horizontal)
              left  — QTreeWidget grouped by namespace
              right — Inspector card (fields + LimitBar + TrajectoryPlot + raw stream)
            stats_bar

        The inspector is populated when a child topic item is selected.
        A 15 Hz QTimer (``_insp_refresh_timer``) updates field values and the
        sparkline from ``topic_cached_msg`` while the page is the visible index.
        Previously monitored topics are restored from ``settings['topic_monitor']``
        and re-subscribed automatically.
        """
        from fic_studio.components.dash_widgets import LimitBar, TrajectoryPlot
        from fic_studio.mixins.dashboard_mixin import _card

        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(22, 20, 22, 20)
        layout.setSpacing(10)

        # ── Title row ────────────────────────────────────────────────────────
        title_row = QHBoxLayout()
        title_row.addWidget(self._page_title('ROS2 Topic Monitor'))
        title_row.addStretch()
        layout.addLayout(title_row)

        # ── Filter row ───────────────────────────────────────────────────────
        filter_row = QHBoxLayout()
        filter_row.setSpacing(8)

        filter_row.addWidget(QLabel('Filter:'))
        self.topic_filter = QLineEdit()
        self.topic_filter.setPlaceholderText('Type to filter topics...')
        self.topic_filter.textChanged.connect(self._filter_topics)
        filter_row.addWidget(self.topic_filter, 1)

        filter_row.addWidget(QLabel('Role:'))
        self.topic_type_filter = QComboBox()
        self.topic_type_filter.addItems([
            'All Topics', 'Publishers Only', 'Subscribers Only',
            'Services', 'Actions',
        ])
        self.topic_type_filter.currentTextChanged.connect(self._filter_topics)
        filter_row.addWidget(self.topic_type_filter)

        refresh_btn = self._btn('↻ Refresh', 'BtnPrimary')
        refresh_btn.clicked.connect(self._refresh_topic_list)
        filter_row.addWidget(refresh_btn)

        self.auto_refresh_check = QCheckBox('Auto-refresh')
        self.auto_refresh_check.setChecked(False)
        self.auto_refresh_check.toggled.connect(self._toggle_auto_refresh)
        filter_row.addWidget(self.auto_refresh_check)

        layout.addLayout(filter_row)

        # ── Main horizontal splitter ─────────────────────────────────────────
        h_split = QSplitter(Qt.Horizontal)

        # ── LEFT: namespace-grouped QTreeWidget ──────────────────────────────
        self.topic_tree = QTreeWidget()
        self.topic_tree.setHeaderLabels(['Topic / Namespace', 'Hz', ''])
        self.topic_tree.header().setSectionResizeMode(0, QHeaderView.Stretch)
        self.topic_tree.header().setSectionResizeMode(1, QHeaderView.Fixed)
        self.topic_tree.header().setSectionResizeMode(2, QHeaderView.Fixed)
        self.topic_tree.setColumnWidth(1, 70)
        self.topic_tree.setColumnWidth(2, 80)
        self.topic_tree.setAlternatingRowColors(True)
        self.topic_tree.setSelectionMode(QAbstractItemView.SingleSelection)
        self.topic_tree.itemSelectionChanged.connect(self._on_topic_selected)
        theme.bind(
            self.topic_tree,
            lambda t: (
                'QTreeWidget {'
                ' background: %s;'
                ' alternate-background-color: %s;'
                ' border: 1px solid %s; border-radius: 6px;'
                '}'
                'QTreeWidget::item:selected {'
                ' background: %s; color: %s;'
                '}'
            ) % (
                t['bg_raised'], t['bg_base'],
                t['border_subtle'], t['accent_blue'], t['text_primary'],
            ),
        )

        # Keep a mapping: topic_name -> QTreeWidgetItem (child)
        self._tree_child_items = {}  # topic_name -> QTreeWidgetItem
        # Mapping: topic_name -> msg_type (needed for subscribe/restore)
        self._tree_topic_types = {}  # topic_name -> msg_type str

        h_split.addWidget(self.topic_tree)

        # ── RIGHT: Inspector card ────────────────────────────────────────────
        insp_outer, insp_layout = _card('INSPECTOR', 'accent_blue')
        insp_outer.setMinimumWidth(300)

        # Selected topic name
        self.selected_topic_label = QLabel('Select a topic to inspect')
        theme.bind(
            self.selected_topic_label,
            lambda t: (
                'color: %s; font-weight: 600; font-size: 12px;'
                ' background: transparent;'
            ) % t['accent_blue'],
        )
        insp_layout.addWidget(self.selected_topic_label)

        # Field scroll area (label + value rows + optional LimitBar)
        self._insp_fields_widget = QWidget()
        self._insp_fields_layout = QVBoxLayout(self._insp_fields_widget)
        self._insp_fields_layout.setContentsMargins(0, 0, 0, 0)
        self._insp_fields_layout.setSpacing(4)

        fields_scroll = QScrollArea()
        fields_scroll.setWidgetResizable(True)
        fields_scroll.setWidget(self._insp_fields_widget)
        fields_scroll.setMinimumHeight(100)
        fields_scroll.setMaximumHeight(180)
        theme.bind(
            fields_scroll,
            lambda t: (
                'QScrollArea {'
                ' background: %s; border: none;'
                '}'
            ) % t['bg_raised'],
        )
        insp_layout.addWidget(fields_scroll)

        # Sparkline (arrival-time fractions — how recently a message arrived)
        spark_label = QLabel('ARRIVAL RATE')
        theme.bind(
            spark_label,
            lambda t: (
                'color: %s; font-size: 9px; font-weight: 700;'
                ' background: transparent;'
            ) % t['text_disabled'],
        )
        insp_layout.addWidget(spark_label)
        self._insp_sparkline = TrajectoryPlot()
        self._insp_sparkline.setMinimumHeight(70)
        self._insp_sparkline.setMaximumHeight(100)
        insp_layout.addWidget(self._insp_sparkline)

        # Raw stream header
        raw_hdr = QHBoxLayout()
        raw_title = QLabel('RAW STREAM')
        theme.bind(
            raw_title,
            lambda t: (
                'color: %s; font-size: 9px; font-weight: 700;'
                ' background: transparent;'
            ) % t['text_disabled'],
        )
        raw_hdr.addWidget(raw_title)
        raw_hdr.addStretch()

        msg_header_row = QHBoxLayout()
        msg_header_row.addWidget(QLabel('Buffer:'))
        self.msg_buffer_spin = QSpinBox()
        self.msg_buffer_spin.setRange(1, 1000)
        self.msg_buffer_spin.setValue(50)
        self.msg_buffer_spin.setSuffix(' msgs')
        msg_header_row.addWidget(self.msg_buffer_spin)

        clear_btn = self._btn('Clear')
        clear_btn.clicked.connect(self._clear_messages)
        msg_header_row.addWidget(clear_btn)

        self.pause_btn = self._btn('‖ Pause')
        self.pause_btn.setCheckable(True)
        self.pause_btn.toggled.connect(self._toggle_pause)
        msg_header_row.addWidget(self.pause_btn)

        insp_layout.addLayout(raw_hdr)
        insp_layout.addLayout(msg_header_row)

        self.msg_display = QTextEdit()
        self.msg_display.setReadOnly(True)
        theme.bind(
            self.msg_display,
            lambda t: (
                'QTextEdit {'
                " font-family: 'Consolas', 'Monaco', monospace;"
                ' font-size: 11px;'
                ' background-color: %s;'
                ' border: 1px solid %s;'
                ' border-radius: 6px;'
                '}'
            ) % (t['bg_base'], t['border_subtle']),
        )
        insp_layout.addWidget(self.msg_display, 1)

        h_split.addWidget(insp_outer)
        h_split.setSizes([400, 350])

        layout.addWidget(h_split, 1)

        # ── Stats bar ────────────────────────────────────────────────────────
        stats_row = QHBoxLayout()
        self.topic_count_label = QLabel('Topics: 0')
        theme.bind(
            self.topic_count_label,
            lambda t: 'color: %s; font-size: 11px; background: transparent;'
            % t['text_secondary'],
        )
        stats_row.addWidget(self.topic_count_label)

        _sep1 = QLabel('•')
        theme.bind(
            _sep1,
            lambda t: 'color: %s; background: transparent;' % t['border_subtle'],
        )
        stats_row.addWidget(_sep1)

        self.monitored_count_label = QLabel('Monitoring: 0')
        theme.bind(
            self.monitored_count_label,
            lambda t: 'color: %s; font-size: 11px; background: transparent;'
            % t['accent_teal'],
        )
        stats_row.addWidget(self.monitored_count_label)

        _sep2 = QLabel('•')
        theme.bind(
            _sep2,
            lambda t: 'color: %s; background: transparent;' % t['border_subtle'],
        )
        stats_row.addWidget(_sep2)

        self.total_msg_label = QLabel('Total Messages: 0')
        theme.bind(
            self.total_msg_label,
            lambda t: 'color: %s; font-size: 11px; background: transparent;'
            % t['text_secondary'],
        )
        stats_row.addWidget(self.total_msg_label)
        stats_row.addStretch()
        layout.addLayout(stats_row)

        # ── Restore previously monitored topics from settings ────────────────
        self._restore_monitored_topics()

        # Trigger initial topic discovery
        self._refresh_topic_list()

        # Restyle transient tree-row buttons on theme switch (they use direct
        # setStyleSheet, not theme.bind, to avoid accumulating dead bindings).
        theme.signals.theme_changed.connect(self._filter_topics)

        return page

    def _restore_monitored_topics(self):
        """Re-subscribe to topics persisted in ``settings['topic_monitor']``.

        Called once from ``_build_topic_monitor_page``.  If the node is not
        yet available the restore step is silently skipped; topics will be
        persisted again next time ``_save_monitored_topics`` is called.
        """
        if not hasattr(self, 'settings'):
            return
        tmon = self.settings.get('topic_monitor', {})
        saved = tmon.get('monitored', {})  # {topic_name: msg_type}
        if not saved or not hasattr(self, 'node') or self.node is None:
            return
        for topic_name, msg_type in saved.items():
            if topic_name not in self.monitored_topics:
                self._start_monitoring(topic_name, msg_type)

    def _save_monitored_topics(self):
        """Persist current ``monitored_topics`` keys+types to ``settings``.

        Writes into ``settings['topic_monitor']['monitored']`` as a plain
        ``{topic_name: msg_type}`` dict so it survives page rebuilds.
        """
        if not hasattr(self, 'settings'):
            return
        if 'topic_monitor' not in self.settings:
            self.settings['topic_monitor'] = {}
        self.settings['topic_monitor']['monitored'] = {
            t: self._tree_topic_types.get(t, 'unknown')
            for t in self.monitored_topics
        }

    def _build_inspector_fields(self, fields: OrderedDict):
        """Rebuild the inspector field rows from *fields* (OrderedDict).

        Each row is a QHBoxLayout with a label (field name) and a value label.
        Numeric scalar fields also get a ``LimitBar`` micro-bar.
        All existing child widgets in ``_insp_fields_layout`` are removed first.
        """
        from fic_studio.components.dash_widgets import LimitBar

        # Clear existing rows
        while self._insp_fields_layout.count():
            item = self._insp_fields_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        self._insp_value_labels = {}  # field_name -> QLabel
        self._insp_limitbars = {}  # field_name -> LimitBar

        for field_name, value in fields.items():
            row_w = QWidget()
            row_l = QHBoxLayout(row_w)
            row_l.setContentsMargins(0, 0, 0, 0)
            row_l.setSpacing(6)

            lbl = QLabel(str(field_name) + ':')
            theme.bind(
                lbl,
                lambda t: (
                    'color: %s; font-size: 11px; background: transparent;'
                ) % t['text_secondary'],
            )
            lbl.setMinimumWidth(90)
            row_l.addWidget(lbl)

            val_str = self._format_field_value(value)
            val_lbl = QLabel(val_str)
            theme.bind(
                val_lbl,
                lambda t: (
                    'color: %s; font-size: 11px;'
                    " font-family: 'Consolas', monospace; background: transparent;"
                ) % t['text_primary'],
            )
            row_l.addWidget(val_lbl, 1)
            self._insp_value_labels[field_name] = val_lbl

            # LimitBar for numeric scalars
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                bar = LimitBar()
                bar.setFixedHeight(5)
                bar.setMinimumWidth(60)
                try:
                    frac = max(0.0, min(1.0, abs(float(value))))
                except Exception:
                    frac = 0.0
                bar.set_frac(frac)
                row_l.addWidget(bar)
                self._insp_limitbars[field_name] = bar

            self._insp_fields_layout.addWidget(row_w)

        self._insp_fields_layout.addStretch()

    def _update_inspector_values(self, fields: OrderedDict):
        """Patch only the value labels and LimitBars (no widget rebuild).

        Called at ~15 Hz by ``_refresh_inspector`` to update the inspector
        without destroying and recreating widgets.
        """
        for field_name, value in fields.items():
            lbl = self._insp_value_labels.get(field_name)
            if lbl is not None:
                lbl.setText(self._format_field_value(value))
            bar = self._insp_limitbars.get(field_name)
            if bar is not None:
                try:
                    frac = max(0.0, min(1.0, abs(float(value))))
                except Exception:
                    frac = 0.0
                bar.set_frac(frac)

    @staticmethod
    def _format_field_value(value) -> str:
        """Return a compact string representation of a field value.

        Truncates nested structures and long arrays to keep the inspector
        readable at 11 px font size.
        """
        if isinstance(value, float):
            return '%.4f' % value
        if isinstance(value, (list, tuple)):
            if len(value) <= 4:
                return str(value)
            return '[%s, ... (%d)]' % (
                ', '.join(str(v) for v in value[:3]), len(value),
            )
        if isinstance(value, dict):
            return '{...(%d keys)}' % len(value)
        s = str(value)
        if len(s) > 60:
            return s[:57] + '...'
        return s

    def _refresh_inspector(self):
        """Refresh inspector field values and sparkline at ~15 Hz.

        Page-visible guard: only runs when the main stack is at index 11.
        Updates ``_insp_value_labels``, ``_insp_limitbars``, and
        ``_insp_sparkline`` from the cached latest message for the selected
        topic.  If fields have changed since the last render (different keys),
        rebuilds the field rows via ``_build_inspector_fields``.
        """
        # Page-visible guard
        if not hasattr(self, 'main_stack'):
            return
        if not self.is_page_visible('topic_monitor'):
            return

        topic = self._selected_topic
        if not topic:
            return

        msg = self.topic_cached_msg.get(topic)
        if msg is None:
            return

        fields = _introspect_msg(msg)

        # Detect whether field set changed (rebuild) vs same fields (patch)
        existing_keys = set(getattr(self, '_insp_value_labels', {}).keys())
        if set(fields.keys()) != existing_keys:
            self._build_inspector_fields(fields)
        else:
            self._update_inspector_values(fields)

        # Update sparkline: push a normalised 'recency' value (0=old, 1=fresh)
        ts_deque = self.topic_last_msg.get(topic)
        if ts_deque:
            now = time.time()
            window = 5.0
            pts = [
                max(0.0, min(1.0, 1.0 - (now - t) / window))
                for t in ts_deque
            ]
            self._insp_sparkline.set_points(pts)
    
    def _refresh_topic_list(self):
        """Refresh the list of ROS2 topics, emitting them sorted by name."""
        if not hasattr(self, 'node') or self.node is None:
            return

        try:
            topic_names_and_types = self.node.get_topic_names_and_types()

            topics = []
            for name, types in topic_names_and_types:
                type_str = types[0] if types else 'unknown'
                topics.append((name, type_str))

            # Sort alphabetically by topic name
            topics = sorted(topics, key=lambda t: t[0])

            self.topic_signals.topic_list_updated.emit(topics)

        except Exception as e:
            self.log_message('ERROR', 'Failed to get topic list: %s' % e, 'Monitor')

    def _update_topic_table(self, topics: list):
        """Update the namespace-grouped topic tree with *topics*.

        Stores the raw list in ``_all_topics`` for filter rebuilds and
        caches each topic's msg_type in ``_tree_topic_types`` for restore.
        """
        self._all_topics = topics
        # Cache msg_types for all known topics
        for name, msg_type in topics:
            self._tree_topic_types[name] = msg_type
        self._filter_topics()
        self.topic_count_label.setText('Topics: %d' % len(topics))

    def _filter_topics(self):
        """Rebuild the grouped tree from ``_all_topics`` applying active filters.

        Filter branches:

        - Text search (case-insensitive substring on topic name).
        - Publishers Only: ``node.count_publishers(name) > 0``.
        - Subscribers Only: ``node.count_subscribers(name) > 0``.
        - Services: msg_type contains ``/srv/``.
        - Actions: msg_type contains ``/action/``.

        Groups filtered topics by ``_topic_namespace`` and builds one parent
        QTreeWidgetItem per namespace (showing child count) and one child per
        topic.  A Monitor toggle QPushButton is placed via ``setItemWidget``
        in column 2.  The ``_tree_child_items`` dict is rebuilt.
        """
        if not hasattr(self, '_all_topics') or not hasattr(self, 'topic_tree'):
            return

        filter_text = self.topic_filter.text().lower()
        type_filter = self.topic_type_filter.currentText()

        filtered = []
        for name, msg_type in self._all_topics:
            if filter_text and filter_text not in name.lower():
                continue
            if type_filter == 'Publishers Only':
                if not hasattr(self, 'node') or self.node is None:
                    filtered.append((name, msg_type))
                    continue
                try:
                    if self.node.count_publishers(name) <= 0:
                        continue
                except Exception:
                    continue
            elif type_filter == 'Subscribers Only':
                if not hasattr(self, 'node') or self.node is None:
                    filtered.append((name, msg_type))
                    continue
                try:
                    if self.node.count_subscribers(name) <= 0:
                        continue
                except Exception:
                    continue
            elif type_filter == 'Services' and '/srv/' not in msg_type:
                continue
            elif type_filter == 'Actions' and '/action/' not in msg_type:
                continue
            filtered.append((name, msg_type))

        # Group by namespace
        groups = OrderedDict()
        for name, msg_type in filtered:
            ns = _topic_namespace(name)
            groups.setdefault(ns, []).append((name, msg_type))

        # Remember which namespaces were expanded
        expanded_ns = set()
        for i in range(self.topic_tree.topLevelItemCount()):
            p = self.topic_tree.topLevelItem(i)
            if p.isExpanded():
                expanded_ns.add(p.data(0, Qt.UserRole))

        from PyQt5.QtGui import QColor as _QColor

        self.topic_tree.clear()
        self._tree_child_items = {}

        for ns, topic_list in groups.items():
            parent_item = QTreeWidgetItem(self.topic_tree)
            ns_text = '%s  (%d)' % (ns, len(topic_list))
            parent_item.setText(0, ns_text)
            parent_item.setData(0, Qt.UserRole, ns)
            parent_item.setFlags(parent_item.flags() & ~Qt.ItemIsSelectable)
            theme_now = theme.current()
            parent_item.setForeground(0, _QColor(theme_now['text_secondary']))
            # Expand if it was expanded before or is the only namespace
            if ns in expanded_ns or len(groups) == 1:
                parent_item.setExpanded(True)

            for name, msg_type in topic_list:
                child_item = QTreeWidgetItem(parent_item)
                # Short leaf name for display
                leaf = name.split('/')[-1] if '/' in name else name
                child_item.setText(0, leaf)
                child_item.setToolTip(0, name)
                child_item.setData(0, Qt.UserRole, name)

                # Hz chip in column 1
                ts_deque = self.topic_last_msg.get(name)
                if ts_deque:
                    hz = _topic_hz(ts_deque)
                    hz_text = '%.1f Hz' % hz if hz > 0 else '-'
                else:
                    hz_text = '-'
                child_item.setText(1, hz_text)
                child_item.setTextAlignment(1, Qt.AlignCenter)

                # Monitor toggle button in column 2 via setItemWidget
                is_active = name in self.monitored_topics
                mon_btn = QPushButton('Stop' if is_active else 'Monitor')
                # Transient row widget rebuilt on every refresh — style it
                # directly (theme.bind would accumulate dead entries); the page
                # connects theme_changed -> _filter_topics to restyle on switch.
                _t = theme.current()
                if is_active:
                    mon_btn.setStyleSheet(
                        'QPushButton {'
                        ' background: %s; color: %s;'
                        ' border: 1px solid %s; border-radius: 4px;'
                        ' padding: 2px 8px; font-size: 11px;'
                        '}' % (_t['bg_overlay'], _t['accent_teal'],
                               _t['border_subtle']))
                else:
                    mon_btn.setStyleSheet(
                        'QPushButton {'
                        ' padding: 2px 8px; font-size: 11px;'
                        ' border-radius: 4px;'
                        '}')
                mon_btn.clicked.connect(
                    partial(self._toggle_monitor, name, msg_type)
                )
                self.topic_tree.setItemWidget(child_item, 2, mon_btn)

                self._tree_child_items[name] = child_item
    
    def _toggle_monitor(self, topic_name: str, msg_type: str):
        """Toggle monitoring of a topic and persist state to settings."""
        if topic_name in self.monitored_topics:
            self._stop_monitoring(topic_name)
        else:
            self._start_monitoring(topic_name, msg_type)

        # Rebuild tree to update button states
        self._filter_topics()
        self.monitored_count_label.setText(
            'Monitoring: %d' % len(self.monitored_topics)
        )
        # Persist to settings
        self._save_monitored_topics()
    
    def _start_monitoring(self, topic_name: str, msg_type: str):
        """Start monitoring a topic."""
        try:
            # Import message type dynamically
            parts = msg_type.split('/')
            if len(parts) >= 3:
                pkg = parts[0]
                msg_folder = parts[1]  # 'msg', 'srv', 'action'
                msg_name = parts[2]
                
                # Import the message type
                import importlib
                module = importlib.import_module(f"{pkg}.{msg_folder}")
                MsgType = getattr(module, msg_name)
                
                # Create subscription
                def callback(msg, topic=topic_name):
                    self._on_message_received(topic, msg)
                
                sub = self.node.create_subscription(
                    MsgType,
                    topic_name,
                    callback,
                    10
                )
                
                self.monitored_topics[topic_name] = sub
                self.topic_rates[topic_name] = 0
                self.topic_msg_count[topic_name] = 0
                self.topic_last_msg[topic_name] = deque(maxlen=100)
                
                self.log_message("INFO", f"Started monitoring: {topic_name}", "Monitor")
            else:
                self.log_message("WARN", f"Cannot parse message type: {msg_type}", "Monitor")
                
        except Exception as e:
            self.log_message("ERROR", f"Failed to monitor {topic_name}: {e}", "Monitor")
    
    def _stop_monitoring(self, topic_name: str):
        """Stop monitoring a topic and clean all associated state.

        Destroys the ROS 2 subscription and removes *topic_name* from
        ``monitored_topics``, ``topic_rates``, ``topic_msg_count``,
        ``topic_last_msg``, and ``topic_cached_msg`` to prevent memory /
        state leaks.
        """
        if topic_name in self.monitored_topics:
            try:
                self.node.destroy_subscription(self.monitored_topics[topic_name])
            except Exception:
                pass
            del self.monitored_topics[topic_name]
            # Clean auxiliary dicts to avoid stale-data leaks
            self.topic_rates.pop(topic_name, None)
            self.topic_msg_count.pop(topic_name, None)
            self.topic_last_msg.pop(topic_name, None)
            self.topic_cached_msg.pop(topic_name, None)
            self.log_message('INFO', 'Stopped monitoring: %s' % topic_name, 'Monitor')
    
    def _on_message_received(self, topic_name: str, msg):
        """Handle a received message (called from the spin thread).

        Records the timestamp, increments the counter, caches the raw message
        object for inspector introspection, converts to string for the raw
        stream, and emits ``message_received`` for thread-safe UI delivery.
        """
        timestamp = datetime.now().timestamp()

        # Update rate tracking
        if topic_name not in self.topic_last_msg:
            self.topic_last_msg[topic_name] = deque(maxlen=100)
        self.topic_last_msg[topic_name].append(timestamp)

        # Update count
        self.topic_msg_count[topic_name] = (
            self.topic_msg_count.get(topic_name, 0) + 1
        )

        # Cache raw message for the inspector (thread-safe assignment)
        self.topic_cached_msg[topic_name] = msg

        # Convert to string for the raw stream
        try:
            msg_str = str(msg)
            if len(msg_str) > 2000:
                msg_str = msg_str[:2000] + '... [truncated]'
        except Exception:
            msg_str = '[Unable to convert message to string]'

        # Emit signal for thread-safe UI update
        self.topic_signals.message_received.emit(topic_name, msg_str, timestamp)
    
    def _display_message(self, topic_name: str, msg_str: str, timestamp: float):
        """Append one raw-stream block to the message display (Qt-thread slot).

        Guards: paused, wrong topic selected, widget not yet built.
        Enforces the buffer size (msg_buffer_spin) by removing the oldest
        block when the document exceeds the limit.
        """
        if not hasattr(self, 'pause_btn') or self.pause_btn.isChecked():
            return
        if not hasattr(self, 'msg_display'):
            return

        # Only show messages for the currently selected topic
        if self._selected_topic != topic_name:
            return

        time_str = datetime.fromtimestamp(timestamp).strftime('%H:%M:%S.%f')[:-3]
        ts_color = theme.current()['text_secondary']
        self.msg_display.append(
            "<span style='color:%s;'>[%s]</span> %s"
            % (ts_color, time_str, msg_str)
        )

        # Enforce buffer limit
        doc = self.msg_display.document()
        if doc.blockCount() > self.msg_buffer_spin.value():
            cursor = self.msg_display.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.movePosition(cursor.Down, cursor.KeepAnchor)
            cursor.removeSelectedText()

        # Update total count in stats bar
        total = sum(self.topic_msg_count.values())
        self.total_msg_label.setText('Total Messages: %d' % total)
    
    def _update_rates(self):
        """Update Hz chip text for each visible child item (1 Hz tick).

        Iterates ``_tree_child_items`` and patches column 1 text in-place,
        avoiding a full tree rebuild that would reset expansion state and
        re-create all monitor buttons.
        """
        child_map = getattr(self, '_tree_child_items', {})
        for topic_name, child_item in child_map.items():
            ts_deque = self.topic_last_msg.get(topic_name)
            if ts_deque:
                hz = _topic_hz(ts_deque)
                hz_text = '%.1f Hz' % hz if hz > 0 else '-'
            else:
                hz_text = '-'
            child_item.setText(1, hz_text)
    
    def _on_topic_selected(self):
        """Handle tree selection change; wire selected child to the inspector.

        Delegates to ``_select_topic(topic_name)`` after resolving the
        selected tree item.  Only child items (topics) trigger inspector
        population — parent items (namespace headers) are not selectable so
        this slot fires only for leaf nodes.
        """
        # Prefer currentItem(); fall back to selectedItems()[0]
        item = self.topic_tree.currentItem()
        if item is None:
            sel = self.topic_tree.selectedItems()
            item = sel[0] if sel else None
        if item is None:
            self._select_topic(None)
            return

        # Namespace parent items are top-level (no parent); skip them.
        if item.parent() is None:
            self._select_topic(None)
            return
        topic_name = item.data(0, Qt.UserRole)
        if not topic_name:
            self._select_topic(None)
            return
        self._select_topic(topic_name)

    def _select_topic(self, topic_name):
        """Activate the inspector for *topic_name* (None = clear selection).

        Separated from ``_on_topic_selected`` so that tests can call this
        directly without needing a live Qt selection event.
        """
        if not topic_name:
            self._selected_topic = None
            if hasattr(self, 'selected_topic_label'):
                self.selected_topic_label.setText('Select a topic to inspect')
            return

        self._selected_topic = topic_name

        if hasattr(self, 'selected_topic_label'):
            self.selected_topic_label.setText('Topic: %s' % topic_name)

        # Auto-subscribe when a topic is selected for inspection
        if topic_name not in self.monitored_topics:
            msg_type = self._tree_topic_types.get(topic_name, '')
            if msg_type:
                self._start_monitoring(topic_name, msg_type)
                self._filter_topics()
                self.monitored_count_label.setText(
                    'Monitoring: %d' % len(self.monitored_topics)
                )
                self._save_monitored_topics()

        # Clear raw stream for the newly selected topic
        if hasattr(self, 'msg_display'):
            self.msg_display.clear()

        # Rebuild inspector fields from any cached message
        msg = self.topic_cached_msg.get(topic_name)
        if msg is not None:
            fields = _introspect_msg(msg)
            self._build_inspector_fields(fields)
        elif hasattr(self, '_insp_fields_layout'):
            # Clear stale fields from a previous selection
            while self._insp_fields_layout.count():
                item_obj = self._insp_fields_layout.takeAt(0)
                if item_obj.widget():
                    item_obj.widget().deleteLater()
            self._insp_value_labels = {}
            self._insp_limitbars = {}
    
    def _clear_messages(self):
        """Clear the message display."""
        self.msg_display.clear()
    
    def _toggle_pause(self, paused: bool):
        """Toggle message display pause."""
        self.pause_btn.setText("▶ Resume" if paused else "‖ Pause")
    
    def _toggle_auto_refresh(self, enabled: bool):
        """Toggle auto-refresh of topic list."""
        if enabled:
            self.topic_list_timer.start(2000)
        else:
            self.topic_list_timer.stop()
    
    def show_topic_monitor_panel(self):
        """Show the topic monitor panel."""
        self._set_active_sidebar_btn(self._side_buttons[11])
        self._show_page('topic_monitor')