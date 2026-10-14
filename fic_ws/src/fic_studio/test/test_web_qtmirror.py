# Copyright 2025 Adnan Takriti
# SPDX-License-Identifier: MIT
"""Showing a Studio panel in a browser: the widget mirror."""
from fic_studio.web_remote import qtmirror
from PyQt5.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QGroupBox, QLabel, QLineEdit,
    QPlainTextEdit, QPushButton, QSlider, QTabWidget, QVBoxLayout, QWidget,
)
from PyQt5.QtCore import Qt
import pytest


def flat(described):
    """Every node, whatever its depth, keyed by label."""
    out = {}

    def walk(nodes):
        for node in nodes:
            out.setdefault(node.get('label', ''), []).append(node)
            walk(node.get('nodes') or [])
    walk(described['nodes'])
    return out


@pytest.fixture
def panel(qapp):
    page = QWidget()
    lay = QVBoxLayout(page)
    lay.addWidget(QLabel('Torque map'))
    go = QPushButton('Run')
    lay.addWidget(go)
    live = QCheckBox('Live')
    lay.addWidget(live)
    lay.addWidget(QLabel('Mode'))
    choice = QComboBox()
    choice.addItems(['Joint', 'Cartesian'])
    lay.addWidget(choice)
    lay.addWidget(QLabel('Speed'))
    speed = QDoubleSpinBox()
    speed.setRange(0.0, 10.0)
    speed.setValue(2.5)
    speed.setSuffix(' m/s')
    lay.addWidget(speed)
    slider = QSlider(Qt.Horizontal)
    slider.setRange(0, 100)
    slider.setValue(40)
    slider.setObjectName('Blend')
    lay.addWidget(slider)
    name = QLineEdit('run-1')
    name.setObjectName('Run name')
    lay.addWidget(name)
    log = QPlainTextEdit('started\nfinished')
    log.setObjectName('Log')
    lay.addWidget(log)
    box = QGroupBox('Limits')
    inner = QVBoxLayout(box)
    stop = QPushButton('Stop')
    inner.addWidget(stop)
    lay.addWidget(box)
    page.widgets = {'go': go, 'live': live, 'choice': choice, 'speed': speed,
                    'slider': slider, 'name': name, 'log': log, 'stop': stop}
    return page


def test_every_kind_of_control_is_described(panel):
    seen = flat(qtmirror.describe(panel))
    assert seen['Run'][0]['kind'] == 'button'
    assert seen['Live'][0] == {'path': seen['Live'][0]['path'], 'kind': 'toggle',
                               'enabled': True, 'label': 'Live', 'value': False}
    assert seen['Mode'][0]['options'] == ['Joint', 'Cartesian']
    assert seen['Speed'][0]['value'] == 2.5
    assert seen['Speed'][0]['suffix'] == ' m/s'
    assert seen['Blend'][0]['kind'] == 'slider' and seen['Blend'][0]['value'] == 40
    assert seen['Run name'][0]['value'] == 'run-1'
    assert seen['Log'][0]['value'] == 'started\nfinished'
    assert seen['Torque map'][0]['kind'] == 'label'


def test_a_control_takes_the_label_beside_it_and_not_twice(panel):
    """A Qt form is a label then a field; the browser shows that once."""
    described = qtmirror.describe(panel)
    labels = [n['label'] for n in described['nodes']]
    assert labels.count('Mode') == 1          # not the QLabel and the box
    choice = [n for n in described['nodes'] if n['kind'] == 'choice'][0]
    assert choice['label'] == 'Mode'
    # a control with a name of its own keeps it, and does not borrow the
    # label belonging to the control before it
    assert [n for n in described['nodes'] if n['kind'] == 'slider'][0]['label'] == 'Blend'


def test_a_group_keeps_its_children(panel):
    described = qtmirror.describe(panel)
    group = [n for n in described['nodes'] if n['kind'] == 'group'][0]
    assert group['label'] == 'Limits'
    assert [n['label'] for n in group['nodes']] == ['Stop']


def test_a_button_is_clicked(panel):
    clicks = []
    panel.widgets['go'].clicked.connect(lambda: clicks.append(1))
    path = flat(qtmirror.describe(panel))['Run'][0]['path']
    assert qtmirror.act(panel, path, 'button')
    assert clicks == [1]


def test_a_button_inside_a_group_is_clicked(panel):
    clicks = []
    panel.widgets['stop'].clicked.connect(lambda: clicks.append(1))
    described = qtmirror.describe(panel)
    group = [n for n in described['nodes'] if n['kind'] == 'group'][0]
    qtmirror.act(panel, group['nodes'][0]['path'], 'button')
    assert clicks == [1]


def test_values_are_set(panel):
    seen = flat(qtmirror.describe(panel))
    qtmirror.act(panel, seen['Live'][0]['path'], 'toggle', True)
    qtmirror.act(panel, seen['Mode'][0]['path'], 'choice', 1)
    qtmirror.act(panel, seen['Speed'][0]['path'], 'number', 7.25)
    qtmirror.act(panel, seen['Blend'][0]['path'], 'slider', 80)
    qtmirror.act(panel, seen['Run name'][0]['path'], 'text', 'run-2')
    assert panel.widgets['live'].isChecked()
    assert panel.widgets['choice'].currentIndex() == 1
    assert panel.widgets['speed'].value() == 7.25
    assert panel.widgets['slider'].value() == 80
    assert panel.widgets['name'].text() == 'run-2'


def test_a_disabled_control_is_not_used(panel):
    panel.widgets['go'].setEnabled(False)
    described = qtmirror.describe(panel)
    node = flat(described)['Run'][0]
    assert node['enabled'] is False
    with pytest.raises(ValueError):
        qtmirror.act(panel, node['path'], 'button')


def test_a_path_that_names_nothing_is_refused(panel):
    with pytest.raises(ValueError):
        qtmirror.act(panel, '99.99', 'button')
    with pytest.raises(ValueError):
        qtmirror.act(panel, flat(qtmirror.describe(panel))['Run'][0]['path'], 'toggle', True)


def test_the_signature_changes_with_the_page(panel, qapp):
    before = qtmirror.describe(panel)['sig']
    assert qtmirror.describe(panel)['sig'] == before
    panel.layout().addWidget(QPushButton('New'))
    assert qtmirror.describe(panel)['sig'] != before


def test_a_hidden_control_is_not_offered(panel):
    panel.widgets['go'].hide()
    assert 'Run' not in flat(qtmirror.describe(panel))


def test_tabs_show_the_open_one(qapp):
    page = QWidget()
    lay = QVBoxLayout(page)
    tabs = QTabWidget()
    first, second = QWidget(), QWidget()
    QVBoxLayout(first).addWidget(QPushButton('In first'))
    QVBoxLayout(second).addWidget(QPushButton('In second'))
    tabs.addTab(first, 'One')
    tabs.addTab(second, 'Two')
    lay.addWidget(tabs)
    node = [n for n in qtmirror.describe(page)['nodes'] if n['kind'] == 'tabs'][0]
    assert [t['label'] for t in node['tabs']] == ['One', 'Two']
    assert node['tabs'][0]['current'] is True
    assert 'In first' in flat(qtmirror.describe(page))
    qtmirror.act(page, node['path'], 'tabs', 1)
    assert tabs.currentIndex() == 1
    assert 'In second' in flat(qtmirror.describe(page))


def test_lists_tables_headings_and_rich_text(qapp):
    from PyQt5.QtWidgets import QListWidget, QScrollArea, QTableWidget, QTableWidgetItem
    page = QWidget()
    lay = QVBoxLayout(page)
    head = QLabel('RECORDED WAYPOINTS')
    head.setStyleSheet('font-weight: bold;')
    lay.addWidget(head)
    lay.addWidget(QLabel('Press <b>Start tracking</b> to begin'))
    items = QListWidget()
    items.addItems(['1. JOINT', '2. CART'])
    clicked = []
    items.itemClicked.connect(lambda item: clicked.append(item.text()))
    lay.addWidget(items)
    table = QTableWidget(1, 2)
    table.setHorizontalHeaderLabels(['#', 'Time'])
    table.setItem(0, 0, QTableWidgetItem('1'))
    table.setItem(0, 1, QTableWidgetItem('0.5 s'))
    lay.addWidget(table)
    icon_only = QPushButton()
    icon_only.setObjectName('clear_btn')
    lay.addWidget(icon_only)
    scroll = QScrollArea()                   # its scroll bars are not controls
    scroll.setWidget(QLabel('inside'))
    lay.addWidget(scroll)
    page.show()
    d = qtmirror.describe(page)
    nodes = flat(d)
    assert nodes['RECORDED WAYPOINTS'][0].get('heading') is True
    assert 'Press Start tracking to begin' in nodes                      # no tags
    lst = next(n for group in nodes.values() for n in group if n['kind'] == 'list')
    assert lst['items'] == ['1. JOINT', '2. CART'] and lst['value'] == -1
    tbl = next(n for group in nodes.values() for n in group if n['kind'] == 'table')
    assert tbl['columns'] == ['#', 'Time'] and tbl['rows'] == [['1', '0.5 s']]
    assert 'Clear' in nodes                                               # named from its objectName
    assert not any(n['kind'] == 'slider' for group in nodes.values() for n in group)
    qtmirror.act(page, lst['path'], 'list', 1)
    assert items.currentRow() == 1 and clicked == ['2. CART']
    with pytest.raises(ValueError):
        qtmirror.act(page, lst['path'], 'list', 5)
    page.close()


def test_repeated_label_is_dropped_whatever_its_colon(qapp):
    page = QWidget()
    lay = QVBoxLayout(page)
    lay.addWidget(QLabel('Target Robot:'))
    combo = QComboBox()
    combo.addItems(['Robot 1'])
    lay.addWidget(combo)
    page.show()
    nodes = qtmirror.describe(page)['nodes']
    assert [n['kind'] for n in nodes] == ['choice'] and nodes[0]['label'] == 'Target Robot'
    page.close()
