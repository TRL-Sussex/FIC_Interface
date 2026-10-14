"""Show a Studio panel in a browser: describe its widgets, and act on them.

The Studio's extensions are Qt pages. Rather than write each one again in
HTML, this describes the page's widget tree as plain data — buttons, tick
boxes, drop-downs, numbers, sliders, text, labels — and applies what the
browser sends back to the same widgets, on the Qt thread. The page a phone
sees is therefore the page the Studio has, including extensions written after
this module.

What it deliberately does not do: paint. A mirrored page is a readable list of
the controls in their original order and grouping, not a pixel copy. Anything
it does not understand (a plot, a 3D view, a custom widget) appears as a note
saying so, so nobody is left wondering where a control went.

Widgets are addressed by their position in the tree ('0.3.1'), with the shape
of the tree carried alongside as ``sig``; a stale path is refused rather than
guessed at, so a page that changed under a phone cannot be clicked blindly.
"""
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QTextDocumentFragment
from PyQt5.QtWidgets import (
    QAbstractButton, QAbstractItemView, QAbstractSlider, QCheckBox, QComboBox,
    QDoubleSpinBox, QGroupBox, QLabel, QLineEdit, QListWidget, QPlainTextEdit,
    QProgressBar, QPushButton, QRadioButton, QScrollBar, QSpinBox, QStackedWidget,
    QTabWidget, QTextEdit, QWidget,
)

#: Widgets whose children are not worth walking into.
_LEAF_TEXT = (QLabel, QLineEdit, QPlainTextEdit, QTextEdit)
#: How much text is sent from a log or a text box.
MAX_TEXT = 4000
#: How many rows of a table or list are sent.
MAX_ROWS = 200


def describe(root):
    """Return the panel under ``root`` as JSON-able data."""
    nodes = _children(root, [])
    return {'sig': _signature(nodes), 'nodes': nodes}


def act(root, path, kind, value=None):
    """Do what the browser asked on the widget at ``path``; return True if done.

    Raises ValueError when the path names nothing, or names a widget of
    another kind — a page that changed while a phone was looking at it.
    """
    widget = _at(root, path)
    if widget is None:
        raise ValueError('no widget at %s' % path)
    if _kind(widget) != kind:
        raise ValueError('%s is not a %s any more' % (path, kind))
    if not widget.isEnabled() or not _visible(widget):
        raise ValueError('that control is not available')
    return _apply(widget, kind, value)


# ── describing ──────────────────────────────────────────────────────────
def _children(widget, prefix):
    out = []
    for index, child in enumerate(widget.children()):
        if not isinstance(child, QWidget) or isinstance(child, QScrollBar):
            continue                      # a scroll area's bars are not controls
        if not _visible(child):
            continue
        path = prefix + [index]
        node = _node(child, '.'.join(str(p) for p in path))
        if node is not None:
            out.append(node)
    return _drop_repeated_labels(out)


def _drop_repeated_labels(nodes):
    """Leave out a label whose text the next control already carries."""
    out = []
    for index, node in enumerate(nodes):
        following = nodes[index + 1] if index + 1 < len(nodes) else None
        if (node['kind'] == 'label' and following is not None
                and following['kind'] in ('choice', 'number', 'slider', 'text')
                and _same_label(following.get('label'), node.get('label'))):
            continue
        out.append(node)
    return out


def _same_label(a, b):
    norm = lambda t: (t or '').strip().rstrip(':').strip().lower()  # noqa: E731
    return norm(a) == norm(b) != ''


def _node(widget, path):
    kind = _kind(widget)
    node = {'path': path, 'kind': kind, 'enabled': bool(widget.isEnabled())}
    tip = widget.toolTip()
    if tip:
        node['tip'] = tip[:200]
    if kind == 'group':
        node['label'] = widget.title()
        node['nodes'] = _children(widget, [int(p) for p in path.split('.')])
    elif kind == 'tabs':
        node['label'] = ''
        node['tabs'] = [{'label': widget.tabText(i),
                         'current': i == widget.currentIndex()}
                        for i in range(widget.count())]
        current = widget.currentWidget()
        node['nodes'] = _children(current, [int(p) for p in path.split('.')]) \
            if current is not None else []
    elif kind == 'button':
        node['label'] = (_plain(widget.text()) or widget.toolTip()[:40] or widget.accessibleName()
                         or _readable(widget.objectName()) or 'Button')
        if widget.isCheckable():
            node['kind'] = kind = 'toggle'
            node['value'] = bool(widget.isChecked())
    elif kind == 'toggle':
        node['label'] = widget.text()
        node['value'] = bool(widget.isChecked())
    elif kind == 'choice':
        node['label'] = _label_for(widget)
        node['options'] = [widget.itemText(i) for i in range(widget.count())]
        node['value'] = widget.currentIndex()
    elif kind == 'number':
        node['label'] = _label_for(widget)
        node['value'] = float(widget.value())
        node['min'], node['max'] = float(widget.minimum()), float(widget.maximum())
        node['step'] = float(widget.singleStep())
        node['suffix'] = widget.suffix()
        node['decimals'] = widget.decimals() if isinstance(widget, QDoubleSpinBox) else 0
    elif kind == 'slider':
        node['label'] = _label_for(widget)
        node['value'] = float(widget.value())
        node['min'], node['max'] = float(widget.minimum()), float(widget.maximum())
    elif kind == 'text':
        node['label'] = _label_for(widget)
        node['value'] = widget.text()
    elif kind == 'label':
        text = _plain(widget.text())
        if not text.strip():
            return None
        node['label'] = text[:400]
        if _is_heading(widget):
            node['heading'] = True
    elif kind == 'list':
        node['label'] = _label_for(widget)
        node['items'] = [widget.item(i).text() for i in range(min(widget.count(), MAX_ROWS))]
        node['value'] = widget.currentRow()
    elif kind == 'table':
        node['label'] = _label_for(widget)
        node['columns'], node['rows'] = _table(widget)
    elif kind == 'log':
        node['label'] = _label_for(widget)
        node['value'] = widget.toPlainText()[-MAX_TEXT:]
    elif kind == 'bar':
        node['label'] = widget.format()
        node['value'] = float(widget.value())
        node['min'], node['max'] = float(widget.minimum()), float(widget.maximum())
    elif kind == 'other':
        kids = _children(widget, [int(p) for p in path.split('.')])
        if kids:
            node['kind'] = 'box'
            node['nodes'] = kids
        elif widget.__class__.__name__ in ('QWidget', 'QFrame', 'QScrollArea',
                                           'QSplitter', 'QStackedWidget'):
            return None                 # an empty container is not worth a line
        else:
            node['label'] = widget.__class__.__name__
    return node


def _kind(widget):
    if isinstance(widget, QListWidget):
        return 'list'
    if isinstance(widget, QAbstractItemView):
        return 'table'                    # tables, trees and other item views: read-only rows
    if isinstance(widget, QGroupBox):
        return 'group'
    if isinstance(widget, QTabWidget):
        return 'tabs'
    if isinstance(widget, (QCheckBox, QRadioButton)):
        return 'toggle'
    if isinstance(widget, QPushButton):
        return 'button'
    if isinstance(widget, QComboBox):
        return 'choice'
    if isinstance(widget, (QSpinBox, QDoubleSpinBox)):
        return 'number'
    if isinstance(widget, QAbstractSlider):
        return 'slider'
    if isinstance(widget, QLineEdit):
        return 'text'
    if isinstance(widget, QLabel):
        return 'label'
    if isinstance(widget, (QTextEdit, QPlainTextEdit)):
        return 'log'
    if isinstance(widget, QProgressBar):
        return 'bar'
    if isinstance(widget, QAbstractButton):
        return 'button'
    return 'other'


def _plain(text):
    """Rich text ('<b>Start</b> tracking') as the plain words a browser shows."""
    text = text or ''
    if Qt.mightBeRichText(text):
        return QTextDocumentFragment.fromHtml(text).toPlainText()
    return text


def _readable(name):
    """An objectName as words ('play_btn' -> 'Play'), or '' for Qt's own names."""
    if not name or name.startswith('qt_'):
        return ''
    words = [w for w in name.replace('-', '_').split('_') if w and w.lower() not in ('btn', 'button', 'pb')]
    return ' '.join(words).capitalize()


def _is_heading(label):
    """A label set in bold or larger type: a section title on the Qt page."""
    font = label.font()
    style = (label.styleSheet() or '').replace(' ', '').lower()
    text = (label.text() or '').lstrip().lower()
    return (font.bold() or font.pointSizeF() >= 13 or 'font-weight:bold' in style
            or 'font-weight:6' in style or 'font-weight:7' in style or text.startswith(('<b>', '<h')))


def _table(view):
    """A table or tree's header and first rows, as text."""
    model = view.model()
    if model is None:
        return [], []
    cols = model.columnCount()
    columns = [str(model.headerData(c, Qt.Horizontal) or '') for c in range(cols)]
    rows = []
    for r in range(min(model.rowCount(), MAX_ROWS)):
        rows.append([str(model.index(r, c).data() if model.index(r, c).data() is not None else '') for c in range(cols)])
    return columns, rows


def _label_for(widget):
    """Return a name for a control: the label beside it, else its own name.

    Qt forms put the name in a QLabel next to the control, so that is the
    first place to look; the tooltip is kept as a tooltip, not repeated as a
    heading.
    """
    name = _readable(widget.objectName())
    if name:
        return name
    parent = widget.parentWidget()
    previous = None
    for child in (parent.children() if parent is not None else []):
        if child is widget:
            break
        if isinstance(child, QWidget):
            previous = child                  # only the widget right before it
    if isinstance(previous, QLabel) and previous.text().strip():
        return _plain(previous.text()).strip().rstrip(':').strip()
    return ''


def _visible(widget):
    """Say whether the widget is shown. An unshown window's children still count."""
    if widget.isHidden():
        return False
    stack = widget.parentWidget()
    if isinstance(stack, QStackedWidget) and stack.currentWidget() is not widget:
        return False
    return True


def _signature(nodes):
    """Return a short fingerprint of the tree's shape, to spot a page that changed."""
    import hashlib
    parts = []

    def walk(items):
        for node in items:
            parts.append('%s:%s' % (node['path'], node['kind']))
            walk(node.get('nodes') or [])
    walk(nodes)
    return hashlib.sha1('|'.join(parts).encode()).hexdigest()[:12]


# ── acting ──────────────────────────────────────────────────────────────
def _at(root, path):
    widget = root
    try:
        for step in str(path).split('.'):
            children = widget.children()
            index = int(step)
            if index < 0 or index >= len(children):
                return None
            widget = children[index]
            if not isinstance(widget, QWidget):
                return None
    except (TypeError, ValueError):
        return None
    return widget


def _apply(widget, kind, value):
    if kind == 'button':
        widget.click()
    elif kind == 'toggle':
        widget.setChecked(bool(value))
    elif kind == 'choice':
        index = int(value)
        if not 0 <= index < widget.count():
            raise ValueError('no such option')
        widget.setCurrentIndex(index)
    elif kind in ('number', 'slider'):
        number = float(value)
        if isinstance(widget, QSpinBox) or kind == 'slider':
            widget.setValue(int(round(number)))
        else:
            widget.setValue(number)
    elif kind == 'text':
        widget.setText(str(value)[:500])
        widget.editingFinished.emit()
    elif kind == 'list':
        index = int(value)
        if not 0 <= index < widget.count():
            raise ValueError('no such row')
        widget.setCurrentRow(index)
        item = widget.item(index)
        widget.itemClicked.emit(item)             # what a click on the row would do
    elif kind == 'tabs':
        index = int(value)
        if not 0 <= index < widget.count():
            raise ValueError('no such tab')
        widget.setCurrentIndex(index)
    else:
        raise ValueError('%s cannot be used from the web page' % kind)
    return True
