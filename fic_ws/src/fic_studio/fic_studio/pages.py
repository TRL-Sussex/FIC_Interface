"""Name-addressed pages for the main window's stack.

Stack positions were hardcoded in three independent places — the order of
``addWidget``, a positional button list carrying ``None`` holes, and four
periodic refreshers comparing ``currentIndex()`` to a literal. Nothing tied
them together, so moving or removing a page silently mis-routed navigation and
left the throttles watching whichever page had shifted into their index.

One registry owns the mapping. Navigation and "is my page showing?" resolve
through the same names, so they cannot drift apart.
"""


class PageRegistry:
    """Maps page names to widgets in a ``QStackedWidget``."""

    def __init__(self, stack):
        """Bind to the stack whose pages this registry owns."""
        self._stack = stack
        self._index = {}
        self._buttons = {}

    # ── Registration ──────────────────────────────────────────────────────

    def register(self, name, widget, button=None):
        """Add ``widget`` as page ``name`` and return its index.

        ``button`` is the sidebar entry to mark active when the page shows;
        pages reached only in code may omit it.
        """
        index = self._stack.addWidget(widget)
        self._index[name] = index
        if button is not None:
            self._buttons[name] = button
        return index

    def bind_button(self, name, button):
        """Attach a sidebar button to an already-registered page."""
        self._buttons[name] = button

    # ── Lookup ────────────────────────────────────────────────────────────

    def index(self, name):
        """Stack index of ``name``, or None if it was never registered."""
        return self._index.get(name)

    def widget(self, name):
        """The widget registered as ``name``, or None."""
        index = self._index.get(name)
        return None if index is None else self._stack.widget(index)

    def button(self, name):
        """The sidebar button bound to ``name``, or None."""
        return self._buttons.get(name)

    def names(self):
        """Every registered page name, in registration order."""
        return list(self._index)

    # ── Navigation ────────────────────────────────────────────────────────

    def show(self, name):
        """Show page ``name``. Returns False for an unknown name.

        Deliberately does not fall back to page zero: a typo that silently
        navigates somewhere plausible is the failure this registry exists to
        remove.
        """
        index = self._index.get(name)
        if index is None:
            return False
        self._stack.setCurrentIndex(index)
        return True

    def is_visible(self, name):
        """True when ``name`` is the page currently on screen.

        The periodic refreshers ask this instead of comparing an index, so
        navigation and throttling share one definition of "showing".
        """
        index = self._index.get(name)
        return index is not None and self._stack.currentIndex() == index


def page_showing(self, name):
    """Whether ``name`` is the visible page, for use inside a mixin.

    Bound as ``_page_showing`` on the mixins whose periodic refreshers need to
    skip work while their page is hidden. It defers to the host's
    ``is_page_visible`` so navigation and throttling share one definition, and
    returns True when the host has no registry at all — a refresher driven
    directly in a headless test should run, not silently no-op.
    """
    check = getattr(self, 'is_page_visible', None)
    return check(name) if callable(check) else True
