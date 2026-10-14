"""
Helpers shared between extensions.

``ProcessRunner`` is re-exported lazily: importing it pulls in PyQt5, and the
pure-Python parts of ``visual_runtime`` (schema, engine, writer, codegen) are
unit-tested on machines with no Qt installed.  Importing
``Extensions._shared.<anything>`` must therefore not drag Qt in by itself.
"""

__all__ = ["ProcessRunner"]


def __getattr__(name):
    """Import ProcessRunner on first use (PEP 562)."""
    if name == "ProcessRunner":
        from .process_runner import ProcessRunner
        return ProcessRunner
    raise AttributeError("module %r has no attribute %r" % (__name__, name))


def __dir__():
    """List the lazily-exported names alongside the real module contents."""
    return sorted(list(globals().keys()) + __all__)
