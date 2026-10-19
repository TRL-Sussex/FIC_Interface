"""Where FIC Studio keeps what belongs to this machine and its people, rather than to the software.

Two kinds of files, kept apart:

* **The lab's robot library** stays in the repository checkout, on purpose: robot configs
  (``fic_studio_robots.yaml``, ``fic_library/config``), tuning (``config.xml``), imported robot
  models and driver libraries. The controllers read them from there and the lab shares them
  through git (see ``config``).
* **This machine's state** — settings, web accounts and certificate, the audit log — lives in the
  home directory, and **recordings** (experiments, bags) in a data folder that can be moved.

    state_dir()        $FIC_STUDIO_HOME, else ~/.fic_studio
    settings_file()    $FIC_STUDIO_SETTINGS, else ~/.robot_selector_gui_settings.yaml (the
                       path every Studio version reads, so switching branches keeps them)
    experiments_dir()  $FIC_EXPERIMENTS_DIR, else Settings > Data > Experiments folder, else
                       <repo>/experiments when the checkout can be written to, else
                       $XDG_DATA_HOME/fic_studio/experiments (~/.local/share/...)

Nothing here imports Qt or ROS, or the rest of the package: ``config`` imports it.
"""
import os

SETTINGS_ENV = 'FIC_STUDIO_SETTINGS'
STATE_ENV = 'FIC_STUDIO_HOME'
EXPERIMENTS_ENV = 'FIC_EXPERIMENTS_DIR'
LEGACY_SETTINGS = '~/.robot_selector_gui_settings.yaml'


def _expand(p):
    return os.path.abspath(os.path.expandvars(os.path.expanduser(str(p))))


def state_dir():
    """This machine's Studio state (accounts, certificate, audit log, hardware-model runs)."""
    return _expand(os.environ.get(STATE_ENV) or '~/.fic_studio')


def state_path(*parts):
    return os.path.join(state_dir(), *parts)


def settings_file():
    """The settings YAML."""
    return _expand(os.environ.get(SETTINGS_ENV) or LEGACY_SETTINGS)


def data_home():
    """$XDG_DATA_HOME/fic_studio (~/.local/share/fic_studio)."""
    base = os.environ.get('XDG_DATA_HOME') or '~/.local/share'
    return os.path.join(_expand(base), 'fic_studio')


def _configured_experiments(settings_path=None):
    """Settings > Data > Experiments folder, read straight from the file ('' when unset)."""
    try:
        import yaml
        with open(settings_path or settings_file()) as f:
            blob = yaml.safe_load(f) or {}
        return str(((blob.get('paths') or {}).get('experiments')) or '').strip()
    except Exception:
        return ''


def _writable_dir(path):
    """True if ``path`` exists and can be written to, or could be created in a writable parent."""
    probe = path
    while probe and not os.path.exists(probe):
        parent = os.path.dirname(probe)
        if parent == probe:
            return False
        probe = parent
    return os.path.isdir(probe) and os.access(probe, os.W_OK)


def experiments_dir(repo_default, settings_path=None):
    """Where experiments and bags are recorded (see the module docstring for the order)."""
    chosen = os.environ.get(EXPERIMENTS_ENV) or _configured_experiments(settings_path)
    if chosen:
        return _expand(chosen)
    repo_default = str(repo_default)
    if _writable_dir(repo_default):
        return repo_default
    return os.path.join(data_home(), 'experiments')


def describe(repo_default):
    """Every location, for Help > About and the docs: [(what, path, why)]."""
    exp = experiments_dir(repo_default)
    why = ('$' + EXPERIMENTS_ENV if os.environ.get(EXPERIMENTS_ENV)
           else 'Settings > Data' if _configured_experiments()
           else 'the repository checkout' if exp == str(repo_default)
           else 'the checkout is read-only')
    return [('Settings', settings_file(), '$' + SETTINGS_ENV if os.environ.get(SETTINGS_ENV) else 'default'),
            ('Machine state', state_dir(), '$' + STATE_ENV if os.environ.get(STATE_ENV) else 'default'),
            ('Experiments and bags', exp, why)]
