"""Pieces the web remote's command modules share."""
import contextlib
import math


class Refused(Exception):
    """A command the Studio will not run; the message is shown on the phone."""

    def __init__(self, message, code='refused', **extra):
        super().__init__(message)
        self.code = code
        self.extra = extra

    def reply(self):
        reply = {'ok': False, 'error': self.code, 'message': str(self)}
        reply.update(self.extra)
        return reply


def _num(value, default=None):
    """Return ``value`` as a finite float, or ``default``."""
    try:
        value = float(value)
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def _arg_num(args, key):
    value = _num(args.get(key))
    if value is None:
        raise Refused('%s must be a number' % key, 'bad_args')
    return value


def _who(client):
    name = (client or {}).get('name') or 'browser'
    addr = (client or {}).get('addr') or '?'
    return '%s @ %s' % (name, addr)


def need_confirm(args, message):
    """Refuse unless the caller sent ``confirm: true`` (an explicit second step)."""
    if args.get('confirm') is not True:
        raise Refused(message, 'confirm')


@contextlib.contextmanager
def scoped_to_selected(w):
    """Make the Studio's "first robot" publishers follow the selected robot.

    The classic control bar publishes the mode, the postural / elbow / HAP-FIC /
    planner toggles, the active config and a tuning reload to the Studio's own
    robot, whichever robot the bar is driving (joint, pose and gripper targets
    already follow ``_rc_active_rid``). From the web UI every command must reach
    only the robot the toolbar selector shows, so for the length of one handler
    an unspecified robot means the selected one. Handlers run on the Qt thread
    one at a time, so the temporary override cannot leak into another command.
    """
    node = getattr(w, 'node', None)
    rid = getattr(w, '_rc_active_rid', None)
    own = getattr(node, 'station_robot_id', None)
    if node is None or not rid or rid == own:
        yield
        return
    original = node._resolve_rid
    saved = {'_resolve_rid': node.__dict__.get('_resolve_rid'),
             'active_profile_publisher_': node.__dict__.get('active_profile_publisher_')}
    saved_planner = w.__dict__.get('_publish_admittance_planner')
    node._resolve_rid = lambda robot_id=None: original(rid if robot_id is None else robot_id)
    pub = (getattr(node, 'pubs', {}).get(rid) or {}).get('active_profile')
    if pub is not None:
        node.active_profile_publisher_ = pub
    orig_planner = w._publish_admittance_planner
    w._publish_admittance_planner = lambda enabled, robot_id=None: orig_planner(enabled, robot_id or rid)
    try:
        yield
    finally:
        if saved['_resolve_rid'] is None:
            node.__dict__.pop('_resolve_rid', None)
        else:
            node._resolve_rid = saved['_resolve_rid']
        if saved['active_profile_publisher_'] is not None or 'active_profile_publisher_' in node.__dict__:
            node.active_profile_publisher_ = saved['active_profile_publisher_']
        if saved_planner is None:
            w.__dict__.pop('_publish_admittance_planner', None)
        else:
            w._publish_admittance_planner = saved_planner
