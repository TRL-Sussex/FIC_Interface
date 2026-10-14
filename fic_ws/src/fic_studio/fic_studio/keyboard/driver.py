"""Driving the arm from the keyboard: which key means what, and how far.

Qt-free and ROS-free on purpose, so the mapping can be tested on its own: it
takes the set of keys held down and a time step, and says how much to move.

The layout is the one most simulators use, so a visitor already knows it:

    joint space      1-7 pick the joint, then Up/Down (or , and .) drive it
    Cartesian space  W/S forward and back, A/D left and right, Q/E up and
                     down, J/L yaw
    either           Tab swaps space, G works the gripper,
                     Shift is fast, Ctrl is slow

Nothing here decides *whether* the keyboard may drive the arm — that is the
Studio's business, and it is deliberately strict about it (see
mixins/keyboard_mixin.py).
"""

#: Metres per second, degrees per second for the wrist, at normal speed.
CART_SPEED = 0.06
YAW_SPEED = 25.0
#: Degrees per second for a joint, at normal speed.
JOINT_SPEED = 12.0
#: Shift and Ctrl multiply those.
FAST, SLOW = 3.0, 0.25
#: How far the gripper moves per second while its key is held, as a fraction.
GRIP_SPEED = 0.8

#: Cartesian keys: name -> (axis, direction). Axes are x, y, z, yaw.
CART_KEYS = {
    'W': ('x', +1), 'S': ('x', -1),
    'A': ('y', +1), 'D': ('y', -1),
    'Q': ('z', +1), 'E': ('z', -1),
    'J': ('yaw', +1), 'L': ('yaw', -1),
}
#: Joint keys: drive the selected joint.
JOINT_KEYS = {'UP': +1, '.': +1, 'DOWN': -1, ',': -1}
#: Gripper keys.
GRIP_KEYS = {'O': +1, 'C': -1}
#: The keys that pick a joint, in order.
PICK_KEYS = ('1', '2', '3', '4', '5', '6', '7', '8', '9')


def speed_factor(keys):
    """Return the multiplier for the modifiers held down."""
    if 'SHIFT' in keys:
        return FAST
    if 'CTRL' in keys:
        return SLOW
    return 1.0


def cartesian_step(keys, dt):
    """Return {axis: metres or degrees} for the keys held, over ``dt`` seconds.

    Opposite keys cancel, so W and S together hold still rather than fighting.
    """
    out = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
    for key in keys:
        axis_dir = CART_KEYS.get(key)
        if axis_dir is None:
            continue
        axis, direction = axis_dir
        speed = YAW_SPEED if axis == 'yaw' else CART_SPEED
        out[axis] += direction * speed * dt * speed_factor(keys)
    return out


def joint_step(keys, dt):
    """Return the angle in degrees to add to the selected joint."""
    move = sum(JOINT_KEYS[k] for k in keys if k in JOINT_KEYS)
    if not move:
        return 0.0
    # Several keys in the same direction are not faster than one.
    direction = 1 if move > 0 else -1
    return direction * JOINT_SPEED * dt * speed_factor(keys)


def gripper_step(keys, dt):
    """Return the change in gripper opening (0-1) for the keys held."""
    move = sum(GRIP_KEYS[k] for k in keys if k in GRIP_KEYS)
    if not move:
        return 0.0
    return (1 if move > 0 else -1) * GRIP_SPEED * dt


def picked_joint(key, joint_count):
    """Return the joint index a number key selects, or None."""
    if key not in PICK_KEYS:
        return None
    index = PICK_KEYS.index(key)
    return index if index < joint_count else None


def is_ours(key):
    """Say whether this key means anything to the driver."""
    return (key in CART_KEYS or key in JOINT_KEYS or key in GRIP_KEYS
            or key in PICK_KEYS or key in ('TAB',))


#: What to show the person, in the order it makes sense to read it.
HELP = (
    ('Cartesian', 'W / S  forward, back     A / D  left, right     '
                  'Q / E  up, down     J / L  yaw'),
    ('Joint', '1-7  pick a joint     ↑ / ↓  move it'),
    ('Both', 'Tab  swap space     O / C  gripper     '
             'Shift  faster     Ctrl  slower'),
)
