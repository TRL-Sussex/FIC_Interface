"""Decide whether the contact node is allowed to report an estimate.

Kept free of ROS, Qt and every other heavyweight dependency so the refusal
logic is testable without a built workspace or a live graph — see
``test/test_purity.py``, which asserts this module imports none of them.
"""

#: The four control modes (``ros_node.py:219``'s 1-indexed
#: ``{rid}_control_mode``) where the residual is trustworthy: the arm's
#: commanded motion does not depend on the contact torque being measured,
#: so the loop around ``tau_ext`` stays open. 1 and 5 hold the arm still
#: (Joint / Cartesian Impedance); 2 and 6 move it under a planner (Joint
#: Impedance With Planner / Cartesian Planner).
DEFAULT_ALLOWED_MODES = frozenset({1, 2, 5, 6})

#: The four admittance modes (3, 4, 7, 8) where the arm yields to a push
#: by design. The residual would be measured inside that closed loop —
#: the very torque driving the yield — so these stay refused regardless
#: of ``allowed_mode``, purely to make the refusal reason explain itself.
_ADMITTANCE_MODES = frozenset({3, 4, 7, 8})

#: Every control mode this gate can be configured to allow, now that the
#: admittance offset (see ``fic_contact.admittance``) gives modes 3, 4, 7
#: and 8 a meaningful estimate too. Pass this as ``allowed_mode`` to a
#: ``ContactGate`` to permit every mode. This does NOT change what a
#: bare ``ContactGate()`` refuses — that is still ``DEFAULT_ALLOWED_MODES``,
#: impedance-only, unchanged.
ALL_MODES = DEFAULT_ALLOWED_MODES | _ADMITTANCE_MODES


class ContactGate:
    """Track whether the node is currently allowed to report an estimate.

    Kept free of ROS so the refusal logic is testable without a graph.
    """

    def __init__(self, allowed_mode=DEFAULT_ALLOWED_MODES):
        """Configure the gate with the permitted control mode(s).

        ``allowed_mode`` is normalised to a frozenset: pass a single mode
        number (e.g. ``1``, kept for backward compatibility) or any
        iterable of mode numbers (the default, ``DEFAULT_ALLOWED_MODES``).
        """
        if isinstance(allowed_mode, int):
            self._allowed_modes = frozenset({allowed_mode})
        else:
            self._allowed_modes = frozenset(allowed_mode)
        self._mode = None
        self._feedback = False
        self._calibrated = False
        self._stale = False

    def set_mode(self, mode):
        """Record the robot's current control mode."""
        self._mode = mode

    def set_feedback(self, ok):
        """Record whether joint feedback has been received."""
        self._feedback = bool(ok)

    def set_calibrated(self, ok):
        """Record whether a bias calibration exists."""
        self._calibrated = bool(ok)

    def set_stale(self, stale):
        """Record whether the arm has left the calibration pose."""
        self._stale = bool(stale)

    def blocked_reason(self):
        """Return why estimation is blocked, or None when it is allowed."""
        if not self._feedback:
            return 'no joint feedback'
        if self._mode is None:
            # Distinct from a rejected mode: `{rid}_control_mode` is
            # VOLATILE and only published on change, so a node started
            # with the hardware stack never sees the mode already in
            # force. Selecting any allowed mode in the Studio clears it.
            return ('no control mode received yet -- select a mode in '
                    'FIC Studio (allowed: %s)'
                    % sorted(self._allowed_modes))
        if self._mode not in self._allowed_modes:
            allowed = sorted(self._allowed_modes)
            if self._mode in _ADMITTANCE_MODES:
                return ('mode %s is an admittance mode, where the arm '
                        'yields to contact instead of holding against it; '
                        'only impedance modes %s are allowed'
                        % (self._mode, allowed))
            return 'mode %s is not one of the allowed modes %s' % (
                self._mode, allowed)
        if not self._calibrated:
            return 'not calibrated'
        if self._stale:
            return 'calibration stale'
        return None
