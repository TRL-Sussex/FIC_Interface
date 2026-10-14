"""Teleoperation: one station's arm follows another's, across the lab network.

Built into the Multi-Robot page (it replaces the old teleop extension).

* ``session``  who leads whom: request / accept / stop, liveness, and what
  each Studio does for its part. Several pairs can run at once, and one
  leader can drive several followers.
* ``tracker``  the follower's maths: the leader's motion since the start,
  scaled, turned into joint targets that respect limits, a speed cap and
  smoothing.

The leader's Studio only streams its joint positions; it does not change the
leader's control mode (its operator picks a mode the arm can be moved in).
The follower's Studio holds its arm in Joint Impedance and streams targets on
its own ``robotN_desired_joint_angles``. Qt-free, tested without a network.
"""
