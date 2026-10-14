"""Fleet: every FIC Studio on the ROS network, seen and commanded from any one.

A lab runs one Studio per robot PC (station N drives robotN_* topics) and
maybe more on other machines. The Studios share a ROS domain, so each one
announces itself (``presence``) and listens for lab-wide commands such as
E-STOP ALL (``commands``). The Multi-Robot page is built on both, and so is
teleoperation (``fic_studio.teleop``).

Everything here is Qt-free and takes its ROS node and clock as arguments,
so it is tested without a network.
"""
