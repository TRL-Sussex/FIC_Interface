# Estimated contact location on a manipulator, from joint torque residuals.
std_msgs/Header header

# 1-based link index, proximal to distal. 0 = no contact detected.
# -1 = not calibrated, so no statement is being made.
int32 link_index -1

# MuJoCo body name for link_index, or "" when link_index <= 0.
string link_name

# 0..1. Combines the deciding joint's margin above threshold with how
# quiet the joints distal to it are.
float64 confidence

# External joint torque, Nm, in the robot's configured joint order.
float64[] tau_ext

# v2 only, empty in v1. Estimated contact force (N) and point (m),
# both in the robot base frame.
float64[] force
float64[] point

# v2 only. Best-hypothesis fit residual, Nm.
float64 residual_norm

bool calibrated
