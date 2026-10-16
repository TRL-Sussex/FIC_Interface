// Joint target speed for the joint impedance modes (1, 2): the joint target the joint law sees moves towards the
// commanded one at no more than `speed` rad/s on each joint, so a target far from the arm (Home, a slider,
// a sequence step) is reached at that speed instead of the law's full effort at once. speed <= 0: off,
// the commanded target as it is. Used by fic_hardware and fic_simulation; set by the 4th value of
// robot{N}_safety/velocity_limits and, on the arm, safety_zones.velocity.joint_target_speed.
#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <vector>

namespace fic {

class JointTargetSlew {
public:
    double speed = 0.0;   // rad/s; <= 0: off

    // The target to give the joint law this tick. Starts from the measured joints when it engages and on
    // every change of mode (each joint mode snaps its commanded target to the arm on entry).
    const Eigen::VectorXd& apply(const Eigen::VectorXd& target, const Eigen::VectorXd& q_now, int mode,
                                 const std::vector<bool>& continuous, double dt) {
        if (speed <= 0.0 || target.size() == 0) {
            active_ = false;
            q_ = target;
            return q_;
        }
        if (!active_ || mode != mode_ || q_.size() != target.size()) {
            q_ = (q_now.size() == target.size()) ? q_now : target;
            mode_ = mode;
            active_ = true;
        }
        const double step = speed * dt;
        for (Eigen::Index i = 0; i < target.size(); ++i) {
            const bool cont = static_cast<size_t>(i) < continuous.size() && continuous[i];
            double d = target[i] - q_[i];
            if (cont) d = std::remainder(d, 2.0 * M_PI);                 // the short way round
            q_[i] += std::clamp(d, -step, step);
            if (cont) q_[i] = std::remainder(q_[i], 2.0 * M_PI);
        }
        return q_;
    }

    bool active() const { return active_; }

private:
    Eigen::VectorXd q_;
    int mode_ = -1;
    bool active_ = false;
};

}  // namespace fic
