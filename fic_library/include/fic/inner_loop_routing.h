#ifndef FIC_INNER_LOOP_ROUTING_H
#define FIC_INNER_LOOP_ROUTING_H

// How each control mode reaches the motors through the joint inner loop: one table for the MCU, the
// PC inner-loop process and the simulation. Portable: no heap, no Eigen.
//
//   node mode            inner mode                   fields
//   idle, hard fault     Torque                       tau_des = node torque - g
//   1-4                  Impedance..AdmittanceHaptic  q_target, tau_ff = wall + zone torque
//   5-8 without H-PMP    Torque (pass-through)        tau_des = node torque - g (J^T w, postural, elbow,
//                                                     zone): the Cartesian modes as they are on the PC
//   5-8 with H-PMP       Hapfic + Jacobian            tau_des = H-PMP's tau_des - g, q_target = its limited
//                                                     and shifted set-point, tau_ff = node torque (postural,
//                                                     elbow, zone); the inner loop runs the split drive
//                                                     (no Jacobian yet: Hapfic, per-joint deadband)
// The inner loop adds gravity in every mode.

#include "fic_link.h"

namespace fic {
namespace link {

// What the task-space node has this tick, in its own arrays (n entries; jac 6 x n row-major, or null).
struct RouteInput {
    int mode = 0;                         // node control mode 0..8
    bool hard_fault = false;
    bool hpmp = false;                    // H-PMP active (modes 5-8)
    int n = 0;
    const double* torque = nullptr;       // the node's torque this tick, gravity included (torque_data)
    const double* gravity = nullptr;      // g(q) as sent in g0 (0 when the driver compensates gravity)
    const double* q_now = nullptr;
    const double* q_joint = nullptr;      // modes 1-4: the commanded joint target
    const double* q_task = nullptr;       // modes 5-8: the IK set-point (H-PMP: limited and shifted)
    const double* hpmp_tau_des = nullptr; // H-PMP's J^T w + g
    const double* wall = nullptr;         // joint modes: the Cartesian wall torque
    const double* zone = nullptr;         // safety-zone torque (every mode; in torque already for 5-8)
    const double* jac = nullptr;          // 6 x n, row-major (H-PMP)
    bool use_planner = false;             // modes 3/4: admittance through the planner
    double joint_target_speed = 0.0;      // modes 1/2: target speed limit (rad/s), 0 = off
};

// Fills mode, the planner and Jacobian flags, q_target, tau_des, tau_ff, jac and joint_target_speed.
// Leaves the node's own fields (epochs, enable / e-stop flags, gravity, kd, gripper) as they are.
inline void route(const RouteInput& in, CmdMsg& m) {
    const int n = in.n > kLinkMaxJoints ? kLinkMaxJoints : (in.n < 0 ? 0 : in.n);
    auto at = [](const double* a, int i) { return a ? a[i] : 0.0; };
    m.flags = static_cast<uint8_t>(m.flags & ~(kCmdUsePlanner | kCmdHasJacobian));
    m.joint_target_speed = 0.0f;
    for (int i = 0; i < n; ++i) {
        m.tau_des[i] = 0.0f;
        m.tau_ff[i] = 0.0f;
        m.q_target[i] = static_cast<float>(at(in.q_now, i));
    }
    if (in.hard_fault || in.mode <= 0 || in.mode > 8) {
        m.mode = InnerMode::Torque;
        for (int i = 0; i < n; ++i) m.tau_des[i] = static_cast<float>(at(in.torque, i) - at(in.gravity, i));
    } else if (in.mode <= 4) {
        static const InnerMode kJoint[5] = {InnerMode::Gravity, InnerMode::Impedance,
            InnerMode::ImpedancePlanner, InnerMode::Admittance, InnerMode::AdmittanceHaptic};
        m.mode = kJoint[in.mode];
        for (int i = 0; i < n; ++i) {
            m.q_target[i] = static_cast<float>(at(in.q_joint, i));
            m.tau_ff[i] = static_cast<float>(at(in.wall, i) + at(in.zone, i));
        }
        if (in.mode <= 2) m.joint_target_speed = static_cast<float>(in.joint_target_speed);
        if (in.mode >= 3 && in.use_planner) m.flags |= kCmdUsePlanner;
    } else if (in.hpmp) {
        // H-PMP: the node's torque holds only postural + elbow + zone (no J^T w, no gravity), so H-PMP always
        // goes to Hapfic. Without a Jacobian (solver not ready) the inner loop uses the per-joint deadband.
        m.mode = InnerMode::Hapfic;
        for (int i = 0; i < n; ++i) {
            m.q_target[i] = static_cast<float>(at(in.q_task, i));
            m.tau_des[i] = static_cast<float>(at(in.hpmp_tau_des, i) - at(in.gravity, i));
            m.tau_ff[i] = static_cast<float>(at(in.torque, i));
        }
        if (in.jac) {
            for (int k = 0; k < 6 * n; ++k) m.jac[k] = static_cast<float>(in.jac[k]);
            m.flags |= kCmdHasJacobian;
        }
    } else {
        m.mode = InnerMode::Torque;
        for (int i = 0; i < n; ++i) {
            m.q_target[i] = static_cast<float>(at(in.q_task, i));
            m.tau_des[i] = static_cast<float>(at(in.torque, i) - at(in.gravity, i));
        }
    }
}

// Stop / Pause on the inner-loop path: the node's idle ramp-down must start from the torque the arm last got
// (STATE tau_cmd, gravity included). On this path the node's own torque holds no joint law and, under H-PMP,
// no task torque or gravity: ramping from it would drop the arm to near zero torque at once. Returns false
// (out untouched) when STATE is older than max_age_s, is for another joint count, or the inner loop is not
// torquing.
inline bool rampStartFromState(const StateMsg& st, double age_s, int n, double* out, double max_age_s = 0.1) {
    if (age_s > max_age_s || st.n != n || n <= 0 || n > kLinkMaxJoints) return false;
    if (st.state != McuState::Running && st.state != McuState::Hold && st.state != McuState::Ramp) return false;
    for (int i = 0; i < n; ++i) out[i] = st.tau_cmd[i];
    return true;
}

}  // namespace link
}  // namespace fic

#endif  // FIC_INNER_LOOP_ROUTING_H
