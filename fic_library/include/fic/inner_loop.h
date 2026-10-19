#ifndef FIC_INNER_LOOP_H
#define FIC_INNER_LOOP_H

// The FIC inner loop: everything the MCU does between receiving the arm's
// feedback and sending it a torque, once per tick (1 kHz).
//
// Robot-agnostic and platform-agnostic — no Zephyr, no POSIX, no heap. The MCU
// firmware wraps it with a network stack, a timer and a vendor adapter; the
// Linux-side MCU simulator (fic_mcu_sim) wraps the very same class, so what is
// tested on a PC is what runs on the board.
//
// What it reproduces from fic_control, per tick:
//   * feedback filtering: torque 2nd-order Butterworth (200 Hz), position
//     2nd-order Butterworth (10 Hz, unwrapped for continuous joints), damping
//     velocity = first-order lag on the raw finite difference;
//   * the joint layer (fic_joint_core.h) for the commanded InnerMode;
//   * gravity: g(q) ≈ g0 + dg/dq·(q − q0), from the latest CMD — so no robot
//     model is needed on the MCU, and gravity stays right while Linux is late;
//   * the output guard: velocity damping, NaN → 0, per-joint clamp, and the
//     per-tick rate limit the Kinova driver applied;
//   * the joint-limit hard fault (gravity only until a new CONFIG).
//
// What it adds (the reason it lives on the MCU):
//   * HOLD after `hold_after_s` without a CMD — impedance to where the arm is;
//   * RAMP after `ramp_after_s`, on e-stop, or on StopTorque — torque decays
//     to gravity (0.995 per tick, like fic_control's ramp-down) and the
//     adapter then leaves torque mode, so the arm's own controller holds it.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "fic_joint_core.h"
#include "fic_link.h"

// Scalar type of the inner loop. double on Linux (fic_control, tests); the
// MCU firmware builds with -DFIC_INNER_REAL=float: the Cortex-M33 has a
// single-precision FPU only, and in double every operation is a library call
// (measured on the VENTUNO Q: ~1.1 ms per 7-joint tick in double, over the
// 1 ms budget).
#ifndef FIC_INNER_REAL
#define FIC_INNER_REAL double
#endif

namespace fic {
namespace mcu {

using Real = FIC_INNER_REAL;

using fic::link::Config;
using fic::link::CmdMsg;
using fic::link::InnerMode;
using fic::link::McuState;
using fic::link::Params;
using fic::link::StateMsg;
using fic::link::kLinkMaxJoints;

// One tick of arm feedback, as the vendor adapter reports it (raw, unfiltered).
struct ArmFeedback {
    bool valid = false;        // a fresh sample arrived this tick
    bool arm_fault = false;    // vendor says the arm is faulted
    float q[kLinkMaxJoints] = {};
    float tau[kLinkMaxJoints] = {};
    float wrench[6] = {};
    float gripper_pos = 0.0f;
    float gripper_current = 0.0f;
};

// A CONFIG made for one step run at another (a simulator's own): the step becomes `dt`, and the per-tick
// damping filter coefficient keeps its cutoff in Hz (alpha = exp(-2 pi f dt)).
inline void retimeConfig(fic::link::Config& c, float dt) {
    if (dt <= 0.0f) return;
    if (c.dt > 0.0f && c.damping_alpha > 0.0f && c.dt != dt)
        c.damping_alpha = static_cast<float>(std::pow(static_cast<double>(c.damping_alpha),
                                                      static_cast<double>(dt) / c.dt));
    c.dt = dt;
}

// What the adapter must do this tick.
struct ArmCommand {
    bool torque_mode = false;  // false: leave / stay out of torque mode (arm holds itself)
    float tau[kLinkMaxJoints] = {};
    float gripper_width = -1.0f;   // < 0: no gripper command
};

class InnerLoop {
public:
    static constexpr int kArmLinkLossTicks = 50;   // as the Kinova driver's error budget
    static constexpr Real kRampDecay = Real(0.995);    // fic_control ramp-down
    static constexpr Real kRampDoneNm = Real(0.05);

    // ------------------------------------------------------------------ setup
    // A new CONFIG resets everything: faults, e-stop latch, params, filters.
    void configure(const Config& c) {
        cfg_ = c;
        n_ = c.n_joints > kLinkMaxJoints ? kLinkMaxJoints : c.n_joints;
        core_.reset();
        core_.dt = c.dt;
        core_.deadband_joint_tau = c.deadband_joint_tau;
        core_.task_db_force = c.task_db_force;
        core_.task_db_torque = c.task_db_torque;
        core_.restart_on_stall = c.restart_on_stall != 0;
        core_.planner_max_hz = c.planner_max_hz;
        core_.planner_speed_margin = c.planner_speed_margin;
        core_.joint_speed_limit = c.joint_speed_limit;
        for (int i = 0; i < n_; ++i) {
            core_.setJointLimits(i, c.lower[i], c.upper[i]);
            core_.setJointContinuous(i, c.continuous[i] != 0);
        }
        state_ = McuState::Connecting;
        faults_ = 0;
        estop_latched_ = false;
        estop_direct_ = false;
        primed_ = false;
        have_cmd_ = false;
        active_epoch_ = 0;
        staged_epoch_ = 0;
        staged_mask_ = 0;
        params_applied_ = false;
        arm_bad_ticks_ = 0;
        arm_left_ = false;
        ramp_done_ = false;
        for (int i = 0; i < kLinkMaxJoints; ++i) { tau_prev_[i] = 0; tau_out_[i] = 0; }
        configured_ = true;
    }

    // PARAMS arrive one joint at a time; an epoch is applied (at the next tick,
    // with the then-current position) once every joint of it has arrived.
    bool acceptParams(const Params& p) {
        if (!configured_ || p.joint >= n_ || p.n_joints != n_) return false;
        if (p.param_epoch != staged_epoch_) {
            staged_epoch_ = p.param_epoch;
            staged_mask_ = 0;
        }
        staged_[p.joint] = p.p;
        staged_mask_ |= (1u << p.joint);
        return true;
    }

    // The adapter calls this once the arm streams feedback and accepts commands.
    void adapterReady(bool ready) {
        if (ready && state_ == McuState::Connecting) state_ = McuState::Ready;
        if (!ready && state_ != McuState::Idle && state_ != McuState::Fault) state_ = McuState::Connecting;
    }

    void onCmd(const CmdMsg& m, uint32_t seq, uint64_t now_us) {
        if (!configured_ || m.n != n_ || m.config_epoch != cfg_.config_epoch) return;
        // A CMD without the gravity slope (a slow link sends it only now and
        // then: it is 4*n*n bytes) keeps the last slope received — dg/dq moves
        // slowly, g0 / q0 are fresh every time.
        if (have_cmd_ && (cmd_.flags & fic::link::kCmdHasGravJac) &&
            !(m.flags & fic::link::kCmdHasGravJac)) {
            std::memcpy(static_cast<void*>(&cmd_), &m, offsetof(CmdMsg, dgdq));
            cmd_.flags |= fic::link::kCmdHasGravJac;
        } else {
            cmd_ = m;
        }
        cmd_seq_ = seq;
        last_cmd_us_ = now_us;
        have_cmd_ = true;
        if (m.joint_speed_limit != Real(core_.joint_speed_limit))
            core_.setJointSpeedLimit(m.joint_speed_limit);   // the operator's cap, live
        if (m.flags & fic::link::kCmdEstop) {
            estop();
        } else if (estop_latched_ && !estop_direct_ && !(m.flags & fic::link::kCmdEnable)) {
            // Re-arm: the operator cleared the e-stop and disabled torque.
            estop_latched_ = false;
            faults_ &= ~fic::link::kFaultEstop;
        }
    }

    // E-STOP: latched (RAMP from any torque state, never back to Running). From a CMD's flag, a later CMD
    // without Enable re-arms it (the operator cleared it). From a direct Estop message (FIC Studio), only a new
    // CONFIG (Start) does: fic_control's own Stop sends CMDs without Enable too.
    void estop(bool direct = false) {
        estop_latched_ = true;
        estop_direct_ = estop_direct_ || direct;
        faults_ |= fic::link::kFaultEstop;
    }

    // ------------------------------------------------------------------- tick
    void tick(const ArmFeedback& fb, uint64_t now_us, ArmCommand& out) {
        ++tick_;
        out.torque_mode = false;
        out.gripper_width = have_cmd_ ? cmd_.gripper_width : -1.0f;
        if (!configured_ || state_ == McuState::Idle || state_ == McuState::Connecting) return;

        // --- arm feedback health
        if (!fb.valid) {
            // Out of torque (Ready: after a finished ramp, or before any torque) the arm leaving is what a Stop
            // does (the Kinova driver exits once it is back in position mode): no fault, but no torque again
            // until a new CONFIG. While torquing it is a lost arm: fault.
            if (++arm_bad_ticks_ > kArmLinkLossTicks) {
                if (state_ == McuState::Ready) arm_left_ = true;
                else enterFault(fic::link::kFaultArmLink);
            }
        } else {
            arm_bad_ticks_ = 0;
        }
        if (fb.arm_fault) enterFault(fic::link::kFaultArm);
        if (fb.valid) updateFeedback(fb);
        if (!primed_) return;                   // nothing known about the arm yet
        if (faults_ & (fic::link::kFaultArm | fic::link::kFaultArmLink)) {
            // Stop commanding: the adapter drops out of torque mode and the
            // vendor's own watchdog / brakes take over.
            state_ = McuState::Fault;
            return;
        }

        // --- stage params once complete
        const uint32_t full = (n_ >= 32) ? 0xFFFFFFFFu : ((1u << n_) - 1u);
        if (staged_mask_ == full && staged_epoch_ != active_epoch_) applyParams();

        // --- joint-limit hard fault (fic_control: gravity only until reload)
        if (!(faults_ & fic::link::kFaultJointLimit)) {
            for (int i = 0; i < n_; ++i) {
                if (q_[i] < cfg_.fault_lower[i] || q_[i] > cfg_.fault_upper[i]) {
                    faults_ |= fic::link::kFaultJointLimit;
                    break;
                }
            }
        }

        // Integer microseconds: exact, and no double arithmetic on the MCU.
        const uint64_t age_us = have_cmd_ ? now_us - last_cmd_us_ : UINT64_MAX;
        const Real age_s = have_cmd_ ? Real(age_us) * Real(1e-6) : Real(1e9);
        cmd_age_ms_ = age_us > 65000000ull ? 65535 : (uint16_t)(age_us / 1000);
        const bool enable = have_cmd_ && (cmd_.flags & fic::link::kCmdEnable);
        const bool stop_req = have_cmd_ && (cmd_.flags & fic::link::kCmdStopTorque);

        // --- state transitions
        // Fault is terminal until the next CONFIG: the arm holds itself.
        if (state_ == McuState::Fault) return;
        const bool torque_active = state_ == McuState::Running || state_ == McuState::Hold ||
                                   state_ == McuState::Ramp;
        if (torque_active) {
            if (estop_latched_ || stop_req || age_s > cfg_.ramp_after_s || !enable) {
                if (state_ != McuState::Ramp) {
                    state_ = McuState::Ramp;
                    ramp_done_ = false;
                    // Decay from the torque the arm last got; the guard's damping
                    // is added on top every tick, never folded back in here.
                    for (int i = 0; i < n_; ++i) ramp_diff_[i] = tau_prev_[i] - gravityAt(i);
                }
            } else if (state_ == McuState::Ramp) {
                // A ramp runs to completion; the next Running entry starts clean.
            } else if (age_s > cfg_.hold_after_s) {
                if (state_ != McuState::Hold) {
                    state_ = McuState::Hold;
                    for (int i = 0; i < n_; ++i) q_hold_[i] = q_[i];
                }
            } else if (state_ == McuState::Hold) {
                state_ = McuState::Running;
            }
        } else if (state_ == McuState::Ready) {
            if (enable && !estop_latched_ && !arm_left_ && !stop_req && age_s <= cfg_.hold_after_s &&
                params_applied_ && cmd_.param_epoch == active_epoch_) {
                // Entering torque: re-seed planners / admittance at the current
                // pose, exactly like a fic_control mode switch.
                reinitCore();
                for (int i = 0; i < n_; ++i) tau_prev_[i] = gravityAt(i);
                ramp_done_ = false;
                state_ = McuState::Running;
            }
        }

        // --- torque
        Real tau[kLinkMaxJoints] = {};
        switch (state_) {
        case McuState::Running:
            if (faults_ & fic::link::kFaultJointLimit) {
                for (int i = 0; i < n_; ++i) tau[i] = gravityAt(i);
            } else if (cmd_.param_epoch != active_epoch_) {
                holdLaw(tau, q_);                  // params in flight: hold here
            } else {
                runLaw(tau);
            }
            break;
        case McuState::Hold:
            holdLaw(tau, q_hold_);
            break;
        case McuState::Ramp: {
            bool converged = true;
            for (int i = 0; i < n_; ++i) {
                ramp_diff_[i] *= kRampDecay;
                if (std::fabs(ramp_diff_[i]) > kRampDoneNm) converged = false;
                else ramp_diff_[i] = Real(0);
                tau[i] = gravityAt(i) + ramp_diff_[i];
            }
            if (converged) {
                ramp_done_ = true;
                state_ = (faults_ & fic::link::kFaultJointLimit) ? McuState::Fault : McuState::Ready;
                return;                               // torque_mode stays false
            }
            break;
        }
        default:
            return;                                   // Ready / Fault: arm holds itself
        }

        for (int i = 0; i < n_; ++i) tau_law_[i] = tau[i];
        guardOutput(tau);
        out.torque_mode = true;
        for (int i = 0; i < n_; ++i) out.tau[i] = (float)tau[i];
    }

    // ------------------------------------------------------------------ state
    void fillState(StateMsg& s, uint16_t jitter_us_max) const {
        s.ack_seq = cmd_seq_;
        s.tick = tick_;
        s.config_epoch = cfg_.config_epoch;
        s.param_epoch = active_epoch_;
        s.faults = faults_;
        s.jitter_us_max = jitter_us_max;
        s.cmd_age_ms = cmd_age_ms_;
        s.state = state_;
        s.mode = have_cmd_ ? cmd_.mode : InnerMode::Gravity;
        s.n = (uint16_t)n_;
        s.dt = cfg_.dt;
        for (int i = 0; i < n_; ++i) {
            s.q_raw[i] = (float)raw_q_prev_[i];
            s.tau_raw[i] = (float)tau_raw_[i];
            s.q[i] = (float)q_[i];
            s.tau_meas[i] = (float)tau_meas_[i];
            s.tau_cmd[i] = (float)tau_out_[i];
            s.tau_law[i] = (float)tau_law_[i];
            s.g_used[i] = (float)gravityAt(i);
            s.adm_offset[i] = (float)core_.humanOffset(i);
            s.q_target_adm[i] = (float)diag_[i].q_target_adm;
        }
        for (int k = 0; k < 6; ++k) s.wrench[k] = wrench_[k];
        s.gripper_pos = gripper_pos_;
        s.gripper_current = gripper_current_;
    }

    McuState state() const { return state_; }
    // Torque is over for this session (a ramp ran to completion, or a fault):
    // the adapter should hand the arm back to its own controller.
    bool released() const { return ramp_done_ || state_ == McuState::Fault; }
    // fic_control is commanding torque right now (fresh, enabled CMD, no e-stop):
    // after a release, the adapter opens a new session with the arm.
    bool wantsTorque(uint64_t now_us) const {
        return have_cmd_ && (cmd_.flags & fic::link::kCmdEnable) && !(cmd_.flags & fic::link::kCmdStopTorque) &&
               !estop_latched_ && Real(now_us - last_cmd_us_) * Real(1e-6) <= cfg_.hold_after_s;
    }
    uint32_t faults() const { return faults_; }
    int joints() const { return n_; }
    // The target the joint law saw last tick in modes 1/2 (after the joint-target speed limit).
    Real slewedTarget(int i) const { return q_slew_[i]; }
    // The joint planners' speed cap in force (rad/s, 0: none).
    Real jointSpeedLimit() const { return core_.joint_speed_limit; }
    // Gravity at the current pose (g0 + dg/dq (q - q0) from the latest CMD; 0 before any): what the arm needs
    // when the inner loop no longer torques it but a driver stays in torque mode.
    Real gravity(int i) const { return gravityAt(i); }
    // The last tick ran H-PMP's split drive (Hapfic with the Jacobian).
    bool usedSplit() const { return used_split_; }
    const Config& config() const { return cfg_; }

private:
    Config cfg_{};
    bool configured_ = false;
    int n_ = 0;
    McuState state_ = McuState::Idle;
    uint32_t faults_ = 0;
    bool estop_latched_ = false;
    bool estop_direct_ = false;     // the latch came from a direct Estop: only a CONFIG clears it
    bool arm_left_ = false;         // the arm/driver went away out of torque: only a CONFIG torques again
    bool ramp_done_ = false;
    uint32_t tick_ = 0;
    int arm_bad_ticks_ = 0;

    fic::core::JointCore<Real> core_;
    fic::core::JointParamsPOD staged_[kLinkMaxJoints];
    fic::core::JointParamsPOD active_[kLinkMaxJoints];
    uint32_t staged_epoch_ = 0, active_epoch_ = 0;
    uint32_t staged_mask_ = 0;
    bool params_applied_ = false;

    CmdMsg cmd_{};
    uint32_t cmd_seq_ = 0;
    uint64_t last_cmd_us_ = 0;
    bool have_cmd_ = false;
    uint16_t cmd_age_ms_ = 0xFFFF;

    // Filtered feedback
    bool primed_ = false;
    fic::core::Butterworth2<Real> tf_[kLinkMaxJoints], pf_[kLinkMaxJoints];
    Real raw_q_prev_[kLinkMaxJoints] = {};
    Real unwrapped_[kLinkMaxJoints] = {};
    Real q_[kLinkMaxJoints] = {};
    Real qd_[kLinkMaxJoints] = {};
    Real v_damp_[kLinkMaxJoints] = {};
    Real tau_meas_[kLinkMaxJoints] = {};
    Real tau_raw_[kLinkMaxJoints] = {};
    Real q_hold_[kLinkMaxJoints] = {};
    float wrench_[6] = {};
    float gripper_pos_ = 0, gripper_current_ = 0;

    Real tau_prev_[kLinkMaxJoints] = {};
    Real ramp_diff_[kLinkMaxJoints] = {};   // RAMP: offset from gravity, decays 0.995 per tick
    Real tau_out_[kLinkMaxJoints] = {};
    Real tau_law_[kLinkMaxJoints] = {};     // the last torque before guardOutput
    fic::core::JointDiagT<Real> diag_[kLinkMaxJoints];

    // Modes 1/2: the target the law sees, moving at most joint_target_speed per tick.
    Real q_slew_[kLinkMaxJoints] = {};
    bool slew_active_ = false;
    InnerMode slew_mode_ = InnerMode::Gravity;
    bool used_split_ = false;

    static Real wrapPi(Real x) { return fic::core::wrapToPi(x); }

    Real jointDelta(int i, Real a, Real b) const {
        return cfg_.continuous[i] ? wrapPi(a - b) : (a - b);
    }

    void enterFault(uint32_t bit) {
        faults_ |= bit;
    }

    void updateFeedback(const ArmFeedback& fb) {
        const Real dt = cfg_.dt > 0 ? Real(cfg_.dt) : Real(0.001);
        const double fs = 1.0 / double(dt);
        if (!primed_) {
            for (int i = 0; i < n_; ++i) {
                tf_[i] = fic::core::Butterworth2<Real>(cfg_.torque_cutoff_hz, fs, fb.tau[i]);
                pf_[i] = fic::core::Butterworth2<Real>(cfg_.state_cutoff_hz, fs, fb.q[i]);
                unwrapped_[i] = fb.q[i];
                raw_q_prev_[i] = fb.q[i];
                q_[i] = fb.q[i];
                qd_[i] = 0;
                v_damp_[i] = 0;
            }
            primed_ = true;
        }
        for (int i = 0; i < n_; ++i) {
            const Real raw_q = fb.q[i];
            tau_raw_[i] = fb.tau[i];
            tau_meas_[i] = cfg_.torque_sign[i] * tf_[i].filter(fb.tau[i]);
            Real q_new;
            if (cfg_.continuous[i]) {
                unwrapped_[i] += wrapPi(raw_q - raw_q_prev_[i]);
                q_new = wrapPi(pf_[i].filter(unwrapped_[i]));
            } else {
                q_new = pf_[i].filter(raw_q);
            }
            qd_[i] = jointDelta(i, q_new, q_[i]) / dt;
            q_[i] = q_new;
            const Real v_raw = jointDelta(i, raw_q, raw_q_prev_[i]) / dt;
            const Real a = cfg_.damping_alpha;
            v_damp_[i] = (a <= Real(0)) ? v_raw : a * v_damp_[i] + (Real(1) - a) * v_raw;
            raw_q_prev_[i] = raw_q;
        }
        for (int k = 0; k < 6; ++k) wrench_[k] = fb.wrench[k];
        gripper_pos_ = fb.gripper_pos;
        gripper_current_ = fb.gripper_current;
    }

    // g(q) ≈ g0 + dg/dq (q - q0), from the latest CMD. Zero before any CMD —
    // Ready never enters torque mode without one.
    Real gravityAt(int i) const {
        if (!have_cmd_) return Real(0);
        Real g = cmd_.g0[i];
        if (cmd_.flags & fic::link::kCmdHasGravJac) {
            for (int j = 0; j < n_; ++j)
                g += cmd_.dgdq[i * n_ + j] * jointDelta(j, q_[j], cmd_.q0[j]);
        }
        return g;
    }

    void applyParams() {
        for (int i = 0; i < n_; ++i) active_[i] = staged_[i];
        active_epoch_ = staged_epoch_;
        reinitCore();
    }

    void reinitCore() {
        bool ok = true;
        for (int i = 0; i < n_; ++i) ok &= core_.initJoint(i, active_[i], q_[i]);
        core_.resetSplit();
        slew_active_ = false;
        params_applied_ = ok && active_epoch_ != 0;
        if (!ok) faults_ |= fic::link::kFaultBadParams;
    }

    void runLaw(Real* tau) {
        const bool planner = cmd_.flags & fic::link::kCmdUsePlanner;
        // Modes 1/2: the target the law sees moves at most joint_target_speed per tick (fic_control's
        // JointTargetSlew), seeded at the measured joints on entry and on a mode change.
        const bool slew = cmd_.joint_target_speed > 0.0f &&
            (cmd_.mode == InnerMode::Impedance || cmd_.mode == InnerMode::ImpedancePlanner);
        if (!slew) {
            slew_active_ = false;
            for (int i = 0; i < n_; ++i) q_slew_[i] = cmd_.q_target[i];
        } else {
            if (!slew_active_ || slew_mode_ != cmd_.mode) {
                for (int i = 0; i < n_; ++i) q_slew_[i] = q_[i];
                slew_active_ = true;
                slew_mode_ = cmd_.mode;
            }
            const Real step = Real(cmd_.joint_target_speed) * Real(cfg_.dt > 0 ? cfg_.dt : 0.001f);
            for (int i = 0; i < n_; ++i) {
                const Real dq = jointDelta(i, Real(cmd_.q_target[i]), q_slew_[i]);
                q_slew_[i] += fic::core::clampv(dq, -step, step);
                if (cfg_.continuous[i]) q_slew_[i] = wrapPi(q_slew_[i]);
            }
        }
        // H-PMP: the split drive from the Jacobian (without it each joint deadbands its own gap).
        Real drive[kLinkMaxJoints];
        used_split_ = cmd_.mode == InnerMode::Hapfic && (cmd_.flags & fic::link::kCmdHasJacobian);
        if (used_split_) {
            Real u[kLinkMaxJoints];
            for (int i = 0; i < n_; ++i) u[i] = Real(cmd_.tau_des[i]) + gravityAt(i) + tau_meas_[i];
            core_.splitDrive(cmd_.jac, u, n_, drive);
        }
        for (int i = 0; i < n_; ++i) {
            fic::core::JointDiagT<Real>& d = diag_[i];
            d = fic::core::JointDiagT<Real>{};
            const Real g = gravityAt(i);
            Real qt = slew ? q_slew_[i] : Real(cmd_.q_target[i]);
            switch (cmd_.mode) {
            case InnerMode::Gravity:
                tau[i] = g;
                break;
            case InnerMode::Impedance:
                tau[i] = core_.impedance(i, qt, q_[i], d) + g;
                break;
            case InnerMode::ImpedancePlanner:
                tau[i] = core_.impedanceWithPlanner(i, qt, q_[i], d) + g;
                break;
            // Modes 3/4 and hapfic: the admittance is driven by the interaction torque
            // alone, so gravity is fed forward as in modes 1 and 2 (as fic_control).
            case InnerMode::Admittance:          // fic_control mode 3
                tau[i] = core_.admittance(i, qt, q_[i], g, -tau_meas_[i], 1, planner, d) + g;
                break;
            case InnerMode::AdmittanceHaptic:    // fic_control mode 4
                tau[i] = core_.admittance(i, qt, q_[i], g, -tau_meas_[i], 2, planner, d) + g;
                break;
            case InnerMode::Hapfic:              // H-PMP's joint layer (fic_control.h cartesianHpmp)
                // Measured torque in the joint-admittance sign, as modes 3/4 and
                // cartesianHpmp (-joint_torques_); +tau_meas_ was a positive feedback
                // that drove the offsets to their bound.
                tau[i] = used_split_
                    ? core_.admittance(i, qt, q_[i], cmd_.tau_des[i] + g, -tau_meas_[i], 2, true, d,
                                       Real(0.25) * core_.deadband_joint_tau, drive[i]) + g
                    : core_.admittance(i, qt, q_[i], cmd_.tau_des[i] + g, -tau_meas_[i], 2, true, d,
                                       Real(0.25) * core_.deadband_joint_tau) + g;   // per-joint hysteresis
                break;
            case InnerMode::Torque:
                tau[i] = cmd_.tau_des[i] + g;
                break;
            }
            tau[i] += cmd_.tau_ff[i];
        }
    }

    void holdLaw(Real* tau, const Real* q_ref) {
        for (int i = 0; i < n_; ++i) {
            fic::core::JointDiagT<Real> d;
            tau[i] = core_.impedance(i, q_ref[i], q_[i], d) + gravityAt(i);
        }
    }

    // fic_control's output guard + the Kinova driver's rate limit.
    void guardOutput(Real* tau) {
        for (int i = 0; i < n_; ++i) {
            Real t = tau[i];
            if (have_cmd_ && cmd_.kd[i] > 0.0f) t -= cmd_.kd[i] * v_damp_[i];
            if (!std::isfinite(t)) t = Real(0);
            const Real lim = cfg_.torque_limit[i];
            if (lim > 0) t = fic::core::clampv(t, -lim, lim);
            const Real rate = cfg_.torque_rate_limit[i];
            if (rate > 0) t = fic::core::clampv(t, tau_prev_[i] - rate, tau_prev_[i] + rate);
            tau[i] = t;
            tau_prev_[i] = t;
            tau_out_[i] = t;
        }
    }
};

} // namespace mcu
} // namespace fic

#endif // FIC_INNER_LOOP_H
