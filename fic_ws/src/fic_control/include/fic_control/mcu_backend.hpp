// mcu_backend.hpp — fic_control's side of the FIC inner-loop MCU.
//
// fic_control runs every robot the same way: it decides per robot, at load and
// again at "Start", whether an MCU is present that can drive it.
//
//   robots_config.yaml robot has an `mcu:` block with an adapter
//     AND an MCU answers HELLO at `mcu_address`
//     AND that MCU reports the adapter           -> MCU path
//   anything else                                -> standard path (vendor
//                                                   driver + shared memory),
//                                                   exactly as before
//
// On the MCU path this class:
//   * sends CONFIG / PARAMS and re-sends them until STATE acknowledges them
//     (UDP may drop a datagram; nothing else retries);
//   * turns the node's control mode + outer-loop output into one CMD per tick;
//   * mirrors the MCU's raw arm feedback into the SAME SharedFeedback fields a
//     vendor driver would write, so readSharedFeedback(), the gripper state,
//     the API wrench and every other consumer work unchanged.
#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include <fic/fic_link.h>
#include <fic/inner_loop_routing.h>
#include <fic/joint_params_pod.h>
#include <fic/mcu_link.hpp>
#include <fic/robot_config_loader.hpp>
#include <fic/shared_params.h>

namespace fic_hw {

class McuBackend {
public:
    using Clock = std::chrono::steady_clock;

    struct Detection {
        bool usable = false;       // MCU found and it can drive this robot
        bool found = false;        // something answered HELLO
        std::string detail;        // human-readable reason / firmware id
    };

    // Everything CONFIG needs that fic_control computes at robot load.
    struct RobotLimits {
        size_t n = 0;
        std::vector<double> lower, upper;            // controller clamp (rad)
        std::vector<double> fault_lower, fault_upper;// hard fault (rad), may be empty
        std::vector<bool> continuous;
        std::vector<double> torque_limit;            // Nm
        double deadband_joint_tau = 1.5;
        double torque_sign = 1.0;
        double damping_alpha = 0.0;
        double torque_cutoff_hz = 200.0;
        double state_cutoff_hz = 10.0;
        double dt = 0.001;
        double joint_speed_limit = 0.0;              // joint planners' speed cap (rad/s), 0: none
        double task_db_force = 1.0;                  // H-PMP's split drive: end-effector deadbands (N, N m)
        double task_db_torque = 0.1;
    };

    // The MCU board's address (mcu_address). The link itself may point at the PC inner loop instead.
    void setAddress(const std::string& a) { board_address_ = a; }
    // A simulated arm: its inner loop is fic_inner_loop --adapter sim at this address, and nothing else is
    // tried (FIC_INNER_LOOP_ADDRESS overrides it, as for the arm).
    void setSimAddress(const std::string& a) {
        const char* e = std::getenv("FIC_INNER_LOOP_ADDRESS");
        sim_address_ = (e && *e) ? std::string(e) : a;
    }
    bool simulated() const { return !sim_address_.empty(); }
    // The topic plant: this node feeds the simulated arm's samples to the inner loop and reads its torques.
    bool sendSimStep(const fic::link::SimStepMsg& m) { return link_.sendSimStep(m); }
    bool latestSimTorque(fic::link::SimTorqueMsg& t, double& age) const { return link_.latestSimTorque(t, age); }
    const std::string& address() const { return address_; }

    // The PC inner loop (fic_inner_loop) on this machine: FIC_INNER_LOOP_ADDRESS, or 127.0.0.1:7360.
    static std::string localAddress() {
        const char* e = std::getenv("FIC_INNER_LOOP_ADDRESS");
        return (e && *e) ? std::string(e) : std::string("127.0.0.1:7360");
    }

    // Which inner loop drives this robot: an MCU that can (a robot with an `mcu:` adapter, at mcu_address),
    // otherwise the PC inner loop on its shared memory. Cheap to call again.
    Detection detect(const fic::RobotConfig& cfg) {
        if (simulated()) {
            Detection d = probeAt(sim_address_, fic::link::Adapter::HostSim, "sim", cfg);
            if (d.usable) local_ = true;
            else d.detail = "no inner loop: " + d.detail;
            return d;
        }
        std::string board_why;
        if (!cfg.mcu.adapter.empty()) {
            const auto want = fic::link::adapterFromName(cfg.mcu.adapter.c_str());
            if (want == fic::link::Adapter::None) {
                board_why = "unknown mcu adapter '" + cfg.mcu.adapter + "'";
            } else if (board_address_.empty()) {
                board_why = "mcu_address is empty (MCU detection disabled)";
            } else {
                Detection d = probeAt(board_address_, want, cfg.mcu.adapter, cfg);
                if (d.usable) { local_ = false; return d; }
                board_why = d.detail;
            }
        }
        Detection l = probeAt(localAddress(), fic::link::Adapter::Shm, "shm", cfg);
        if (l.usable) {
            local_ = true;
            if (!board_why.empty()) l.detail += " (MCU: " + board_why + ")";
            return l;
        }
        l.detail = "no inner loop: " + l.detail + (board_why.empty() ? "" : "; MCU: " + board_why);
        return l;
    }

    // Driving through the PC inner loop (fic_inner_loop), not an MCU board.
    bool local() const { return local_; }

private:
    Detection probeAt(const std::string& address, fic::link::Adapter want, const std::string& want_name,
                      const fic::RobotConfig& cfg) {
        Detection d;
        if (address != address_) { link_.close(); address_ = address; }
        if (!link_.isOpen() && !link_.open(address_)) {
            d.detail = "cannot open " + address_ + ": " + link_.lastError();
            return d;
        }
        auto ack = link_.probe();
        if (!ack) { d.detail = "nothing answered at " + link_.address(); return d; }
        d.found = true;
        hello_ = *ack;
        char buf[160];
        std::snprintf(buf, sizeof(buf), "%s v%u at %s", ack->fw_name, ack->fw_version,
                      link_.address().c_str());
        fw_ = buf;
        if (!(ack->adapters & fic::link::adapterBit(want))) {
            d.detail = fw_ + " does not support adapter '" + want_name + "'";
            return d;
        }
        if (ack->max_joints < cfg.num_joints) {
            d.detail = fw_ + " supports only " + std::to_string(ack->max_joints) + " joints";
            return d;
        }
        // A PC inner loop already torquing an arm that is not ours (another station on this PC, or a board-
        // less fallback while another robot runs): never take it over — our CONFIG would reset it mid-motion.
        // A simulated station's inner loop (fic::link::simInnerLoopPort) is the station's own: a robot reload keeps driving it.
        const auto st = static_cast<fic::link::McuState>(ack->state);
        const bool busy = st == fic::link::McuState::Running || st == fic::link::McuState::Hold ||
                          st == fic::link::McuState::Ramp;
        if (want == fic::link::Adapter::Shm && busy && !(have_config_ && local_)) {
            d.detail = fw_ + " is driving another arm (" + std::string(stateName(st)) + ")";
            return d;
        }
        d.usable = true;
        d.detail = fw_ + ", adapter '" + want_name + "'";
        return d;
    }

public:

    // Build and send CONFIG. Starts the MCU's adapter (it connects to the arm).
    bool configure(const fic::RobotConfig& cfg, const RobotLimits& lim) {
        using fic::link::kLinkMaxJoints;
        if (lim.n == 0 || lim.n > static_cast<size_t>(kLinkMaxJoints)) return false;
        fic::link::Config c{};
        c.config_epoch = ++config_epoch_;
        c.adapter = static_cast<uint32_t>(simulated() ? fic::link::Adapter::HostSim
                                          : local_ ? fic::link::Adapter::Shm
                                                   : fic::link::adapterFromName(cfg.mcu.adapter.c_str()));
        c.n_joints = static_cast<uint16_t>(lim.n);
        c.arm_port = static_cast<uint16_t>(cfg.mcu.arm_port);
        parseIp(cfg.mcu.arm_ip, c.arm_ip);
        c.hold_after_s = static_cast<float>(cfg.mcu.hold_after_s);
        c.ramp_after_s = static_cast<float>(cfg.mcu.ramp_after_s);
        c.deadband_joint_tau = static_cast<float>(lim.deadband_joint_tau);
        c.dt = static_cast<float>(lim.dt);
        c.torque_cutoff_hz = static_cast<float>(lim.torque_cutoff_hz);
        c.state_cutoff_hz = static_cast<float>(lim.state_cutoff_hz);
        c.damping_alpha = static_cast<float>(lim.damping_alpha);
        c.restart_on_stall = cfg.restart_on_stall ? 1 : 0;
        c.planner_max_hz = static_cast<float>(cfg.planner_max_hz);
        c.planner_speed_margin = static_cast<float>(cfg.planner_speed_margin);
        c.joint_speed_limit = static_cast<float>(lim.joint_speed_limit);
        c.task_db_force = static_cast<float>(lim.task_db_force);
        c.task_db_torque = static_cast<float>(lim.task_db_torque);
        std::strncpy(c.robot_name, cfg.robot_id.c_str(), sizeof(c.robot_name) - 1);
        const float inf = std::numeric_limits<float>::infinity();
        for (size_t i = 0; i < lim.n; ++i) {
            c.lower[i] = static_cast<float>(at(lim.lower, i, -M_PI));
            c.upper[i] = static_cast<float>(at(lim.upper, i, M_PI));
            c.fault_lower[i] = i < lim.fault_lower.size() ? static_cast<float>(lim.fault_lower[i]) : -inf;
            c.fault_upper[i] = i < lim.fault_upper.size() ? static_cast<float>(lim.fault_upper[i]) : inf;
            c.torque_limit[i] = static_cast<float>(at(lim.torque_limit, i, 0.0));
            c.torque_rate_limit[i] = static_cast<float>(at(cfg.mcu.torque_rate_limit, i, 0.0));
            c.torque_sign[i] = static_cast<float>(lim.torque_sign);
            c.continuous[i] = (i < lim.continuous.size() && lim.continuous[i]) ? 1 : 0;
        }
        config_ = c;
        n_ = lim.n;
        have_config_ = true;
        param_epoch_ = 0;
        params_.clear();
        last_config_tx_ = Clock::now();
        return link_.sendConfig(config_);
    }

    // Ship one epoch of joint parameters (whatever fic_control just loaded for
    // the active mode). Returns false if any joint's params do not fit the MCU.
    bool sendParams(const std::vector<fic::ScalarParams>& s, const std::vector<fic::ListParams>& l,
                    const std::vector<fic::PlannerParams>& p, const std::vector<fic::AdmittanceParams>& a) {
        if (!have_config_) return false;
        const size_t n = n_;
        if (s.size() < n || l.size() < n || p.size() < n || a.size() < n) return false;
        std::vector<fic::link::Params> out(n);
        bool ok = true;
        ++param_epoch_;
        for (size_t i = 0; i < n; ++i) {
            out[i].param_epoch = param_epoch_;
            out[i].joint = static_cast<uint16_t>(i);
            out[i].n_joints = static_cast<uint16_t>(n);
            ok &= fic::core::toPOD(s[i], l[i], p[i], a[i], out[i].p);
        }
        params_ = std::move(out);
        sendAllParams();
        return ok;
    }

    // Once per control tick: re-send CONFIG / PARAMS the MCU has not confirmed.
    void service() {
        if (!have_config_) return;
        fic::link::StateMsg st;
        double age = 0;
        const bool have = link_.latestState(st, age) && age < 0.5;
        const auto now = Clock::now();
        if (now - last_config_tx_ < std::chrono::milliseconds(100)) return;
        if (!have || st.config_epoch != config_.config_epoch) {
            link_.sendConfig(config_);
            last_config_tx_ = now;
        } else if (!params_.empty() && st.param_epoch != param_epoch_) {
            sendAllParams();
            last_config_tx_ = now;
        }
    }

    bool sendCmd(fic::link::CmdMsg& m) {
        if (!have_config_) return false;
        m.config_epoch = config_.config_epoch;
        m.param_epoch = param_epoch_;
        m.n = static_cast<uint16_t>(n_);
        if (link_.sendCmd(m)) return true;
        // Too large for one datagram (dg/dq and the Jacobian, many joints): send this tick without dg/dq;
        // the inner loop keeps the last slope it received.
        if ((m.flags & fic::link::kCmdHasGravJac) && (m.flags & fic::link::kCmdHasJacobian)) {
            m.flags = static_cast<uint8_t>(m.flags & ~fic::link::kCmdHasGravJac);
            return link_.sendCmd(m);
        }
        return false;
    }

    // Copy the MCU's raw arm feedback into shared memory, where the standard
    // path's vendor driver would have put it. Returns false if STATE is older
    // than max_age_s (the shared memory is then left untouched).
    bool mirrorFeedback(SharedFeedback* shm, double max_age_s) {
        if (!shm) return false;
        double age = 0;
        if (!link_.latestState(state_, age)) return false;
        state_age_ = age;
        if (age > max_age_s || state_.n != n_) return false;
        const size_t lim = std::min<size_t>(n_, kMaxJoints);
        for (size_t i = 0; i < lim; ++i) {
            shm->joint_positions[i].store(state_.q_raw[i], std::memory_order_release);
            shm->torque[i].store(state_.tau_raw[i], std::memory_order_release);
        }
        for (int k = 0; k < 6; ++k) shm->tool_wrench[k].store(state_.wrench[k], std::memory_order_release);
        if (n_ < kGripperFeedbackSlot) {
            shm->joint_positions[kGripperFeedbackSlot].store(state_.gripper_pos, std::memory_order_release);
            shm->torque[kGripperFeedbackSlot].store(state_.gripper_current, std::memory_order_release);
        }
        std::atomic_thread_fence(std::memory_order_release);
        return true;
    }

    const fic::link::StateMsg& lastState() const { return state_; }
    double lastStateAge() const { return state_age_; }
    uint32_t paramEpoch() const { return param_epoch_; }
    bool configured() const { return have_config_; }

    void reset() {
        have_config_ = false;
        params_.clear();
        param_epoch_ = 0;
        n_ = 0;
    }

    static const char* stateName(fic::link::McuState s) {
        switch (s) {
        case fic::link::McuState::Idle: return "idle";
        case fic::link::McuState::Connecting: return "connecting";
        case fic::link::McuState::Ready: return "ready";
        case fic::link::McuState::Running: return "running";
        case fic::link::McuState::Hold: return "HOLD";
        case fic::link::McuState::Ramp: return "RAMP";
        case fic::link::McuState::Fault: return "FAULT";
        }
        return "?";
    }

    // One line for the {rid}_inner_loop status topic.
    std::string status() const {
        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "%s %s | %s | state %.0f Hz, age %.0f ms | tick jitter %u us | cmd age %u ms | faults 0x%X",
                      local_ ? "inner loop" : "mcu", fw_.c_str(), stateName(state_.state), link_.stateRateHz(),
                      state_age_ * 1e3,
                      state_.jitter_us_max, state_.cmd_age_ms, state_.faults);
        return buf;
    }

private:
    fic::McuLink link_;
    std::string address_;          // where the link points now
    std::string board_address_;    // the MCU board (mcu_address)
    bool local_ = false;           // the link is the PC inner loop
    std::string sim_address_;      // a simulated arm's inner loop (target simulation)
    std::string fw_;
    fic::link::HelloAck hello_{};
    fic::link::Config config_{};
    bool have_config_ = false;
    size_t n_ = 0;
    // Random start: a restarted fic_control must never repeat the epoch the
    // MCU is running, or its CONFIG would be taken for a duplicate and ignored.
    uint32_t config_epoch_ = static_cast<uint32_t>(std::random_device{}()) | 1u;
    uint32_t param_epoch_ = 0;
    std::vector<fic::link::Params> params_;
    Clock::time_point last_config_tx_{};
    fic::link::StateMsg state_{};
    double state_age_ = 1e9;

    void sendAllParams() {
        for (const auto& p : params_) link_.sendParams(p);
    }

    template <typename V>
    static double at(const std::vector<V>& v, size_t i, double dflt) {
        return i < v.size() ? static_cast<double>(v[i]) : dflt;
    }

    static void parseIp(const std::string& s, uint8_t out[4]) {
        unsigned a = 0, b = 0, c = 0, d = 0;
        if (std::sscanf(s.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) == 4 &&
            a < 256 && b < 256 && c < 256 && d < 256) {
            out[0] = a; out[1] = b; out[2] = c; out[3] = d;
        } else {
            out[0] = out[1] = out[2] = out[3] = 0;   // adapter default
        }
    }
};

} // namespace fic_hw
