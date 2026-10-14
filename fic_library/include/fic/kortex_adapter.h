#ifndef FIC_KORTEX_ADAPTER_H
#define FIC_KORTEX_ADAPTER_H

// Kinova Gen3 arm adapter for the FIC inner loop: drives the arm's Kortex
// session from inside the 1 kHz tick, never blocking.
//
// Sequence (as robot_library/Kinova/.../00-Torque_Controller.cpp does it):
//   TCP 10000: connect, Session.CreateSession, Base.SetServoingMode(LOW_LEVEL)
//   UDP 10001: Session.CreateSession, BaseCyclic.RefreshFeedback (prime)
//   cyclic:    each tick, read the feedback answering last tick's Refresh,
//              run the inner loop, send the next Refresh — position = measured
//              (what an actuator in POSITION mode holds), torque_joint = the
//              inner loop's torque (used once the actuator is in TORQUE mode)
//   TCP, asynchronous, one call at a time: ActuatorConfig.SetControlMode
//              TORQUE on every actuator when the inner loop wants torque,
//              POSITION when it lets go; Session.KeepAlive while idle
//   stop():    POSITION on every actuator, SINGLE_LEVEL servoing, close.
//
// Portable: the network is behind KortexNet (the firmware implements it on the
// W5500; a Linux test implements it on sockets).

#include <cmath>
#include <cstdint>
#include <cstring>

#include "inner_loop.h"
#include "kortex_min.h"

namespace fic {
namespace kortex {

struct KortexNet {
    virtual bool udpSend(const uint8_t* d, size_t n) = 0;       // to arm:10001
    virtual int udpRecv(uint8_t* d, size_t cap) = 0;            // from arm; 0 = none
    virtual bool tcpOpen() = 0;                                 // start connecting to arm:10000
    virtual int tcpStatus() = 0;                                // 0 connecting, 1 up, -1 down
    virtual bool tcpSend(const uint8_t* d, size_t n) = 0;
    virtual int tcpRecv(uint8_t* d, size_t cap) = 0;            // stream bytes; 0 = none
    virtual void tcpClose() = 0;
    virtual ~KortexNet() = default;
};

class Adapter {
public:
    enum class Phase : uint8_t {
        Off, TcpConnect, TcpSession, Servoing, UdpSession, Prime, Cyclic,
        StopPosition, StopServoing, Stopped, Failed,
    };
    static constexpr float kGripperMaxWidth = 0.085f;          // Robotiq 2F-85, as the driver
    static constexpr uint32_t kRpcTimeoutUs = 1000000;
    static constexpr uint32_t kKeepAliveUs = 500000;

    // Last error, for the status line.
    const char* error() const { return error_; }
    Phase phase() const { return phase_; }
    bool cyclic() const { return phase_ == Phase::Cyclic; }
    bool torqueModeActive() const { return torque_applied_; }
    uint32_t feedbackLost() const { return fb_lost_; }

    void start(KortexNet* net, int n_joints, uint64_t now_us) {
        net_ = net;
        n_ = n_joints > kMaxActuators ? kMaxActuators : n_joints;
        phase_ = Phase::TcpConnect;
        phase_since_ = now_us;
        tcp_session_ = udp_session_ = 0;
        torque_applied_ = torque_wanted_ = false;
        mode_step_ = -1;
        rx_len_ = 0;
        rpc_pending_ = false;
        have_fb_ = false;
        error_ = "";
        net_->tcpClose();
        net_->tcpOpen();
    }

    // Graceful release: actuators back to POSITION, SINGLE_LEVEL servoing.
    void stop(uint64_t now_us) {
        if (phase_ == Phase::Off || phase_ == Phase::Stopped || phase_ == Phase::Failed) return;
        if (phase_ < Phase::Servoing) { net_->tcpClose(); phase_ = Phase::Stopped; return; }
        phase_ = Phase::StopPosition;
        phase_since_ = now_us;
        mode_step_ = 0;
        rpc_pending_ = false;
    }

    // Start of tick: advance the session, and if a cyclic feedback arrived,
    // fill fb (raw: rad, Nm, as the inner loop wants it).
    void poll(uint64_t now_us, fic::mcu::ArmFeedback& fb) {
        fb.valid = false;
        if (!net_ || phase_ == Phase::Off || phase_ == Phase::Stopped || phase_ == Phase::Failed) return;
        pollTcp(now_us);
        pollUdp(now_us, fb);
        advance(now_us);
    }

    // End of tick: send this tick's command (cyclic phase only).
    void command(const fic::mcu::ArmCommand& cmd, uint64_t now_us) {
        // The arm is in LOW_LEVEL servoing from Cyclic until SINGLE_LEVEL is
        // confirmed, and faults if the cyclic stream stops: keep sending
        // through the whole stop sequence too.
        const bool streaming = phase_ == Phase::Cyclic || phase_ == Phase::StopPosition ||
                               phase_ == Phase::StopServoing;
        if (!streaming || !have_fb_) return;
        if (phase_ == Phase::Cyclic) torque_wanted_ = cmd.torque_mode;
        float tau[kMaxActuators];
        for (int i = 0; i < n_; ++i) {
            // While an actuator is (still) in TORQUE mode it must keep getting a
            // torque: hold the last one until POSITION is confirmed.
            if (cmd.torque_mode && phase_ == Phase::Cyclic) last_tau_[i] = cmd.tau[i];
            tau[i] = torque_applied_ || cmd.torque_mode ? last_tau_[i] : 0.0f;
        }
        float grip_pct = -1.0f, grip_vel = 0.0f;
        if (cmd.gripper_width >= 0.0f && fb_has_gripper_) {
            float closed = 1.0f - cmd.gripper_width / kGripperMaxWidth;
            closed = closed < 0 ? 0 : closed > 1 ? 1 : closed;
            grip_pct = closed * 100.0f;
            const float err = std::fabs(grip_pct - fb_gripper_pct_);
            grip_vel = 2.2f * err < 100.0f ? 2.2f * err : 100.0f;
        }
        ++frame_id_;
        const size_t pn = encodeCommand(pl_, sizeof(pl_), frame_id_, n_, pos_deg_, tau, grip_pct, grip_vel, 100.0f);
        const size_t fn = encodeRequest(fr_, sizeof(fr_), Refresh, udp_session_, uint16_t(frame_id_), 0, pl_, pn);
        if (fn) net_->udpSend(fr_, fn);
        (void)now_us;
    }

private:
    KortexNet* net_ = nullptr;
    int n_ = 7;
    Phase phase_ = Phase::Off;
    uint64_t phase_since_ = 0;
    const char* error_ = "";
    uint16_t tcp_session_ = 0, udp_session_ = 0;
    uint16_t msg_id_ = 0;
    uint32_t frame_id_ = 0;

    // TCP RPC: one call in flight.
    bool rpc_pending_ = false;
    uint16_t rpc_msg_ = 0;
    uint64_t rpc_sent_ = 0, last_tcp_ = 0;
    bool rpc_done_ = false, rpc_ok_ = false;
    Header rpc_hdr_{};
    uint8_t rx_[2048];
    size_t rx_len_ = 0;

    // Control-mode switching over TCP.
    bool torque_wanted_ = false, torque_applied_ = false;
    int mode_step_ = -1;              // actuator index being switched, -1 = idle

    // Cyclic state.
    bool have_fb_ = false;
    float pos_deg_[kMaxActuators] = {};
    float last_tau_[kMaxActuators] = {};
    bool fb_has_gripper_ = false;
    float fb_gripper_pct_ = 0;
    uint64_t last_fb_us_ = 0;
    uint32_t fb_lost_ = 0;

    uint8_t pl_[512], fr_[1400], tx_[1400];
    uint8_t udp_rx_[2048];

    void fail(const char* why) { error_ = why; phase_ = Phase::Failed; net_->tcpClose(); }
    void enter(Phase p, uint64_t now) { phase_ = p; phase_since_ = now; rpc_done_ = false; }

    bool rpcTcp(uint32_t fn, uint8_t device, const uint8_t* payload, size_t n, uint64_t now) {
        rpc_msg_ = ++msg_id_;
        const size_t fl = encodeRequest(fr_, sizeof(fr_), fn, tcp_session_, rpc_msg_, device, payload, n);
        if (!fl || fl + 12 > sizeof(tx_)) return false;
        tcpHeader(tx_, uint32_t(fl));
        std::memcpy(tx_ + 12, fr_, fl);
        if (!net_->tcpSend(tx_, fl + 12)) return false;
        rpc_pending_ = true;
        rpc_done_ = false;
        rpc_sent_ = last_tcp_ = now;
        return true;
    }

    void pollTcp(uint64_t now) {
        if (phase_ == Phase::TcpConnect) return;
        const int got = net_->tcpRecv(rx_ + rx_len_, sizeof(rx_) - rx_len_);
        if (got > 0) rx_len_ += size_t(got);
        while (rx_len_ >= 12) {
            const long fl = tcpFrameLength(rx_);
            if (fl < 0 || size_t(fl) > sizeof(rx_) - 12) { fail("kortex tcp: bad stream header"); return; }
            if (rx_len_ < 12 + size_t(fl)) break;
            Header h; const uint8_t* p; size_t pn;
            if (decodeFrame(rx_ + 12, size_t(fl), h, p, pn) && h.frameType() == RESPONSE &&
                rpc_pending_ && h.messageId() == rpc_msg_) {
                rpc_pending_ = false;
                rpc_done_ = true;
                rpc_ok_ = h.errorCode() == 0;
                rpc_hdr_ = h;
            }
            std::memmove(rx_, rx_ + 12 + fl, rx_len_ - 12 - size_t(fl));
            rx_len_ -= 12 + size_t(fl);
        }
        if (rpc_pending_ && now - rpc_sent_ > kRpcTimeoutUs) fail("kortex tcp: no response");
        if (net_->tcpStatus() < 0 && phase_ != Phase::StopServoing) fail("kortex tcp: connection lost");
    }

    void pollUdp(uint64_t now, fic::mcu::ArmFeedback& fb) {
        int n;
        while ((n = net_->udpRecv(udp_rx_, sizeof(udp_rx_))) > 0) {
            Header h; const uint8_t* p; size_t pn;
            if (!decodeFrame(udp_rx_, size_t(n), h, p, pn) || h.frameType() != RESPONSE) continue;
            if (h.functionUid() == CreateSession && phase_ == Phase::UdpSession) {
                if (h.errorCode()) { fail("kortex udp: session refused"); return; }
                udp_session_ = h.sessionId();
                rpc_done_ = true;
                continue;
            }
            if (h.functionUid() != Refresh && h.functionUid() != RefreshFeedback) continue;
            if (h.errorCode()) continue;
            Feedback k;
            if (!decodeFeedback(p, pn, k) || k.n_actuators < n_) continue;
            for (int i = 0; i < n_; ++i) {
                pos_deg_[i] = k.act[i].position;
                const float rad = k.act[i].position * float(M_PI / 180.0);
                fb.q[i] = std::atan2(std::sin(rad), std::cos(rad));        // driver: wrapToPi
                fb.tau[i] = k.act[i].torque;
            }
            for (int j = 0; j < 6; ++j) fb.wrench[j] = k.tool_wrench[j];
            fb_has_gripper_ = k.has_gripper;
            if (k.has_gripper) {
                fb_gripper_pct_ = k.gripper_position;
                float closed = k.gripper_position / 100.0f;
                closed = closed < 0 ? 0 : closed > 1 ? 1 : closed;
                fb.gripper_pos = kGripperMaxWidth * (1.0f - closed);
                fb.gripper_current = k.gripper_current;
            }
            fb.arm_fault = k.arm_state == ARM_IN_FAULT;
            fb.valid = true;
            have_fb_ = true;
            last_fb_us_ = now;
            if (phase_ == Phase::Prime) rpc_done_ = true;
        }
        if (phase_ == Phase::Cyclic && !fb.valid) ++fb_lost_;
    }

    void advance(uint64_t now) {
        uint8_t p[64];
        switch (phase_) {
        case Phase::TcpConnect: {
            const int st = net_->tcpStatus();
            if (st > 0) {
                const size_t n = encodeCreateSession(p, sizeof(p), "admin", "admin", 60000, 60000);
                if (!rpcTcp(CreateSession, 0, p, n, now)) { fail("kortex tcp: send failed"); return; }
                enter(Phase::TcpSession, now);
            } else if (now - phase_since_ > 3000000) {
                fail("kortex tcp: cannot connect to the arm (port 10000)");
            }
            break;
        }
        case Phase::TcpSession:
            if (!rpc_done_) break;
            if (!rpc_ok_) { fail("kortex: TCP session refused"); return; }
            tcp_session_ = rpc_hdr_.sessionId();
            if (!rpcTcp(SetServoingMode, 0, p, encodeServoingMode(p, sizeof(p), LOW_LEVEL), now)) {
                fail("kortex tcp: send failed"); return;
            }
            enter(Phase::Servoing, now);
            break;
        case Phase::Servoing:
            if (!rpc_done_) break;
            if (!rpc_ok_) { fail("kortex: LOW_LEVEL servoing refused (arm in fault?)"); return; }
            {
                const size_t n = encodeCreateSession(p, sizeof(p), "admin", "admin", 60000, 2000);
                const size_t fl = encodeRequest(fr_, sizeof(fr_), CreateSession, 0, ++msg_id_, 0, p, n);
                net_->udpSend(fr_, fl);
            }
            enter(Phase::UdpSession, now);
            break;
        case Phase::UdpSession:
            if (rpc_done_) {
                const size_t fl = encodeRequest(fr_, sizeof(fr_), RefreshFeedback, udp_session_, ++msg_id_, 0, nullptr, 0);
                net_->udpSend(fr_, fl);
                enter(Phase::Prime, now);
            } else if (now - phase_since_ > kRpcTimeoutUs) {
                fail("kortex udp: no session (port 10001)");
            }
            break;
        case Phase::Prime:
            if (rpc_done_) enter(Phase::Cyclic, now);
            else if (now - phase_since_ > kRpcTimeoutUs) fail("kortex udp: no feedback");
            break;
        case Phase::Cyclic:
            if (rpc_pending_) break;
            if (torque_wanted_ != torque_applied_ || mode_step_ >= 0) {
                if (mode_step_ < 0) mode_step_ = 0;
                if (mode_step_ > 0 && rpc_done_ && !rpc_ok_) { fail("kortex: SetControlMode refused"); return; }
                if (mode_step_ < n_) {
                    const ControlMode m = torque_wanted_ ? TORQUE : POSITION;
                    rpcTcp(SetControlMode, uint8_t(mode_step_ + 1), p, encodeControlMode(p, sizeof(p), m), now);
                    ++mode_step_;
                } else {
                    torque_applied_ = torque_wanted_;
                    mode_step_ = -1;
                }
            } else if (now - last_tcp_ > kKeepAliveUs) {
                rpcTcp(0x0010003 /* Session.KeepAlive */, 0, nullptr, 0, now);
            }
            break;
        case Phase::StopPosition:
            if (rpc_pending_) break;
            if (mode_step_ < n_) {
                rpcTcp(SetControlMode, uint8_t(mode_step_ + 1), p, encodeControlMode(p, sizeof(p), POSITION), now);
                ++mode_step_;
            } else {
                torque_applied_ = false;
                rpcTcp(SetServoingMode, 0, p, encodeServoingMode(p, sizeof(p), SINGLE_LEVEL), now);
                enter(Phase::StopServoing, now);
            }
            break;
        case Phase::StopServoing:
            if (rpc_done_ || now - phase_since_ > kRpcTimeoutUs) {
                net_->tcpClose();
                phase_ = Phase::Stopped;
            }
            break;
        default:
            break;
        }
    }
};

}  // namespace kortex
}  // namespace fic

#endif  // FIC_KORTEX_ADAPTER_H
