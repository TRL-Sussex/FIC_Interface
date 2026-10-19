#ifndef FIC_MCU_NODE_H
#define FIC_MCU_NODE_H

// The MCU end of FIC-link, minus the platform: datagram dispatch around an
// InnerLoop. The Zephyr firmware and the Linux MCU simulator (fic_mcu_sim) both
// wrap this — they only supply a socket, a 1 kHz timer and a vendor adapter.
//
//   datagram in  -> onDatagram()  -> HELLO_ACK reply / CONFIG / PARAMS / CMD
//   every tick   -> tick()        -> ArmCommand for the adapter
//   every tick   -> buildState()  -> STATE datagram back to Linux

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "fic_link.h"
#include "inner_loop.h"

namespace fic {
namespace mcu {

class McuNode {
public:
    McuNode(const char* fw_name, uint32_t fw_version, uint32_t adapters)
        : fw_version_(fw_version), adapters_(adapters) {
        std::strncpy(fw_name_, fw_name, sizeof(fw_name_) - 1);
    }

    // Handle one datagram. Returns the length of a reply to send back to the
    // sender (only HELLO has one), 0 otherwise.
    size_t onDatagram(const uint8_t* buf, size_t len, uint64_t now_us, uint8_t* reply, size_t cap) {
        fic::link::Reader r;
        if (!r.open(buf, len)) { ++rx_bad_; return 0; }
        switch (r.type()) {
        case fic::link::MsgType::Hello: {
            fic::link::Hello h;
            if (!fic::link::decodeFixed(r, h)) break;
            fic::link::HelloAck a{};
            a.nonce = h.nonce;
            a.fw_version = fw_version_;
            a.adapters = adapters_;
            a.max_joints = fic::link::kLinkMaxJoints;
            a.state = static_cast<uint8_t>(loop_.state());
            std::memcpy(a.fw_name, fw_name_, sizeof(a.fw_name));
            return fic::link::encodeFixed(fic::link::MsgType::HelloAck, a, ++tx_seq_, reply, cap);
        }
        case fic::link::MsgType::Config: {
            fic::link::Config c;
            if (!fic::link::decodeFixed(r, c)) break;
            if (!(adapters_ & (1u << c.adapter)) || c.n_joints == 0 ||
                c.n_joints > fic::link::kLinkMaxJoints) { ++rx_bad_; break; }
            // A re-sent copy of the active CONFIG must not reset a running loop.
            if (have_config_ && c.config_epoch == loop_.config().config_epoch) break;
            loop_.configure(c);
            have_config_ = true;
            ++config_changes_;
            break;
        }
        case fic::link::MsgType::Params: {
            fic::link::Params p;
            if (!fic::link::decodeFixed(r, p) || !loop_.acceptParams(p)) ++rx_bad_;
            break;
        }
        case fic::link::MsgType::Estop: {
            fic::link::EstopMsg e;
            if (!fic::link::decodeFixed(r, e)) { ++rx_bad_; break; }
            loop_.estop(/*direct=*/true);
            break;
        }
        case fic::link::MsgType::Cmd: {
            if (!fic::link::decodeCmd(r, cmd_scratch_)) { ++rx_bad_; break; }
            loop_.onCmd(cmd_scratch_, r.seq(), now_us);
            break;
        }
        default:
            break;
        }
        return 0;
    }

    void tick(const ArmFeedback& fb, uint64_t now_us, ArmCommand& out) { loop_.tick(fb, now_us, out); }

    size_t buildState(uint8_t* buf, size_t cap, uint16_t jitter_us_max) {
        loop_.fillState(state_scratch_, jitter_us_max);
        return fic::link::encodeState(state_scratch_, ++tx_seq_, buf, cap);
    }

    InnerLoop& loop() { return loop_; }
    // Adapters the firmware found it can drive (e.g. Kortex only if the
    // Ethernet shield answered at boot) — reported in HELLO_ACK.
    void setAdapters(uint32_t adapters) { adapters_ = adapters; }
    uint32_t adapters() const { return adapters_; }
    bool hasConfig() const { return have_config_; }
    const fic::link::Config& config() const { return loop_.config(); }
    // Bumps on every applied CONFIG: the adapter (re)connects when it changes.
    uint32_t configChanges() const { return config_changes_; }
    uint32_t badDatagrams() const { return rx_bad_; }

private:
    char fw_name_[24] = {};
    uint32_t fw_version_;
    uint32_t adapters_;
    InnerLoop loop_;
    bool have_config_ = false;
    uint32_t config_changes_ = 0;
    uint32_t tx_seq_ = 0;
    uint32_t rx_bad_ = 0;
    fic::link::CmdMsg cmd_scratch_;       // large: kept off the stack
    fic::link::StateMsg state_scratch_;
};

} // namespace mcu
} // namespace fic

#endif // FIC_MCU_NODE_H
