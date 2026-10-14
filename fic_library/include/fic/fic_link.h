#ifndef FIC_LINK_H
#define FIC_LINK_H

// FIC-link: the UDP protocol between fic_hardware (Linux, outer loops) and an
// FIC inner-loop MCU (joint impedance / admittance + vendor arm adapter).
//
// Portable: no heap, no exceptions, no POSIX — compiled into both the MCU
// firmware and fic_hardware. Both ends are little-endian (aarch64 / x86-64 /
// Cortex-M), so fields are copied as-is.
//
// Every datagram: Header | payload | crc16 (CCITT-FALSE over header+payload).
// Joint arrays are sent with length n (the active joint count), so one packet
// fits any robot up to kLinkMaxJoints within a 1472-byte UDP payload.
//
//   Linux                                   MCU
//   HELLO  ───────────────────────────────►
//          ◄─────────────────────────────── HELLO_ACK (adapters it can drive)
//   CONFIG ───────────────────────────────► (robot, limits, arm address)
//   PARAMS ───────────────────────────────► (one per joint, per mode change)
//   CMD    ──────── ≤1 kHz, best effort ──►
//          ◄───────────── 1 kHz ─────────── STATE
//
// The MCU owns stability: it runs the joint layer every tick from the latest
// CMD. A late CMD only slows the outer loops; a missing one drives the MCU into
// HOLD, then RAMP (see McuState).

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "fic_joint_core.h"

namespace fic {
namespace link {

inline constexpr uint32_t kMagic = 0x4C434946u;   // "FICL"
inline constexpr uint16_t kVersion = 1;
// Below 7400: ROS 2's DDS owns 7400 (domain-0 discovery) and every port from
// there up (7400 + 250*domain + participant offsets). FIC-link shares hosts and
// networks with ROS, so it must never land in that range.
inline constexpr uint16_t kDefaultMcuPort = 7350;
inline constexpr int kLinkMaxJoints = 16;
inline constexpr size_t kMaxDatagram = 1472;

enum class MsgType : uint16_t {
    Hello = 1,
    HelloAck = 2,
    Config = 3,
    Params = 4,
    Cmd = 5,
    State = 6,
};

// Vendor adapters an MCU firmware can drive. A robot opts in with
// `mcu_adapter: <name>` in robots_config.yaml; it runs on the MCU only when the
// MCU reports that adapter in HELLO_ACK.
enum class Adapter : uint32_t {
    None = 0,
    Loopback = 1,   // simulated plant inside the MCU — link / timing tests
    Kortex = 2,     // Kinova Gen3 over Ethernet (UDP 10001 BaseCyclic)
    CanFd = 3,      // CAN-FD actuators (e.g. CSPM AK60) on the MCU's FDCAN
    HostSim = 4,    // the arm is simulated on the Linux host (MuJoCo), in lockstep
};

inline uint32_t adapterBit(Adapter a) { return 1u << static_cast<uint32_t>(a); }

inline Adapter adapterFromName(const char* s) {
    if (!s) return Adapter::None;
    if (!std::strcmp(s, "loopback")) return Adapter::Loopback;
    if (!std::strcmp(s, "kortex"))   return Adapter::Kortex;
    if (!std::strcmp(s, "canfd"))    return Adapter::CanFd;
    if (!std::strcmp(s, "hostsim"))  return Adapter::HostSim;
    return Adapter::None;
}

// Inner-loop law the MCU runs (fic_joint_core.h).
enum class InnerMode : uint8_t {
    Gravity = 0,             // passive: gravity compensation only
    Impedance = 1,           // JointCore::impedance
    ImpedancePlanner = 2,    // JointCore::impedanceWithPlanner
    Admittance = 3,          // JointCore::admittance, push-and-stay
    AdmittanceHaptic = 4,    // JointCore::admittance, haptic (joint mode 4)
    Hapfic = 5,              // haptic admittance around tau_des: Cartesian modes land here
    Torque = 6,              // pass tau_des through (+ gravity); MCU keeps safety only
};

inline constexpr uint8_t kInnerModeMax = static_cast<uint8_t>(InnerMode::Torque);

enum class McuState : uint8_t {
    Idle = 0,         // no CONFIG yet
    Connecting = 1,   // adapter bringing the arm up
    Ready = 2,        // feedback flowing, commanding gravity only
    Running = 3,      // tracking CMDs
    Hold = 4,         // CMDs stale: impedance to the last position + gravity
    Ramp = 5,         // CMDs gone / e-stop: torque ramping to gravity, then exit
    Fault = 6,        // see fault bits; needs a new CONFIG
};

// CMD flags
inline constexpr uint8_t kCmdEnable = 1u << 0;      // allow torque (else Ready)
inline constexpr uint8_t kCmdEstop = 1u << 1;       // ramp down now, latch
inline constexpr uint8_t kCmdStopTorque = 1u << 2;  // leave torque mode after ramp
inline constexpr uint8_t kCmdUsePlanner = 1u << 3;  // admittance through the planner
inline constexpr uint8_t kCmdHasGravJac = 1u << 4;  // dg/dq block present

// STATE fault bits
inline constexpr uint32_t kFaultArm = 1u << 0;          // vendor-reported fault
inline constexpr uint32_t kFaultArmLink = 1u << 1;      // arm feedback lost
inline constexpr uint32_t kFaultJointLimit = 1u << 2;   // joint outside CONFIG limits
inline constexpr uint32_t kFaultBadParams = 1u << 3;    // PARAMS rejected
inline constexpr uint32_t kFaultOverrun = 1u << 4;      // control tick overran
inline constexpr uint32_t kFaultEstop = 1u << 5;

#pragma pack(push, 1)
struct Header {
    uint32_t magic;
    uint16_t version;
    uint16_t type;
    uint32_t seq;
    uint16_t len;        // payload bytes
    uint16_t reserved;
};

struct Hello {
    uint32_t nonce;
};

struct HelloAck {
    uint32_t nonce;
    uint32_t fw_version;
    uint32_t adapters;       // adapterBit() mask
    uint16_t max_joints;
    uint8_t state;           // McuState
    uint8_t reserved;
    char fw_name[24];
};

struct Config {
    uint32_t config_epoch;
    uint32_t adapter;        // Adapter
    uint16_t n_joints;
    uint16_t arm_port;       // 0 = adapter default
    uint8_t arm_ip[4];
    float hold_after_s;      // stale CMD -> Hold
    float ramp_after_s;      // stale CMD -> Ramp
    float deadband_joint_tau;
    float dt;                // inner-loop period (s)
    float torque_cutoff_hz;  // measured-torque Butterworth (fic_hardware: 200)
    float state_cutoff_hz;   // position Butterworth (fic_hardware: 10)
    float damping_alpha;     // first-order lag on the damping velocity
    char robot_name[32];
    // Per joint (n_joints entries each, fixed arrays for simplicity)
    float lower[kLinkMaxJoints];              // controller clamp limits (URDF)
    float upper[kLinkMaxJoints];
    float fault_lower[kLinkMaxJoints];        // hard-fault limits (joint_limits_*_deg)
    float fault_upper[kLinkMaxJoints];
    float torque_limit[kLinkMaxJoints];
    float torque_rate_limit[kLinkMaxJoints];  // Nm per tick
    float torque_sign[kLinkMaxJoints];        // measured-torque sign convention
    uint8_t continuous[kLinkMaxJoints];
};

struct Params {
    uint32_t param_epoch;
    uint16_t joint;
    uint16_t n_joints;       // joints in this epoch (MCU applies when all arrive)
    fic::core::JointParamsPOD p;
};
#pragma pack(pop)

// ---------------------------------------------------------------------------
// Variable-length CMD / STATE. Built and parsed through Writer / Reader.
// ---------------------------------------------------------------------------
struct CmdMsg {
    uint32_t config_epoch = 0;
    uint32_t param_epoch = 0;
    InnerMode mode = InnerMode::Gravity;
    uint8_t flags = 0;
    uint16_t n = 0;
    float gripper_width = -1.0f;              // < 0: no gripper command
    float q_target[kLinkMaxJoints] = {};
    float tau_des[kLinkMaxJoints] = {};        // desired effort for haptic admittance (no gravity)
    float tau_ff[kLinkMaxJoints] = {};         // additive torque (walls, null-space...)
    float kd[kLinkMaxJoints] = {};             // velocity damping gain (Nm s/rad), 0 = off
    float q0[kLinkMaxJoints] = {};             // pose g0 was evaluated at
    float g0[kLinkMaxJoints] = {};             // gravity at q0
    float dgdq[kLinkMaxJoints * kLinkMaxJoints] = {};  // row-major n x n, if kCmdHasGravJac
};

struct StateMsg {
    uint32_t ack_seq = 0;          // last CMD seq applied
    uint32_t tick = 0;
    uint32_t config_epoch = 0;
    uint32_t param_epoch = 0;
    uint32_t faults = 0;
    uint16_t jitter_us_max = 0;    // worst tick lateness since last STATE
    uint16_t cmd_age_ms = 0;
    McuState state = McuState::Idle;
    InnerMode mode = InnerMode::Gravity;
    uint16_t n = 0;
    float q_raw[kLinkMaxJoints] = {};          // arm position as the adapter read it
    float tau_raw[kLinkMaxJoints] = {};        // arm torque as the adapter read it (vendor sign)
    float q[kLinkMaxJoints] = {};              // MCU-filtered position (what the law used)
    float tau_meas[kLinkMaxJoints] = {};       // MCU-filtered, sign-corrected torque
    float tau_cmd[kLinkMaxJoints] = {};
    float g_used[kLinkMaxJoints] = {};
    float adm_offset[kLinkMaxJoints] = {};
    float q_target_adm[kLinkMaxJoints] = {};
    float wrench[6] = {};
    float gripper_pos = 0.0f;
    float gripper_current = 0.0f;
};

// ---------------------------------------------------------------------------
// CRC16-CCITT-FALSE
// ---------------------------------------------------------------------------
// Table-driven (bytewise) — ~8x faster than the bitwise loop, identical
// results; the MCU checks several datagrams per 1 ms tick.
struct Crc16Table {
    uint16_t t[256];
    constexpr Crc16Table() : t() {
        for (int i = 0; i < 256; ++i) {
            uint16_t c = static_cast<uint16_t>(i << 8);
            for (int b = 0; b < 8; ++b)
                c = (c & 0x8000) ? static_cast<uint16_t>((c << 1) ^ 0x1021) : static_cast<uint16_t>(c << 1);
            t[i] = c;
        }
    }
};
inline constexpr Crc16Table kCrc16Table{};

inline uint16_t crc16(const uint8_t* d, size_t n, uint16_t crc = 0xFFFF) {
    for (size_t i = 0; i < n; ++i)
        crc = static_cast<uint16_t>((crc << 8) ^ kCrc16Table.t[((crc >> 8) ^ d[i]) & 0xFF]);
    return crc;
}

// ---------------------------------------------------------------------------
// Framing
// ---------------------------------------------------------------------------
class Writer {
public:
    Writer(uint8_t* buf, size_t cap) : buf_(buf), cap_(cap) {}

    void begin(MsgType t, uint32_t seq) {
        pos_ = sizeof(Header);
        ok_ = cap_ >= sizeof(Header) + 2;
        type_ = t;
        seq_ = seq;
    }
    void put(const void* p, size_t n) {
        if (!ok_ || pos_ + n + 2 > cap_) { ok_ = false; return; }
        std::memcpy(buf_ + pos_, p, n);
        pos_ += n;
    }
    template <typename V> void put(const V& v) { put(&v, sizeof(V)); }
    void putArr(const float* a, int n) { put(a, sizeof(float) * static_cast<size_t>(n)); }

    // Returns the datagram length, or 0 on overflow.
    size_t end() {
        if (!ok_) return 0;
        Header h{kMagic, kVersion, static_cast<uint16_t>(type_), seq_,
                 static_cast<uint16_t>(pos_ - sizeof(Header)), 0};
        std::memcpy(buf_, &h, sizeof(h));
        uint16_t c = crc16(buf_, pos_);
        std::memcpy(buf_ + pos_, &c, 2);
        return pos_ + 2;
    }

private:
    uint8_t* buf_;
    size_t cap_;
    size_t pos_ = 0;
    bool ok_ = false;
    MsgType type_ = MsgType::Hello;
    uint32_t seq_ = 0;
};

class Reader {
public:
    // Validates magic, version, length and CRC.
    bool open(const uint8_t* buf, size_t len) {
        if (len < sizeof(Header) + 2) return false;
        std::memcpy(&h_, buf, sizeof(h_));
        if (h_.magic != kMagic || h_.version != kVersion) return false;
        if (sizeof(Header) + h_.len + 2 != len) return false;
        uint16_t c;
        std::memcpy(&c, buf + sizeof(Header) + h_.len, 2);
        if (crc16(buf, sizeof(Header) + h_.len) != c) return false;
        p_ = buf + sizeof(Header);
        left_ = h_.len;
        return true;
    }
    MsgType type() const { return static_cast<MsgType>(h_.type); }
    uint32_t seq() const { return h_.seq; }

    bool get(void* out, size_t n) {
        if (n > left_) return false;
        std::memcpy(out, p_, n);
        p_ += n;
        left_ -= n;
        return true;
    }
    template <typename V> bool get(V& v) { return get(&v, sizeof(V)); }
    bool getArr(float* a, int n) { return get(a, sizeof(float) * static_cast<size_t>(n)); }
    size_t left() const { return left_; }

private:
    Header h_{};
    const uint8_t* p_ = nullptr;
    size_t left_ = 0;
};

// ---------------------------------------------------------------------------
// CMD / STATE codecs
// ---------------------------------------------------------------------------
inline size_t encodeCmd(const CmdMsg& m, uint32_t seq, uint8_t* buf, size_t cap) {
    if (m.n > kLinkMaxJoints) return 0;
    Writer w(buf, cap);
    w.begin(MsgType::Cmd, seq);
    w.put(m.config_epoch);
    w.put(m.param_epoch);
    w.put(static_cast<uint8_t>(m.mode));
    w.put(m.flags);
    w.put(m.n);
    w.put(m.gripper_width);
    w.putArr(m.q_target, m.n);
    w.putArr(m.tau_des, m.n);
    w.putArr(m.tau_ff, m.n);
    w.putArr(m.kd, m.n);
    w.putArr(m.q0, m.n);
    w.putArr(m.g0, m.n);
    if (m.flags & kCmdHasGravJac) w.putArr(m.dgdq, m.n * m.n);
    return w.end();
}

inline bool decodeCmd(Reader& r, CmdMsg& m) {
    uint8_t mode = 0;
    bool ok = r.get(m.config_epoch) && r.get(m.param_epoch) && r.get(mode) &&
              r.get(m.flags) && r.get(m.n) && r.get(m.gripper_width);
    if (!ok || m.n > kLinkMaxJoints || mode > kInnerModeMax)
        return false;
    m.mode = static_cast<InnerMode>(mode);
    ok = r.getArr(m.q_target, m.n) && r.getArr(m.tau_des, m.n) && r.getArr(m.tau_ff, m.n) &&
         r.getArr(m.kd, m.n) && r.getArr(m.q0, m.n) && r.getArr(m.g0, m.n);
    if (ok && (m.flags & kCmdHasGravJac)) ok = r.getArr(m.dgdq, m.n * m.n);
    return ok && r.left() == 0;
}

inline size_t encodeState(const StateMsg& m, uint32_t seq, uint8_t* buf, size_t cap) {
    if (m.n > kLinkMaxJoints) return 0;
    Writer w(buf, cap);
    w.begin(MsgType::State, seq);
    w.put(m.ack_seq);
    w.put(m.tick);
    w.put(m.config_epoch);
    w.put(m.param_epoch);
    w.put(m.faults);
    w.put(m.jitter_us_max);
    w.put(m.cmd_age_ms);
    w.put(static_cast<uint8_t>(m.state));
    w.put(static_cast<uint8_t>(m.mode));
    w.put(m.n);
    w.putArr(m.q_raw, m.n);
    w.putArr(m.tau_raw, m.n);
    w.putArr(m.q, m.n);
    w.putArr(m.tau_meas, m.n);
    w.putArr(m.tau_cmd, m.n);
    w.putArr(m.g_used, m.n);
    w.putArr(m.adm_offset, m.n);
    w.putArr(m.q_target_adm, m.n);
    w.putArr(m.wrench, 6);
    w.put(m.gripper_pos);
    w.put(m.gripper_current);
    return w.end();
}

inline bool decodeState(Reader& r, StateMsg& m) {
    uint8_t st = 0, mode = 0;
    bool ok = r.get(m.ack_seq) && r.get(m.tick) && r.get(m.config_epoch) &&
              r.get(m.param_epoch) && r.get(m.faults) && r.get(m.jitter_us_max) &&
              r.get(m.cmd_age_ms) && r.get(st) && r.get(mode) && r.get(m.n);
    if (!ok || m.n > kLinkMaxJoints) return false;
    m.state = static_cast<McuState>(st);
    m.mode = static_cast<InnerMode>(mode);
    ok = r.getArr(m.q_raw, m.n) && r.getArr(m.tau_raw, m.n) && r.getArr(m.q, m.n) && r.getArr(m.tau_meas, m.n) &&
         r.getArr(m.tau_cmd, m.n) && r.getArr(m.g_used, m.n) && r.getArr(m.adm_offset, m.n) &&
         r.getArr(m.q_target_adm, m.n) && r.getArr(m.wrench, 6) &&
         r.get(m.gripper_pos) && r.get(m.gripper_current);
    return ok && r.left() == 0;
}

template <typename V>
inline size_t encodeFixed(MsgType t, const V& v, uint32_t seq, uint8_t* buf, size_t cap) {
    Writer w(buf, cap);
    w.begin(t, seq);
    w.put(v);
    return w.end();
}

template <typename V>
inline bool decodeFixed(Reader& r, V& v) { return r.get(v) && r.left() == 0; }

// Worst case CMD must fit one datagram.
static_assert(sizeof(Header) + 4 + 4 + 1 + 1 + 2 + 4 +
              sizeof(float) * (6 * kLinkMaxJoints + kLinkMaxJoints * kLinkMaxJoints) + 2
              <= kMaxDatagram, "CMD exceeds one UDP datagram");
static_assert(sizeof(Header) + sizeof(Params) + 2 <= kMaxDatagram, "PARAMS too large");
// JointParamsPOD crosses between aarch64 Linux and the Cortex-M MCU; pin its
// layout so an ABI difference fails the build instead of scrambling gains.
static_assert(sizeof(fic::core::JointParamsPOD) == 504, "JointParamsPOD layout changed");
static_assert(offsetof(fic::core::JointParamsPOD, error_max) == 32, "JointParamsPOD layout changed");
static_assert(sizeof(Header) + sizeof(Config) + 2 <= kMaxDatagram, "CONFIG too large");

} // namespace link
} // namespace fic

#endif // FIC_LINK_H
