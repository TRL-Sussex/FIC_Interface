#ifndef FIC_KORTEX_MIN_H
#define FIC_KORTEX_MIN_H

// Minimal Kinova Kortex client codec derived from API calls.
// This file acts as a low-level abstraction of the underlying communication protocol between robot and client.
// To be used on embedded platforms with hardware level control of the networking stack, without requiring the massive overhead of the entire Kortex API.
// Only the functions required for low-level torque control are ported.

// Communication overview:
//   UDP 10001 (cyclic):   one serialized Kinova.Api.Frame per datagram
//   TCP 10000 (config):   "\x07xEtRoK\x07" + u32 big-endian length + Frame
//   Frame  { Header header = 1; bytes payload = 2; }
//   Header { fixed32 frame_info = 1; message_info = 2; service_info = 3; payload_info = 4; }
//     frame_info   = headerVersion<<28 | frameType<<24 | deviceId<<16 | errorCode<<12 | subCode
//     message_info = sessionId<<16 | messageId
//     service_info = serviceVersion<<28 | serviceId<<16 | functionId
//     payload_info = payload length (24 bits)

// Services used (serviceVersion 1):
//   Session.CreateSession 0x0010001, CloseSession 0x0010002
//   Base.SetServoingMode 0x0020098, GetArmState 0x00200af, ClearFaults 0x0020092
//   BaseCyclic.Refresh 0x0030001, RefreshFeedback 0x0030003
//   ActuatorConfig.SetControlMode 0x00a0007 (deviceId = actuator 1..7)

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace fic {
namespace kortex {

inline constexpr uint16_t kTcpPort = 10000;
inline constexpr uint16_t kUdpPort = 10001;
inline constexpr char kTcpMagic[8] = {'\x07', 'x', 'E', 't', 'R', 'o', 'K', '\x07'};
inline constexpr int kMaxActuators = 7;

enum FrameType : uint8_t { REQUEST = 1, RESPONSE = 3, NOTIFICATION = 5 };
enum Fn : uint32_t {
    CreateSession = 0x0010001,
    CloseSession = 0x0010002,
    SetServoingMode = 0x0020098,
    GetArmState = 0x00200af,
    ClearFaults = 0x0020092,
    Refresh = 0x0030001,
    RefreshFeedback = 0x0030003,
    SetControlMode = 0x00a0007,
};
enum ServoingMode : uint32_t { SINGLE_LEVEL = 2, LOW_LEVEL = 3 };
enum ControlMode : uint32_t { POSITION = 1, TORQUE = 3 };
enum ArmState : uint32_t { ARM_IN_FAULT = 4, ARM_SERVOING_LOW_LEVEL = 6, ARM_SERVOING_READY = 7 };

// Protobuf wire types (low 3 bits of every field tag).
enum WireType : uint8_t { WIRE_VARINT = 0, WIRE_I64 = 1, WIRE_LEN = 2, WIRE_I32 = 5 };

// ------------------------------------------------------------------------------------------------
// # Protobuf encoder
// Takes a byte array b and its capacity cap, encodes fields and writes them straight into b.
// Any overflow latches ok() to false; callers check it once at the end.
// ------------------------------------------------------------------------------------------------
class Out {
public:
    /**
     * Start encoding into a caller-owned buffer. Nothing is allocated.
     *
     * @param b    Output buffer; writes start at b[0].
     * @param cap  Size of `b` in bytes. Writing past it latches ok() to false.
     */
    Out(uint8_t* b, size_t cap) : b_(b), cap_(cap) {}
    /// False once any write overflowed the buffer (or a nested message was too long).
    bool ok() const { return ok_; }
    /// Bytes written so far.
    size_t size() const { return n_; }
    /// Start of the output buffer.
    uint8_t* data() { return b_; }

    /**
     * Append raw bytes as-is, with no tag or length.
     *
     * @param p  Bytes to copy.
     * @param n  Number of bytes.
     */
    void raw(const void* p, size_t n) {
        if (!ok_ || n_ + n > cap_) { ok_ = false; return; }
        std::memcpy(b_ + n_, p, n);
        n_ += n;
    }
    /**
     * Append a varint: 7 bits per byte, low bits first, high bit set = "more bytes follow".
     *
     * @param v  Value to encode (1 to 10 bytes on the wire).
     */
    void varint(uint64_t v) {
        do {
            uint8_t c = v & 0x7F;
            v >>= 7;
            if (v) c |= 0x80;
            raw(&c, 1);
        } while (v);
    }
    /**
     * Append a field tag (field number << 3 | wire type). The value must follow.
     *
     * @param field  Protobuf field number from the .proto.
     * @param wire   WireType of the value that follows.
     */
    void tag(uint32_t field, uint8_t wire) { varint((uint64_t(field) << 3) | wire); }

    // Typed fields. proto3 omits fields equal to their default (0 / empty), so we do too.
    // In all of them, `field` is the protobuf field number from the .proto.

    /// uint32 / enum field, as a varint. @param field Field number. @param v Value; 0 is omitted.
    void u32(uint32_t field, uint32_t v) { if (v) { tag(field, WIRE_VARINT); varint(v); } }
    /// fixed32 field, 4 bytes little-endian. @param field Field number. @param v Value; 0 is omitted.
    void fixed32(uint32_t field, uint32_t v) { if (v) { tag(field, WIRE_I32); raw(&v, 4); } }
    /// float field, 4 bytes. @param field Field number. @param v Value; 0.0f is omitted.
    void flt(uint32_t field, float v) { if (v != 0.0f) { tag(field, WIRE_I32); raw(&v, 4); } }
    /**
     * Length-delimited field: tag, length, then the bytes. Written even when empty.
     *
     * @param field  Field number.
     * @param p      Bytes to copy (e.g. an already-encoded sub-message).
     * @param n      Number of bytes.
     */
    void bytes(uint32_t field, const void* p, size_t n) { tag(field, WIRE_LEN); varint(n); raw(p, n); }
    /// string field. @param field Field number. @param s Null-terminated string; "" is omitted.
    void str(uint32_t field, const char* s) { const size_t n = std::strlen(s); if (n) bytes(field, s, n); }

    /**
     * Open a nested message: writes the tag and reserves a 2-byte length (messages here are < 16 KB).
     * Write the message's fields next, then close it with end().
     *
     * @param field  Field number of the nested message.
     * @return Position of the reserved length; pass it to end().
     */
    size_t begin(uint32_t field) {
        tag(field, WIRE_LEN);
        const size_t at = n_;
        raw("\x80\x00", 2);
        return at;
    }
    /**
     * Close a nested message: patches its real length into the space begin() reserved.
     *
     * @param at  The value begin() returned.
     */
    void end(size_t at) {
        if (!ok_) return;
        const size_t len = n_ - at - 2;
        if (len >= (1u << 14)) { ok_ = false; return; }   // doesn't fit in 2 varint bytes
        b_[at] = uint8_t(0x80 | (len & 0x7F));             // 2-byte varint, non-minimal is valid protobuf
        b_[at + 1] = uint8_t(len >> 7);
    }

private:
    uint8_t* b_;     // output buffer
    size_t cap_;     // buffer capacity
    size_t n_ = 0;   // bytes written
    bool ok_ = true;
};

// ------------------------------------------------------------------------------------------------
// # Protobuf decoder
// Reads fields out of a byte array b of length n, without copying.
// Usage: while (in.more() && in.next(field, wire)) { read the value, or skip(wire) it }.
// Any malformed / truncated input latches ok() to false.
// ------------------------------------------------------------------------------------------------
class In {
public:
    /**
     * Start decoding a caller-owned buffer. Nothing is copied, so `b` must outlive the In.
     *
     * @param b  Encoded message; reads b[0..n).
     * @param n  Length of `b` in bytes.
     */
    In(const uint8_t* b, size_t n) : p_(b), e_(b + n) {}
    /// True while there is input left and nothing has failed.
    bool more() const { return p_ < e_ && ok_; }
    /// False once any read hit malformed or truncated input.
    bool ok() const { return ok_; }

    /**
     * Read the next field's tag. Then read its value with the matching helper below, or skip() it.
     *
     * @param[out] field  Protobuf field number.
     * @param[out] wire   WireType of the value that follows.
     * @return False on malformed input.
     */
    bool next(uint32_t& field, uint8_t& wire) {
        uint64_t t;
        if (!varint(t)) return false;
        field = uint32_t(t >> 3);
        wire = uint8_t(t & 7);
        return true;
    }
    /**
     * Read a varint (inverse of Out::varint).
     *
     * @param[out] v  Decoded value.
     * @return False if truncated or longer than 10 bytes (64 bits).
     */
    bool varint(uint64_t& v) {
        v = 0;
        for (int shift = 0; shift < 64; shift += 7) {
            if (p_ >= e_) { ok_ = false; return false; }
            const uint8_t c = *p_++;
            v |= uint64_t(c & 0x7F) << shift;
            if (!(c & 0x80)) return true;
        }
        ok_ = false;
        return false;
    }
    /**
     * Read a fixed32 value (4 bytes little-endian).
     *
     * @param[out] v  Decoded value.
     * @return False if fewer than 4 bytes are left.
     */
    bool fixed32(uint32_t& v) {
        if (e_ - p_ < 4) { ok_ = false; return false; }
        std::memcpy(&v, p_, 4);
        p_ += 4;
        return true;
    }
    /**
     * Read a float value (a fixed32 reinterpreted as float).
     *
     * @param[out] v  Decoded value.
     * @return False if fewer than 4 bytes are left.
     */
    bool flt(float& v) {
        uint32_t u;
        if (!fixed32(u)) return false;
        std::memcpy(&v, &u, 4);   // reinterpret the 4 bytes as a float
        return true;
    }
    /**
     * Read a length-delimited value (bytes, string or nested message). Nothing is copied.
     *
     * @param[out] b  Points into the input buffer at the value's first byte.
     * @param[out] n  Length of the value in bytes. Decode a nested message with In(b, n).
     * @return False if the length is malformed or runs past the end of the input.
     */
    bool bytes(const uint8_t*& b, size_t& n) {
        uint64_t len;
        if (!varint(len) || uint64_t(e_ - p_) < len) { ok_ = false; return false; }
        b = p_;
        n = size_t(len);
        p_ += len;
        return true;
    }
    /**
     * Step over the value of a field we don't care about.
     *
     * @param wire  The WireType next() returned for this field.
     * @return False if truncated, or for wire types Kortex doesn't use (groups).
     */
    bool skip(uint8_t wire) {
        uint64_t v; uint32_t u; const uint8_t* b; size_t n;
        switch (wire) {
        case WIRE_VARINT: return varint(v);
        case WIRE_I64:    if (e_ - p_ < 8) { ok_ = false; return false; } p_ += 8; return true;
        case WIRE_LEN:    return bytes(b, n);
        case WIRE_I32:    return fixed32(u);
        default:          ok_ = false; return false;   // groups (3, 4) are not used by Kortex
        }
    }

private:
    const uint8_t* p_;   // read position
    const uint8_t* e_;   // end of input
    bool ok_ = true;
};

// ------------------------------------------------------------------------------------------------
// # Frame
// Every Kortex message is a Frame { Header, payload }. See the communication overview at the top.
// ------------------------------------------------------------------------------------------------
struct Header {
    // Raw fields, in protobuf field order 1..4 (decodeFrame relies on this order).
    uint32_t frame_info = 0;
    uint32_t message_info = 0;
    uint32_t service_info = 0;
    uint32_t payload_info = 0;

    // Bit-field accessors.
    /// FrameType: REQUEST, RESPONSE or NOTIFICATION.
    uint8_t frameType() const { return (frame_info >> 24) & 0xF; }
    /// 0 for the base, 1..7 for an actuator (e.g. SetControlMode).
    uint8_t deviceId() const { return (frame_info >> 16) & 0xFF; }
    /// 0 = success; anything else means the robot rejected the request.
    uint8_t errorCode() const { return (frame_info >> 12) & 0xF; }
    /// Detail for a non-zero errorCode().
    uint16_t errorSubCode() const { return frame_info & 0xFFF; }
    /// Session this frame belongs to (from the CreateSession response).
    uint16_t sessionId() const { return message_info >> 16; }
    /// Echoes the request's msg_id, so a response can be matched to its request.
    uint16_t messageId() const { return message_info & 0xFFFF; }
    /// Service function this frame calls or answers; compare with the Fn enum.
    uint32_t functionUid() const { return service_info & 0x0FFFFFFF; }
};

/**
 * Wrap an already-encoded payload in a request Frame.
 *
 * @param out          Buffer the frame is written into.
 * @param cap          Size of `out` in bytes.
 * @param fn           Function to call (Fn enum).
 * @param session      Session id from the CreateSession response (0 for CreateSession itself).
 * @param msg_id       Caller-chosen id, echoed in the response's messageId().
 * @param device_id    0 for the base, 1..7 to address an actuator (SetControlMode).
 * @param payload      Encoded payload, e.g. from encodeServoingMode(). May be null if payload_len is 0.
 * @param payload_len  Payload length in bytes.
 * @return Frame size in bytes, or 0 if `out` was too small.
 */
inline size_t encodeRequest(uint8_t* out, size_t cap, uint32_t fn, uint16_t session, uint16_t msg_id,
                            uint8_t device_id, const uint8_t* payload, size_t payload_len) {
    Out o(out, cap);
    const size_t h = o.begin(1);                                                           // Frame.header
    o.fixed32(1, (1u << 28) | (uint32_t(REQUEST) << 24) | (uint32_t(device_id) << 16));    //   frame_info
    o.fixed32(2, (uint32_t(session) << 16) | msg_id);                                      //   message_info
    o.fixed32(3, (1u << 28) | (fn & 0x0FFFFFFF));                                          //   service_info
    o.fixed32(4, uint32_t(payload_len) & 0xFFFFFF);                                        //   payload_info
    o.end(h);
    if (payload_len) o.bytes(2, payload, payload_len);                                     // Frame.payload
    return o.ok() ? o.size() : 0;
}

/**
 * Split a received Frame into its header and payload.
 *
 * @param      b            Received frame bytes (a UDP datagram, or a TCP frame without its 12-byte header).
 * @param      n            Length of `b` in bytes.
 * @param[out] h            Decoded header. Check h.errorCode() before trusting the payload.
 * @param[out] payload      Points into `b` (not copied); null if the frame has no payload.
 * @param[out] payload_len  Payload length in bytes.
 * @return False if the frame is malformed.
 */
inline bool decodeFrame(const uint8_t* b, size_t n, Header& h, const uint8_t*& payload, size_t& payload_len) {
    In in(b, n);
    payload = nullptr;
    payload_len = 0;
    h = Header{};
    uint32_t field; uint8_t wire;
    while (in.more() && in.next(field, wire)) {
        if (field == 1 && wire == WIRE_LEN) {                // Frame.header
            const uint8_t* hb; size_t hn;
            if (!in.bytes(hb, hn)) return false;
            In hin(hb, hn);
            uint32_t hfield; uint8_t hwire;
            while (hin.more() && hin.next(hfield, hwire)) {
                if (hwire == WIRE_I32 && hfield >= 1 && hfield <= 4) {
                    uint32_t v = 0;
                    if (!hin.fixed32(v)) return false;
                    (&h.frame_info)[hfield - 1] = v;         // fields 1..4 -> the 4 consecutive Header members
                } else if (!hin.skip(hwire)) return false;
            }
        } else if (field == 2 && wire == WIRE_LEN) {         // Frame.payload
            if (!in.bytes(payload, payload_len)) return false;
        } else if (!in.skip(wire)) return false;
    }
    return in.ok();
}

/**
 * Write the 12-byte TCP stream header that goes in front of every frame sent on port 10000.
 *
 * @param[out] out        The 12 header bytes: magic, then the frame length big-endian.
 * @param      frame_len  Size of the frame that follows (what encodeRequest() returned).
 */
inline void tcpHeader(uint8_t out[12], uint32_t frame_len) {
    std::memcpy(out, kTcpMagic, 8);                 // Copying the "KORTEX" TCP magic string to the start of the header.
    out[8] = uint8_t(frame_len >> 24);              // specifying the frame length, big-endian
    out[9] = uint8_t(frame_len >> 16);
    out[10] = uint8_t(frame_len >> 8);
    out[11] = uint8_t(frame_len);
}
/**
 * Parse a received TCP stream header.
 *
 * @param h  The first 12 bytes read from the TCP stream.
 * @return Length of the frame that follows, or -1 if the magic is wrong (stream out of sync).
 */
inline long tcpFrameLength(const uint8_t h[12]) {
    if (std::memcmp(h, kTcpMagic, 8) != 0) return -1;
    return (long(h[8]) << 24) | (long(h[9]) << 16) | (long(h[10]) << 8) | long(h[11]);
}

// ------------------------------------------------------------------------------------------------
// # Create Kortex Session
// Session.CreateSessionInfo: log in over TCP. The session id in the response is used in every
// later frame header.
// ------------------------------------------------------------------------------------------------

/**
 * Encode the CreateSession payload (Session.CreateSessionInfo). Send it with encodeRequest(), session 0.
 *
 * @param out                    Buffer the payload is written into.
 * @param cap                    Size of `out` in bytes.
 * @param user                   Login username (Kinova default "admin").
 * @param pass                   Login password (Kinova default "admin").
 * @param session_timeout_ms     The robot closes the session after this long with no traffic.
 * @param connection_timeout_ms  The robot closes the connection after this long with no traffic.
 * @return Payload size in bytes, or 0 if `out` was too small.
 */
inline size_t encodeCreateSession(uint8_t* out, size_t cap, const char* user, const char* pass,
                                  uint32_t session_timeout_ms, uint32_t connection_timeout_ms) {
    Out o(out, cap);
    o.str(1, user);                       // username
    o.u32(2, session_timeout_ms);         // session_inactivity_timeout
    o.str(3, pass);                       // password
    o.u32(4, connection_timeout_ms);      // connection_inactivity_timeout
    return o.ok() ? o.size() : 0;
}

// ------------------------------------------------------------------------------------------------
// # Set Modes
// Base.ServoingModeInformation: LOW_LEVEL lets us stream commands to the actuators over UDP.
// ActuatorConfig.ControlModeInformation: per actuator (deviceId = actuator 1..7), POSITION or TORQUE.
// ------------------------------------------------------------------------------------------------

/**
 * Encode the Base.SetServoingMode payload. Send it to device 0 (the base).
 *
 * @param out  Buffer the payload is written into.
 * @param cap  Size of `out` in bytes.
 * @param m    LOW_LEVEL to stream cyclic commands, SINGLE_LEVEL to hand control back.
 * @return Payload size in bytes, or 0 if `out` was too small.
 */
inline size_t encodeServoingMode(uint8_t* out, size_t cap, ServoingMode m) {
    Out o(out, cap);
    o.u32(1, m);                          // servoing_mode
    return o.ok() ? o.size() : 0;
}

/**
 * Encode the ActuatorConfig.SetControlMode payload. Send it once per actuator, device_id = 1..7.
 *
 * @param out  Buffer the payload is written into.
 * @param cap  Size of `out` in bytes.
 * @param m    TORQUE to follow torque_joint commands, POSITION to hold position.
 * @return Payload size in bytes, or 0 if `out` was too small.
 */
inline size_t encodeControlMode(uint8_t* out, size_t cap, ControlMode m) {
    Out o(out, cap);
    o.u32(1, m);                          // control_mode
    return o.ok() ? o.size() : 0;
}

// ------------------------------------------------------------------------------------------------
// # Cyclic Command
// BaseCyclic.Command as 00-Torque_Controller sends it: every actuator gets
// command_id = frame_id, position = measured (deg) — what an actuator still in
// POSITION mode holds — and torque_joint (Nm), used in TORQUE mode.
// gripper_pos_pct < 0: no gripper command.
// ------------------------------------------------------------------------------------------------

/**
 * Encode the BaseCyclic.Refresh payload: one command for every actuator, plus the gripper if requested.
 *
 * @param out                Buffer the payload is written into.
 * @param cap                Size of `out` in bytes.
 * @param frame_id           Cycle counter. Also used as every actuator's command_id; feedback echoes it.
 * @param n                  Number of actuators (array length of position_deg and torque_nm).
 * @param position_deg       Per-actuator position, deg. Send the measured position so that
 *                           actuators still in POSITION mode hold still.
 * @param torque_nm          Per-actuator joint torque, Nm. Used by actuators in TORQUE mode.
 * @param gripper_pos_pct    Gripper target position, 0..100 %. Negative = no gripper command.
 * @param gripper_vel_pct    Gripper speed, 0..100 %.
 * @param gripper_force_pct  Gripper force limit, 0..100 %.
 * @return Payload size in bytes, or 0 if `out` was too small.
 */
inline size_t encodeCommand(uint8_t* out, size_t cap, uint32_t frame_id, int n,
                            const float* position_deg, const float* torque_nm,
                            float gripper_pos_pct = -1.0f, float gripper_vel_pct = 0.0f,
                            float gripper_force_pct = 100.0f) {
    Out o(out, cap);
    o.fixed32(1, frame_id);                           // frame_id
    for (int i = 0; i < n; ++i) {
        const size_t a = o.begin(2);                  // actuators[i]
        o.fixed32(1, frame_id);                       //   command_id
        o.flt(3, position_deg[i]);                    //   position
        o.flt(5, torque_nm[i]);                       //   torque_joint
        o.end(a);
    }
    if (gripper_pos_pct >= 0.0f) {
        const size_t ic = o.begin(3);                 // interconnect
        const size_t id = o.begin(1);                 //   command_id { identifier = frame_id }
        o.fixed32(1, frame_id);
        o.end(id);
        const size_t gc = o.begin(3);                 //   gripper_command
        const size_t mc = o.begin(3);                 //     motor_cmd
        o.flt(3, gripper_pos_pct);                    //       position
        o.flt(4, gripper_vel_pct);                    //       velocity
        o.flt(5, gripper_force_pct);                  //       force
        o.end(mc);
        o.end(gc);
        o.end(ic);
    }
    return o.ok() ? o.size() : 0;
}

// ------------------------------------------------------------------------------------------------
// # Cyclic Feedback
// BaseCyclic.Feedback, received once per UDP cycle. Only the fields the MCU uses are kept;
// everything else is skipped. decodeFeedback is the entry point, the others decode its sub-messages.
// ------------------------------------------------------------------------------------------------
struct ActuatorFb {
    uint32_t command_id = 0;
    uint32_t status_flags = 0;
    uint32_t fault_bank_a = 0;
    uint32_t fault_bank_b = 0;
    float position = 0;        // deg
    float velocity = 0;        // deg/s
    float torque = 0;          // Nm
    float current_motor = 0;   // A
};
struct Feedback {
    uint32_t frame_id = 0;
    uint32_t arm_state = 0;                  // ArmState
    uint32_t base_fault_bank_a = 0;
    uint32_t base_fault_bank_b = 0;
    float tool_wrench[6] = {};               // tool_external_wrench_{force,torque}_{x,y,z}
    int n_actuators = 0;
    ActuatorFb act[kMaxActuators];
    bool has_gripper = false;
    float gripper_position = 0;              // %
    float gripper_current = 0;               // A
};

/**
 * Decode one ActuatorCyclic.Feedback. Every field we want is a fixed32 (uint32 or float).
 *
 * @param      b  Sub-message bytes (one actuators[] entry of the feedback).
 * @param      n  Length of `b` in bytes.
 * @param[out] a  Fields found are written; the others keep their current values.
 * @return False if the sub-message is malformed.
 */
inline bool decodeActuator(const uint8_t* b, size_t n, ActuatorFb& a) {
    In in(b, n);
    uint32_t field; uint8_t wire;
    while (in.more() && in.next(field, wire)) {
        if (wire == WIRE_I32) {
            uint32_t u;
            if (!in.fixed32(u)) return false;
            float x;
            std::memcpy(&x, &u, 4);           // same 4 bytes, read as a float
            switch (field) {
            case 1:  a.command_id = u;    break;
            case 2:  a.status_flags = u;  break;
            case 4:  a.position = x;      break;
            case 5:  a.velocity = x;      break;
            case 6:  a.torque = x;        break;
            case 7:  a.current_motor = x; break;
            case 11: a.fault_bank_a = u;  break;
            case 12: a.fault_bank_b = u;  break;
            default: break;
            }
        } else if (!in.skip(wire)) return false;
    }
    return in.ok();
}

/**
 * Decode BaseFeedback: arm state, tool wrench and base fault banks.
 *
 * @param      b   Sub-message bytes (the `base` field of the feedback).
 * @param      n   Length of `b` in bytes.
 * @param[out] fb  arm_state, tool_wrench and base_fault_bank_a/b are written; nothing else is touched.
 * @return False if the sub-message is malformed.
 */
inline bool decodeBase(const uint8_t* b, size_t n, Feedback& fb) {
    In in(b, n);
    uint32_t field; uint8_t wire;
    while (in.more() && in.next(field, wire)) {
        if (field == 2 && wire == WIRE_VARINT) {                            // active_state
            uint64_t v;
            if (!in.varint(v)) return false;
            fb.arm_state = uint32_t(v);
        } else if (wire == WIRE_I32) {
            uint32_t u;
            if (!in.fixed32(u)) return false;
            if (field >= 25 && field <= 30) std::memcpy(&fb.tool_wrench[field - 25], &u, 4);   // wrench, as floats
            else if (field == 31) fb.base_fault_bank_a = u;
            else if (field == 32) fb.base_fault_bank_b = u;
        } else if (!in.skip(wire)) return false;
    }
    return in.ok();
}

/**
 * Decode InterconnectCyclic.Feedback -> gripper_feedback -> first motor: position and current.
 *
 * @param      b   Sub-message bytes (the `interconnect` field of the feedback).
 * @param      n   Length of `b` in bytes.
 * @param[out] fb  gripper_position and gripper_current are written, and has_gripper is set,
 *                 if a gripper motor is present; nothing else is touched.
 * @return False if the sub-message is malformed.
 */
inline bool decodeGripper(const uint8_t* b, size_t n, Feedback& fb) {
    In in(b, n);                                                // InterconnectCyclic.Feedback
    uint32_t field; uint8_t wire;
    while (in.more() && in.next(field, wire)) {
        if (field == 18 && wire == WIRE_LEN) {                  // gripper_feedback
            const uint8_t* gb; size_t gn;
            if (!in.bytes(gb, gn)) return false;
            In gin(gb, gn);
            uint32_t gfield; uint8_t gwire;
            while (gin.more() && gin.next(gfield, gwire)) {
                if (gfield == 7 && gwire == WIRE_LEN && !fb.has_gripper) {   // motor[], first one only
                    const uint8_t* mb; size_t mn;
                    if (!gin.bytes(mb, mn)) return false;
                    In min(mb, mn);
                    uint32_t mfield; uint8_t mwire;
                    while (min.more() && min.next(mfield, mwire)) {
                        if (mwire == WIRE_I32 && (mfield == 4 || mfield == 7)) {   // position, current_motor
                            float x;
                            if (!min.flt(x)) return false;
                            (mfield == 4 ? fb.gripper_position : fb.gripper_current) = x;
                        } else if (!min.skip(mwire)) return false;
                    }
                    fb.has_gripper = true;
                } else if (!gin.skip(gwire)) return false;
            }
        } else if (!in.skip(wire)) return false;
    }
    return in.ok();
}

/**
 * Decode a BaseCyclic.Feedback: the payload of each cyclic UDP response. This is the entry point;
 * it calls the decoders above for each sub-message.
 *
 * @param      b   Feedback payload (the `payload` decodeFrame() returned).
 * @param      n   Length of `b` in bytes.
 * @param[out] fb  Reset, then filled in. Actuators beyond kMaxActuators are ignored.
 * @return False if the payload is malformed; `fb` may then be partly filled.
 */
inline bool decodeFeedback(const uint8_t* b, size_t n, Feedback& fb) {
    fb = Feedback{};
    In in(b, n);
    uint32_t field; uint8_t wire;
    while (in.more() && in.next(field, wire)) {
        const uint8_t* sb; size_t sn;                                       // sub-message bytes
        if (field == 1 && wire == WIRE_I32) {                               // frame_id
            if (!in.fixed32(fb.frame_id)) return false;
        } else if (field == 2 && wire == WIRE_LEN) {                        // base
            if (!in.bytes(sb, sn) || !decodeBase(sb, sn, fb)) return false;
        } else if (field == 3 && wire == WIRE_LEN) {                        // actuators[]
            if (!in.bytes(sb, sn)) return false;
            if (fb.n_actuators < kMaxActuators && !decodeActuator(sb, sn, fb.act[fb.n_actuators++])) return false;
        } else if (field == 4 && wire == WIRE_LEN) {                        // interconnect
            if (!in.bytes(sb, sn) || !decodeGripper(sb, sn, fb)) return false;
        } else if (!in.skip(wire)) return false;
    }
    return in.ok();
}

}  // namespace kortex
}  // namespace fic

#endif  // FIC_KORTEX_MIN_H
