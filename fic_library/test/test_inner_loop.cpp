// Host tests for the portable MCU-side code: FIC-link framing, and the
// InnerLoop state machine (Running / Hold / Ramp / e-stop / faults / guards).
// No Eigen, no ROS:   see fic_library/test/CMakeLists.txt

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>

#include <fic/fic_link.h>
#include <fic/inner_loop.h>
#include <fic/mcu_node.h>

using namespace fic::link;
using fic::mcu::ArmCommand;
using fic::mcu::ArmFeedback;
using fic::mcu::InnerLoop;

static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); ++g_fail; } } while (0)

static const int N = 7;

// Decoupled joints with a known gravity, integrated at 1 kHz.
struct Plant {
    double q[N], qd[N];
    double G[N] = {10, 30, 5, 15, 2, 3, 0.5};
    Plant() { for (int i = 0; i < N; ++i) { q[i] = 0.3 * (i % 3) - 0.2; qd[i] = 0; } }
    double g(int i, const double* qq) const { return G[i] * std::sin(qq[i]); }
    double dg(int i) const { return G[i] * std::cos(q[i]); }
    void step(const ArmCommand& c, double dt) {
        for (int i = 0; i < N; ++i) {
            // Out of torque mode the arm's own controller holds it.
            double tau = c.torque_mode ? c.tau[i] : g(i, q);
            double qdd = (tau - g(i, q) - 2.0 * qd[i]) / 0.5;
            qd[i] += qdd * dt;
            q[i] += qd[i] * dt;
        }
    }
    ArmFeedback fb() const {
        ArmFeedback f;
        f.valid = true;
        for (int i = 0; i < N; ++i) { f.q[i] = (float)q[i]; f.tau[i] = (float)g(i, q); }
        return f;
    }
};

static Config makeConfig(uint32_t epoch = 7) {
    Config c{};
    c.config_epoch = epoch;
    c.adapter = (uint32_t)Adapter::Loopback;
    c.n_joints = N;
    c.hold_after_s = 0.05f;
    c.ramp_after_s = 2.0f;
    c.deadband_joint_tau = 1.5f;
    c.dt = 0.001f;
    c.torque_cutoff_hz = 200;
    c.state_cutoff_hz = 10;
    c.damping_alpha = std::exp(-2.0 * M_PI * 20.0 / 1000.0);
    c.restart_on_stall = 1; c.planner_max_hz = 8.75f; c.planner_speed_margin = 1.5708f; c.joint_speed_limit = 0;
    for (int i = 0; i < N; ++i) {
        c.lower[i] = -3.0f; c.upper[i] = 3.0f;
        c.fault_lower[i] = -2.5f; c.fault_upper[i] = 2.5f;
        c.torque_limit[i] = 39.0f;
        c.torque_rate_limit[i] = 2.0f;
        c.torque_sign[i] = 1.0f;
    }
    return c;
}

static Params makeParams(uint32_t epoch, int j) {
    Params p{};
    p.param_epoch = epoch;
    p.joint = (uint16_t)j;
    p.n_joints = N;
    p.p.stiffness = 100;
    p.p.n_saturations = 2; p.p.n_neg_sat = 1; p.p.asymmetric = 0;
    p.p.n_error_max = 1; p.p.error_max[0] = 0.4;
    p.p.n_force_max = 1; p.p.force_max[0] = 30;
    p.p.planner_stiffness = 15000; p.p.vel_desired = 0.5; p.p.acc_max = 5000;
    p.p.damping_ratio = 0.7; p.p.frequency = 20;
    p.p.inv_m = 1.0; p.p.adm_damping = 10;
    p.p.dflow[0] = -5; p.p.dflow[1] = 5; p.p.flow[0] = -1; p.p.flow[1] = 1;
    return p;
}

static CmdMsg makeCmd(const Plant& pl, InnerMode mode, uint8_t flags, uint32_t pe, const double* qt) {
    CmdMsg m;
    m.config_epoch = 7;
    m.param_epoch = pe;
    m.mode = mode;
    m.flags = flags | kCmdHasGravJac;
    m.n = N;
    for (int i = 0; i < N; ++i) {
        m.q_target[i] = (float)qt[i];
        m.q0[i] = (float)pl.q[i];
        m.g0[i] = (float)pl.g(i, pl.q);
        m.dgdq[i * N + i] = (float)pl.dg(i);
    }
    return m;
}

static void testLinkRoundTrip() {
    std::printf("link round trip\n");
    uint8_t buf[kMaxDatagram];
    CmdMsg a;
    a.config_epoch = 3; a.param_epoch = 9; a.mode = InnerMode::Hapfic; a.n = 16;
    a.flags = kCmdEnable | kCmdHasGravJac;
    for (int i = 0; i < 16; ++i) { a.q_target[i] = i * 0.1f; a.tau_des[i] = -i; a.kd[i] = 0.5f * i; }
    for (int i = 0; i < 256; ++i) a.dgdq[i] = i * 0.01f;
    size_t n = encodeCmd(a, 42, buf, sizeof(buf));
    CHECK(n > 0 && n <= kMaxDatagram);
    Reader r;
    CmdMsg b;
    CHECK(r.open(buf, n) && r.type() == MsgType::Cmd && r.seq() == 42);
    CHECK(decodeCmd(r, b));
    CHECK(b.mode == InnerMode::Hapfic && b.n == 16 && b.param_epoch == 9);
    CHECK(std::memcmp(a.q_target, b.q_target, sizeof(a.q_target)) == 0);
    CHECK(std::memcmp(a.kd, b.kd, sizeof(a.kd)) == 0);
    CHECK(std::memcmp(a.dgdq, b.dgdq, sizeof(a.dgdq)) == 0);
    buf[20] ^= 0x40;                                   // corrupt one bit
    CHECK(!r.open(buf, n));
    buf[20] ^= 0x40;
    CHECK(!r.open(buf, n - 1));                        // truncated

    StateMsg s;
    s.n = 7; s.state = McuState::Hold; s.faults = kFaultOverrun;
    for (int i = 0; i < 7; ++i) { s.q_raw[i] = i; s.tau_cmd[i] = -i; }
    s.wrench[5] = 3.5f;
    n = encodeState(s, 1, buf, sizeof(buf));
    StateMsg t;
    CHECK(r.open(buf, n) && decodeState(r, t));
    CHECK(t.state == McuState::Hold && t.faults == kFaultOverrun && t.wrench[5] == 3.5f && t.q_raw[6] == 6);
}

// FIC-link v3: H-PMP's Jacobian and the joint-target speed in CMD; a v2 datagram refused.
static void testLinkV3() {
    std::printf("link v3\n");
    uint8_t buf[kMaxDatagram];
    CmdMsg m;
    m.config_epoch = 1; m.param_epoch = 2; m.mode = InnerMode::Hapfic; m.n = 7;
    m.flags = kCmdEnable | kCmdHasJacobian | kCmdHasGravJac;
    m.joint_target_speed = 0.5f;
    for (int k = 0; k < 6 * 7; ++k) m.jac[k] = 0.01f * k;
    for (int k = 0; k < 49; ++k) m.dgdq[k] = -0.02f * k;
    size_t len = encodeCmd(m, 5, buf, sizeof(buf));
    CHECK(len > 0);
    Reader r;
    CmdMsg d;
    CHECK(r.open(buf, len) && decodeCmd(r, d));
    CHECK(d.flags == m.flags && d.joint_target_speed == 0.5f);
    CHECK(d.jac[41] == m.jac[41] && d.dgdq[48] == m.dgdq[48]);
    // Both blocks for 16 joints do not fit one datagram: encode refuses (the client then drops dg/dq).
    m.n = 16;
    CHECK(encodeCmd(m, 6, buf, sizeof(buf)) == 0);
    m.flags &= static_cast<uint8_t>(~kCmdHasGravJac);
    len = encodeCmd(m, 7, buf, sizeof(buf));
    CHECK(len > 0);
    // A v2 datagram is refused.
    Header h;
    std::memcpy(&h, buf, sizeof(h));
    h.version = 2;
    std::memcpy(buf, &h, sizeof(h));
    CHECK(!r.open(buf, len));
}

// FIC-link: the simulator in lockstep with the inner loop (SimStep plant -> loop, SimTorque loop -> plant).
static void testLinkSim() {
    std::printf("link sim step\n");
    CHECK(adapterFromName("sim") == Adapter::HostSim);
    uint8_t buf[kMaxDatagram];
    for (int nj : {7, 16}) {
        SimStepMsg a;
        a.t_us = 123456789ull; a.n = (uint16_t)nj;
        for (int i = 0; i < nj; ++i) { a.q[i] = 0.1f * i; a.tau[i] = -2.0f * i; }
        a.wrench[5] = 1.5f;
        size_t len = encodeSimStep(a, 3, buf, sizeof(buf));
        CHECK(len > 0);
        Reader r;
        SimStepMsg b;
        CHECK(r.open(buf, len) && r.type() == MsgType::SimStep && decodeSimStep(r, b));
        CHECK(b.t_us == a.t_us && b.n == nj && b.q[nj - 1] == a.q[nj - 1] && b.tau[nj - 1] == a.tau[nj - 1] &&
              b.wrench[5] == 1.5f);
        CHECK(!r.open(buf, len - 1));
        SimTorqueMsg x;
        x.step = 77; x.torque_mode = 1; x.n = (uint16_t)nj;
        for (int i = 0; i < nj; ++i) x.tau[i] = 3.0f + i;
        len = encodeSimTorque(x, 4, buf, sizeof(buf));
        SimTorqueMsg y;
        CHECK(len > 0 && r.open(buf, len) && r.type() == MsgType::SimTorque && decodeSimTorque(r, y));
        CHECK(y.step == 77 && y.torque_mode == 1 && y.n == nj && y.tau[nj - 1] == x.tau[nj - 1]);
    }
}

// Drives an InnerLoop through the full lifecycle against the plant.
static void testLifecycle() {
    std::printf("inner loop lifecycle\n");
    static InnerLoop L;   // ~35 KB: static, as on the MCU
    Plant pl;
    ArmCommand out;
    uint64_t t = 0;
    const double dt = 0.001;
    auto tickN = [&](int k) { for (int j = 0; j < k; ++j) { t += 1000; L.tick(pl.fb(), t, out); pl.step(out, dt); } };

    L.configure(makeConfig());
    CHECK(L.state() == McuState::Connecting);
    tickN(10);
    CHECK(!out.torque_mode);
    L.adapterReady(true);
    CHECK(L.state() == McuState::Ready);
    tickN(10);
    CHECK(!out.torque_mode);                           // no CMD: never enter torque mode

    for (int j = 0; j < N; ++j) L.acceptParams(makeParams(1, j));
    double qt[N];
    for (int i = 0; i < N; ++i) qt[i] = pl.q[i] + 0.2;
    // Stream CMDs for 3 s.
    for (int k = 0; k < 3000; ++k) {
        L.onCmd(makeCmd(pl, InnerMode::ImpedancePlanner, kCmdEnable, 1, qt), k + 1, t);
        double prev[N];
        for (int i = 0; i < N; ++i) prev[i] = out.tau[i];
        tickN(1);
        if (k > 0 && out.torque_mode)
            for (int i = 0; i < N; ++i) CHECK(std::fabs(out.tau[i] - prev[i]) <= 2.0f + 1e-4f);
    }
    CHECK(L.state() == McuState::Running);
    CHECK(out.torque_mode);
    CHECK(!L.released() && L.wantsTorque(t));
    for (int i = 0; i < N; ++i) CHECK(std::isfinite(out.tau[i]) && std::fabs(out.tau[i]) <= 39.0f);
    double err = 0;
    for (int i = 0; i < N; ++i) err = std::fmax(err, std::fabs(qt[i] - pl.q[i]));
    std::printf("  tracking error after 3 s: %.4f rad\n", err);
    CHECK(err < 0.15);

    // Linux goes quiet: HOLD after 50 ms, RAMP after 2 s, then out of torque mode.
    tickN(40);
    CHECK(L.state() == McuState::Running);
    tickN(20);
    CHECK(L.state() == McuState::Hold);
    CHECK(out.torque_mode);
    double q_hold[N];
    for (int i = 0; i < N; ++i) q_hold[i] = pl.q[i];
    tickN(1900);
    CHECK(L.state() == McuState::Hold);
    double drift = 0;
    for (int i = 0; i < N; ++i) drift = std::fmax(drift, std::fabs(pl.q[i] - q_hold[i]));
    std::printf("  drift during 1.9 s HOLD: %.4f rad\n", drift);
    CHECK(drift < 0.05);
    tickN(100);
    CHECK(L.state() == McuState::Ramp || L.state() == McuState::Ready);
    tickN(3000);
    CHECK(L.state() == McuState::Ready);
    CHECK(!out.torque_mode);
    CHECK(L.released() && !L.wantsTorque(t));          // the adapter hands the arm back now

    // Linux back: fresh CMDs re-enter Running.
    L.onCmd(makeCmd(pl, InnerMode::Impedance, kCmdEnable, 1, qt), 4999, t);
    CHECK(L.wantsTorque(t));                           // after a release: reopen the arm session
    for (int k = 0; k < 20; ++k) { L.onCmd(makeCmd(pl, InnerMode::Impedance, kCmdEnable, 1, qt), 5000 + k, t); tickN(1); }
    CHECK(L.state() == McuState::Running);
    CHECK(!L.released());                              // a new Running entry is not released

    // E-stop: ramp out and stay out until re-armed (Enable dropped).
    for (int k = 0; k < 3000; ++k) {
        L.onCmd(makeCmd(pl, InnerMode::Impedance, kCmdEnable | kCmdEstop, 1, qt), 6000 + k, t);
        tickN(1);
    }
    CHECK(L.state() == McuState::Ready && !out.torque_mode);
    CHECK(L.faults() & kFaultEstop);
    CHECK(L.released() && !L.wantsTorque(t));
    for (int k = 0; k < 10; ++k) { L.onCmd(makeCmd(pl, InnerMode::Impedance, kCmdEnable, 1, qt), 9000 + k, t); tickN(1); }
    CHECK(L.state() == McuState::Ready);               // latched: Enable alone is not enough
    L.onCmd(makeCmd(pl, InnerMode::Impedance, 0, 1, qt), 9100, t); tickN(1);
    CHECK(!(L.faults() & kFaultEstop));
    for (int k = 0; k < 10; ++k) { L.onCmd(makeCmd(pl, InnerMode::Impedance, kCmdEnable, 1, qt), 9200 + k, t); tickN(1); }
    CHECK(L.state() == McuState::Running);

    // NaN from the outer loop never reaches the arm.
    {
        CmdMsg m = makeCmd(pl, InnerMode::Torque, kCmdEnable, 1, qt);
        m.tau_des[2] = NAN;
        L.onCmd(m, 9300, t); tickN(1);
        CHECK(std::isfinite(out.tau[2]));
    }

    // Arm feedback lost for > 50 ticks: Fault, stop commanding.
    ArmFeedback bad;
    for (int k = 0; k < 60; ++k) {
        L.onCmd(makeCmd(pl, InnerMode::Impedance, kCmdEnable, 1, qt), 9400 + k, t);
        t += 1000; L.tick(bad, t, out);
    }
    CHECK(L.state() == McuState::Fault && !out.torque_mode && (L.faults() & kFaultArmLink));
}

static void testJointLimitFault() {
    std::printf("joint-limit hard fault\n");
    static InnerLoop L;
    Plant pl;
    ArmCommand out;
    uint64_t t = 0;
    L.configure(makeConfig());
    L.adapterReady(true);
    for (int j = 0; j < N; ++j) L.acceptParams(makeParams(1, j));
    double qt[N];
    for (int i = 0; i < N; ++i) qt[i] = pl.q[i];
    for (int k = 0; k < 100; ++k) {
        L.onCmd(makeCmd(pl, InnerMode::Impedance, kCmdEnable, 1, qt), k + 1, t);
        t += 1000; L.tick(pl.fb(), t, out); pl.step(out, 0.001);
    }
    CHECK(L.state() == McuState::Running);
    pl.q[1] = 2.8;                                      // past fault_upper 2.5
    for (int k = 0; k < 300; ++k) {                     // let the 10 Hz filter catch up
        L.onCmd(makeCmd(pl, InnerMode::Impedance, kCmdEnable, 1, qt), 200 + k, t);
        t += 1000; L.tick(pl.fb(), t, out);
    }
    CHECK(L.faults() & kFaultJointLimit);
    CHECK(out.torque_mode);
    // Gravity only (rate-limited toward it; after 300 ticks it has arrived).
    CHECK(std::fabs(out.tau[1] - pl.g(1, pl.q)) < 0.5);
}

// Modes 3, 4 and hpmp at rest, nothing pushing: the measured torque is gravity (in the
// hardware convention, joint_torques_ = -g at rest, so torque_sign -1 with this plant).
// The admittances are driven by the interaction torque alone, so their offsets stay at 0,
// and gravity is fed forward, so the joint law carries none of the weight: the arm holds.
static void testAdmittanceHold() {
    std::printf("admittance modes hold at rest\n");
    static InnerLoop L[3];
    const InnerMode modes[3] = {InnerMode::Admittance, InnerMode::AdmittanceHaptic, InnerMode::Hapfic};
    const char* names[3] = {"mode 3", "mode 4", "hpmp"};
    for (int m = 0; m < 3; ++m) {
        Plant pl;
        ArmCommand out;
        uint64_t t = 0;
        Config c = makeConfig();
        for (int i = 0; i < N; ++i) c.torque_sign[i] = -1.0f;
        L[m].configure(c);
        L[m].adapterReady(true);
        for (int j = 0; j < N; ++j) L[m].acceptParams(makeParams(1, j));
        double qt[N], q0[N];
        for (int i = 0; i < N; ++i) qt[i] = q0[i] = pl.q[i];
        for (int k = 0; k < 3000; ++k) {
            L[m].onCmd(makeCmd(pl, modes[m], kCmdEnable | kCmdUsePlanner, 1, qt), k + 1, t);
            t += 1000; L[m].tick(pl.fb(), t, out); pl.step(out, 0.001);
        }
        CHECK(L[m].state() == McuState::Running && out.torque_mode);
        double drift = 0;
        for (int i = 0; i < N; ++i) drift = std::fmax(drift, std::fabs(pl.q[i] - q0[i]));
        std::printf("  %s: largest joint drift after 3 s %.2g rad\n", names[m], drift);
        CHECK(drift < 0.005);
    }
}

// Configured, adapter up, params applied, Running in `mode` holding the plant where it is.
static void toRunning(InnerLoop& L, Plant& pl, uint64_t& t, InnerMode mode) {
    ArmCommand out;
    L.configure(makeConfig());
    L.adapterReady(true);
    for (int j = 0; j < N; ++j) L.acceptParams(makeParams(1, j));
    double qt[N];
    for (int i = 0; i < N; ++i) qt[i] = pl.q[i];
    for (int k = 0; k < 20; ++k) {
        L.onCmd(makeCmd(pl, mode, kCmdEnable, 1, qt), k + 1, t);
        t += 1000; L.tick(pl.fb(), t, out); pl.step(out, 0.001);
    }
}

// Modes 1/2: the law's target moves at most joint_target_speed per tick; 0 = the commanded target at once;
// a mode change re-seeds it at the measured joints.
static void testTargetSpeed() {
    std::printf("joint-target speed limit\n");
    static InnerLoop L;
    Plant pl;
    uint64_t t = 0;
    ArmCommand out;
    toRunning(L, pl, t, InnerMode::Impedance);
    const double q_start = pl.q[0];
    double qt[N];
    for (int i = 0; i < N; ++i) qt[i] = pl.q[i];
    qt[0] = q_start + 1.0;                                     // a target 1 rad away
    CmdMsg m = makeCmd(pl, InnerMode::Impedance, kCmdEnable, 1, qt);
    m.joint_target_speed = 0.5f;                               // rad/s
    for (int k = 0; k < 200; ++k) {                            // 0.2 s: 0.1 rad
        m.q0[0] = (float)pl.q[0];
        L.onCmd(m, 100 + k, t);
        t += 1000; L.tick(pl.fb(), t, out); pl.step(out, 0.001);
    }
    const double moved = L.slewedTarget(0) - q_start;
    std::printf("  target moved %.4f rad in 0.2 s at 0.5 rad/s\n", moved);
    CHECK(moved > 0.09 && moved < 0.11);
    m.mode = InnerMode::ImpedancePlanner;                      // mode change: re-seeded at the joint
    L.onCmd(m, 400, t);
    t += 1000; L.tick(pl.fb(), t, out); pl.step(out, 0.001);
    StateMsg st;
    L.fillState(st, 0);
    CHECK(std::fabs(L.slewedTarget(0) - st.q[0]) < 0.0005 + 1e-6);   // seeded at the filtered joint, one step on
    m.joint_target_speed = 0.0f;                               // off: the commanded target at once
    L.onCmd(m, 401, t);
    t += 1000; L.tick(pl.fb(), t, out);
    CHECK(std::fabs(L.slewedTarget(0) - qt[0]) < 1e-6);
}

// H-PMP (Hapfic) with the Jacobian runs the split drive; a CMD without it falls back to the per-joint deadband.
static void testHapficSplitFallback() {
    std::printf("hpmp split drive and its fallback\n");
    static InnerLoop L;
    Plant pl;
    uint64_t t = 0;
    ArmCommand out;
    toRunning(L, pl, t, InnerMode::Hapfic);
    double qt[N];
    for (int i = 0; i < N; ++i) qt[i] = pl.q[i];
    CmdMsg m = makeCmd(pl, InnerMode::Hapfic, kCmdEnable | kCmdHasJacobian, 1, qt);
    for (int k = 0; k < 6 * N; ++k) m.jac[k] = (k % (N + 1) == 0) ? 1.0f : 0.0f;
    for (int k = 0; k < 50; ++k) {
        L.onCmd(m, 100 + k, t);
        t += 1000; L.tick(pl.fb(), t, out); pl.step(out, 0.001);
    }
    CHECK(L.usedSplit());
    m.flags = static_cast<uint8_t>(m.flags & ~kCmdHasJacobian);
    L.onCmd(m, 200, t);
    t += 1000; L.tick(pl.fb(), t, out);
    CHECK(!L.usedSplit());
    for (int i = 0; i < N; ++i) CHECK(std::isfinite(out.tau[i]));
    m.mode = InnerMode::Impedance;                             // another mode: no split
    L.onCmd(m, 201, t);
    t += 1000; L.tick(pl.fb(), t, out);
    CHECK(!L.usedSplit());
}

// STATE carries the law's torque before the output guard (tau_law: what fic_simulation recorded as
// fic_torque_commands, before the velocity damper and the clamp) next to tau_cmd.
static void testTauLaw() {
    std::printf("tau_law before the guard\n");
    static InnerLoop L;
    Plant pl;
    uint64_t t = 0;
    ArmCommand out;
    toRunning(L, pl, t, InnerMode::Impedance);
    double qt[N];
    for (int i = 0; i < N; ++i) qt[i] = pl.q[i] + 0.1;
    CmdMsg m = makeCmd(pl, InnerMode::Impedance, kCmdEnable, 1, qt);
    for (int i = 0; i < N; ++i) m.kd[i] = 5.0f;            // a damper: tau_cmd differs from tau_law
    for (int k = 0; k < 200; ++k) { L.onCmd(m, 600 + k, t); t += 1000; L.tick(pl.fb(), t, out); pl.step(out, 0.001); }
    StateMsg s;
    L.fillState(s, 0);
    double diff = 0, law = 0;
    for (int i = 0; i < N; ++i) { diff = std::fmax(diff, std::fabs(s.tau_law[i] - s.tau_cmd[i])); law += std::fabs(s.tau_law[i]); }
    CHECK(law > 1.0 && diff > 1e-4);
    uint8_t buf[kMaxDatagram];
    size_t len = encodeState(s, 9, buf, sizeof(buf));
    Reader r; StateMsg d;
    CHECK(r.open(buf, len) && decodeState(r, d) && d.tau_law[3] == s.tau_law[3]);
}

static void testNodeDispatch() {
    std::printf("mcu node dispatch\n");
    static fic::mcu::McuNode node("test-fw", 3, adapterBit(Adapter::Loopback));
    uint8_t buf[kMaxDatagram], reply[kMaxDatagram];
    Hello h{0xABCD};
    size_t n = encodeFixed(MsgType::Hello, h, 1, buf, sizeof(buf));
    size_t r = node.onDatagram(buf, n, 0, reply, sizeof(reply));
    Reader rd;
    HelloAck a;
    CHECK(r > 0 && rd.open(reply, r) && rd.type() == MsgType::HelloAck && decodeFixed(rd, a));
    CHECK(a.nonce == 0xABCD && a.fw_version == 3 && (a.adapters & adapterBit(Adapter::Loopback)));
    CHECK(std::strcmp(a.fw_name, "test-fw") == 0);

    Config c = makeConfig(11);
    n = encodeFixed(MsgType::Config, c, 2, buf, sizeof(buf));
    node.onDatagram(buf, n, 0, reply, sizeof(reply));
    CHECK(node.hasConfig() && node.configChanges() == 1);
    node.onDatagram(buf, n, 0, reply, sizeof(reply));      // duplicate: ignored
    CHECK(node.configChanges() == 1);
    c.adapter = (uint32_t)Adapter::Kortex;                 // not supported by this node
    c.config_epoch = 12;
    n = encodeFixed(MsgType::Config, c, 3, buf, sizeof(buf));
    node.onDatagram(buf, n, 0, reply, sizeof(reply));
    CHECK(node.configChanges() == 1 && node.badDatagrams() == 1);
}

// A direct Estop message (the Studio's E-STOP, not through fic_control): RAMP at once, latched like a CMD's
// e-stop flag; a CMD without Enable re-arms. Adapter::Shm is the PC inner loop's adapter.
static void testDirectEstop() {
    std::printf("direct estop\n");
    CHECK(adapterFromName("shm") == Adapter::Shm);
    static fic::mcu::McuNode node("test-fw", 3, adapterBit(Adapter::Loopback) | adapterBit(Adapter::Shm));
    uint8_t buf[kMaxDatagram], reply[kMaxDatagram];
    Config c = makeConfig(7);
    c.adapter = (uint32_t)Adapter::Shm;
    size_t n = encodeFixed(MsgType::Config, c, 1, buf, sizeof(buf));
    node.onDatagram(buf, n, 0, reply, sizeof(reply));
    CHECK(node.hasConfig());
    node.loop().adapterReady(true);
    for (int j = 0; j < N; ++j) {
        Params p = makeParams(1, j);
        n = encodeFixed(MsgType::Params, p, 2 + j, buf, sizeof(buf));
        node.onDatagram(buf, n, 0, reply, sizeof(reply));
    }
    Plant pl;
    ArmCommand out;
    uint64_t t = 0;
    double qt[N];
    for (int i = 0; i < N; ++i) qt[i] = pl.q[i];
    for (int k = 0; k < 50; ++k) {
        node.loop().onCmd(makeCmd(pl, InnerMode::Impedance, kCmdEnable, 1, qt), 100 + k, t);
        t += 1000; node.tick(pl.fb(), t, out); pl.step(out, 0.001);
    }
    CHECK(node.loop().state() == McuState::Running);
    EstopMsg e{1};
    n = encodeFixed(MsgType::Estop, e, 200, buf, sizeof(buf));
    CHECK(n > 0);
    node.onDatagram(buf, n, t, reply, sizeof(reply));
    t += 1000; node.tick(pl.fb(), t, out);
    // RAMP, or already through it (at rest the torque is at gravity, so the ramp can end in that tick).
    CHECK(node.loop().state() == McuState::Ramp ||
          (node.loop().state() == McuState::Ready && node.loop().released()));
    CHECK(node.loop().faults() & kFaultEstop);
    for (int k = 0; k < 20; ++k) {                                                     // never back to Running
        node.loop().onCmd(makeCmd(pl, InnerMode::Impedance, kCmdEnable, 1, qt), 210 + k, t);
        t += 1000; node.tick(pl.fb(), t, out);
        CHECK(node.loop().state() != McuState::Running);
    }
    node.loop().onCmd(makeCmd(pl, InnerMode::Impedance, kCmdEnable, 1, qt), 300, t);   // still latched
    CHECK(node.loop().faults() & kFaultEstop);
    // A direct E-STOP is not re-armed by a CMD without Enable (fic_control's own Stop sends those): only a
    // new CONFIG (Start) clears it (review 19 Oct, I9).
    node.loop().onCmd(makeCmd(pl, InnerMode::Impedance, 0, 1, qt), 301, t);
    CHECK(node.loop().faults() & kFaultEstop);
    c.config_epoch = 8;
    n = encodeFixed(MsgType::Config, c, 302, buf, sizeof(buf));
    node.onDatagram(buf, n, t, reply, sizeof(reply));
    CHECK(!(node.loop().faults() & kFaultEstop));
}

// The operator's joint speed cap reaches the inner loop live, in every CMD (review 19 Oct, I5).
static void testSpeedCapInCmd() {
    std::printf("joint speed cap in CMD\n");
    static InnerLoop L;
    Plant pl;
    uint64_t t = 0;
    ArmCommand out;
    toRunning(L, pl, t, InnerMode::ImpedancePlanner);
    CHECK(L.jointSpeedLimit() == 0.0);
    double qt[N];
    for (int i = 0; i < N; ++i) qt[i] = pl.q[i];
    CmdMsg m = makeCmd(pl, InnerMode::ImpedancePlanner, kCmdEnable, 1, qt);
    m.joint_speed_limit = 0.4f;
    L.onCmd(m, 500, t);
    t += 1000; L.tick(pl.fb(), t, out);
    CHECK(std::fabs(L.jointSpeedLimit() - 0.4) < 1e-6);
    m.joint_speed_limit = 0.0f;                                // 0: no cap
    L.onCmd(m, 501, t);
    t += 1000; L.tick(pl.fb(), t, out);
    CHECK(L.jointSpeedLimit() == 0.0);
}

// RAMP with velocity damping and sensor noise, as on the real Gen3: the ramp
// must decay its own torque, not re-integrate the guard's -kd*v every tick
// (that is a stiff, lagging spring: the arm shook and the ramp never ended).
static void testRampWithDamping() {
    std::printf("ramp with damping and sensor noise\n");
    static InnerLoop L;
    Plant pl;
    ArmCommand out;
    uint64_t t = 0;
    uint32_t rng = 12345;
    auto noise = [&] { rng = rng * 1664525u + 1013904223u; return ((rng >> 8) / 16777216.0 - 0.5) * 0.002; };
    auto noisyFb = [&] { ArmFeedback f = pl.fb(); for (int i = 0; i < N; ++i) f.q[i] += (float)noise(); return f; };
    auto tickN = [&](int k) { for (int j = 0; j < k; ++j) { t += 1000; L.tick(noisyFb(), t, out); pl.step(out, 0.001); } };

    L.configure(makeConfig());
    tickN(5);
    L.adapterReady(true);
    for (int j = 0; j < N; ++j) L.acceptParams(makeParams(1, j));
    double qt[N];
    for (int i = 0; i < N; ++i) qt[i] = pl.q[i];
    for (int k = 0; k < 1000; ++k) {
        CmdMsg m = makeCmd(pl, InnerMode::ImpedancePlanner, kCmdEnable, 1, qt);
        for (int i = 0; i < N; ++i) m.kd[i] = 2.0f;
        L.onCmd(m, k + 1, t);
        tickN(1);
    }
    CHECK(L.state() == McuState::Running);
    // Linux goes quiet: HOLD, then RAMP from 2 s.
    tickN(2100);
    CHECK(L.state() == McuState::Ramp || L.state() == McuState::Ready);
    double worst = 0;
    int ramp_ms = 0;
    while (L.state() == McuState::Ramp && ramp_ms < 5000) {
        tickN(1);
        ++ramp_ms;
        if (out.torque_mode)
            for (int i = 0; i < N; ++i) worst = std::fmax(worst, std::fabs(out.tau[i] - pl.g(i, pl.q)));
    }
    std::printf("  ramp finished after %d ms; largest |tau - g| during it %.2f Nm\n", ramp_ms, worst);
    CHECK(L.state() == McuState::Ready);
    CHECK(ramp_ms < 3000);
    CHECK(worst < 3.0);
    CHECK(L.released());
}

// A CONFIG made for one step (the node's 2 ms) run at the simulator's (1 ms): the per-tick filter
// coefficient keeps its cutoff in Hz, the step becomes the simulator's.
static void testRetimeConfig() {
    Config c{};
    c.dt = 0.002f;
    c.damping_alpha = (float)std::exp(-2.0 * M_PI * 20.0 * 0.002);
    fic::mcu::retimeConfig(c, 0.001f);
    CHECK(std::fabs(c.dt - 0.001f) < 1e-9f);
    CHECK(std::fabs(c.damping_alpha - (float)std::exp(-2.0 * M_PI * 20.0 * 0.001)) < 1e-6f);
    fic::mcu::retimeConfig(c, 0.001f);              // already at that step: unchanged
    CHECK(std::fabs(c.damping_alpha - (float)std::exp(-2.0 * M_PI * 20.0 * 0.001)) < 1e-6f);
}

// The simulated inner loop's UDP port per station: below the MCU (7350), the PC inner loop (7360) and DDS
// (7400 and up, 7400 + 250 * domain), for 35 robot stations, Masters (100-999) and stations 1001-1099.
static void testSimPorts() {
    const int table[][2] = {{1, 7201}, {2, 7202}, {35, 7235}, {99, 7299}, {100, 6100}, {545, 6545}, {999, 6999},
                            {1001, 7101}, {1099, 7199}};
    for (const auto& t : table) CHECK(fic::link::simInnerLoopPort(t[0]) == t[1]);
    for (int n = 1; n <= 1099; ++n) {
        const int p = fic::link::simInnerLoopPort(n);
        CHECK(p > 1024 && p < 7350);
    }
}

int main() {
    testSimPorts();
    testRetimeConfig();
    testLinkRoundTrip();
    testLinkV3();
    testLinkSim();
    testLifecycle();
    testRampWithDamping();
    testJointLimitFault();
    testAdmittanceHold();
    testNodeDispatch();
    testTargetSpeed();
    testHapficSplitFallback();
    testDirectEstop();
    testSpeedCapInCmd();
    testTauLaw();
    if (g_fail) { std::printf("%d check(s) FAILED\n", g_fail); return 1; }
    std::printf("all inner-loop tests passed\n");
    return 0;
}
