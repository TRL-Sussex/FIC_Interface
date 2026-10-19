// Host tests for the portable MCU-side code: FIC-link framing, and the
// InnerLoop state machine (Running / Hold / Ramp / e-stop / faults / guards).
// No Eigen, no ROS:   see fic_library/test/CMakeLists.txt

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

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

    // Linux back: fresh CMDs re-enter Running.
    for (int k = 0; k < 20; ++k) { L.onCmd(makeCmd(pl, InnerMode::Impedance, kCmdEnable, 1, qt), 5000 + k, t); tickN(1); }
    CHECK(L.state() == McuState::Running);

    // E-stop: ramp out and stay out until re-armed (Enable dropped).
    for (int k = 0; k < 3000; ++k) {
        L.onCmd(makeCmd(pl, InnerMode::Impedance, kCmdEnable | kCmdEstop, 1, qt), 6000 + k, t);
        tickN(1);
    }
    CHECK(L.state() == McuState::Ready && !out.torque_mode);
    CHECK(L.faults() & kFaultEstop);
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

int main() {
    testLinkRoundTrip();
    testLifecycle();
    testJointLimitFault();
    testAdmittanceHold();
    testNodeDispatch();
    if (g_fail) { std::printf("%d check(s) FAILED\n", g_fail); return 1; }
    std::printf("all inner-loop tests passed\n");
    return 0;
}
