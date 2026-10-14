// End-to-end over real UDP: fic_hardware's McuLink (Linux end) against a
// running fic_mcu_sim (MCU end). Start the simulator first:
//
//   robot_library/fic_mcu/build/fic_mcu_sim --port 7351 &
//   ./test_mcu_link_loop 127.0.0.1:7351
//
// Probe -> CONFIG -> PARAMS -> 3 s of 1 kHz CMDs (Running, tracking) -> Linux
// goes silent (HOLD, then RAMP, then out of torque mode) -> Linux returns.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <thread>

#include <fic/mcu_link.hpp>

using namespace fic::link;
using Clock = std::chrono::steady_clock;

static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { std::printf("  FAIL line %d: %s\n", __LINE__, #c); ++g_fail; } } while (0)

static const int N = 7;

static StateMsg waitState(fic::McuLink& l, double s) {
    std::this_thread::sleep_for(std::chrono::duration<double>(s));
    StateMsg st; double age = 0;
    l.latestState(st, age);
    return st;
}

int main(int argc, char** argv) {
    const char* addr = argc > 1 ? argv[1] : "127.0.0.1:7350";
    fic::McuLink link;
    CHECK(link.open(addr));
    auto ack = link.probe();
    CHECK(ack.has_value());
    if (!ack) { std::printf("no MCU at %s\n", addr); return 1; }
    std::printf("probe: %s v%u adapters 0x%X\n", ack->fw_name, ack->fw_version, ack->adapters);
    CHECK(ack->adapters & adapterBit(Adapter::Loopback));

    Config c{};
    c.config_epoch = 0x5EED;
    c.adapter = (uint32_t)Adapter::Loopback;
    c.n_joints = N;
    c.hold_after_s = 0.05f; c.ramp_after_s = 1.0f;
    c.deadband_joint_tau = 1.5f; c.dt = 0.001f;
    c.torque_cutoff_hz = 200; c.state_cutoff_hz = 10;
    c.damping_alpha = (float)std::exp(-2.0 * M_PI * 20.0 / 1000.0);
    for (int i = 0; i < N; ++i) {
        c.lower[i] = -3; c.upper[i] = 3; c.fault_lower[i] = -3; c.fault_upper[i] = 3;
        c.torque_limit[i] = 39; c.torque_rate_limit[i] = 2; c.torque_sign[i] = 1;
    }
    CHECK(link.sendConfig(c));
    StateMsg st = waitState(link, 0.3);
    CHECK(st.config_epoch == c.config_epoch);
    CHECK(st.state == McuState::Ready);

    for (int j = 0; j < N; ++j) {
        Params p{};
        p.param_epoch = 1; p.joint = j; p.n_joints = N;
        p.p.stiffness = 100; p.p.n_saturations = 2; p.p.n_neg_sat = 1;
        p.p.n_error_max = 1; p.p.error_max[0] = 0.4;
        p.p.n_force_max = 1; p.p.force_max[0] = 30;
        p.p.planner_stiffness = 15000; p.p.vel_desired = 0.5; p.p.acc_max = 5000;
        p.p.damping_ratio = 0.7; p.p.frequency = 20;
        p.p.inv_m = 1; p.p.adm_damping = 10;
        p.p.dflow[0] = -5; p.p.dflow[1] = 5; p.p.flow[0] = -1; p.p.flow[1] = 1;
        CHECK(link.sendParams(p));
    }

    // 3 s of CMDs at 1 kHz: impedance+planner to q + 0.2, zero gravity.
    double qt[N];
    st = waitState(link, 0.05);
    for (int i = 0; i < N; ++i) qt[i] = st.q_raw[i] + 0.2;
    auto next = Clock::now();
    for (int k = 0; k < 3000; ++k) {
        CmdMsg m;
        m.config_epoch = c.config_epoch; m.param_epoch = 1; m.n = N;
        m.mode = InnerMode::ImpedancePlanner; m.flags = kCmdEnable;
        for (int i = 0; i < N; ++i) m.q_target[i] = (float)qt[i];
        link.sendCmd(m);
        next += std::chrono::milliseconds(1);
        std::this_thread::sleep_until(next);
    }
    double age = 0;
    CHECK(link.latestState(st, age) && age < 0.05);
    CHECK(st.state == McuState::Running);
    CHECK(st.param_epoch == 1);
    double err = 0;
    for (int i = 0; i < N; ++i) err = std::fmax(err, std::fabs(qt[i] - st.q_raw[i]));
    std::printf("running: tracking error %.4f rad, STATE %.0f Hz, MCU jitter %u us, cmd age %u ms\n",
                err, link.stateRateHz(), st.jitter_us_max, st.cmd_age_ms);
    CHECK(err < 0.15);
    CHECK(link.stateRateHz() > 800);

    // Linux goes silent.
    st = waitState(link, 0.2);
    std::printf("silent 0.2 s: %d\n", (int)st.state);
    CHECK(st.state == McuState::Hold);
    st = waitState(link, 3.0);
    std::printf("silent 3.2 s: %d\n", (int)st.state);
    CHECK(st.state == McuState::Ready);

    std::printf(g_fail ? "%d check(s) FAILED\n" : "end-to-end OK\n", g_fail);
    return g_fail ? 1 : 0;
}
