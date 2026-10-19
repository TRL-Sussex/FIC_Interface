// End to end: the fic_inner_loop program (the PC inner loop) against a fake vendor driver that keeps the Kinova
// driver's shared-memory contract (waits in position mode for a finite command, then applies commanded_torque
// every ms, leaves torque mode on stop_torque_mode), driven over FIC-link as fic_control drives it.
//
//   test_inner_loop_process <addr:port> <segment name>     (ctest starts fic_inner_loop with the same name)

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <thread>

#include <fic/mcu_link.hpp>
#include <fic/shared_params.h>

using namespace fic::link;
using Clock = std::chrono::steady_clock;

static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { std::printf("  FAIL line %d: %s\n", __LINE__, #c); ++g_fail; } } while (0)
static const int N = 7;

// The fake driver: a decoupled 7-joint arm with gravity G_i sin q_i, on the shared segment.
struct FakeDriver {
    std::atomic<bool> run{true}, torque_mode{false}, saw_finite_early{false}, left_torque{false};
    std::atomic<int> torque_ticks{0};
    long noise_k = 0;
    double q[N], qd[N];
    const double G[N] = {10, 30, 5, 15, 2, 3, 0.5};
    void loop(SharedFeedback* s) {
        for (int i = 0; i < N; ++i) { q[i] = 0.2 + 0.1 * i; qd[i] = 0; }
        s->alive.store(1);
        auto next = Clock::now();
        while (run) {
            next += std::chrono::milliseconds(1);
            std::this_thread::sleep_until(next);
            double tau_cmd[N];
            bool finite = true;
            for (int i = 0; i < N; ++i) {
                tau_cmd[i] = s->commanded_torque[i].load();
                finite = finite && std::isfinite(tau_cmd[i]);
            }
            if (!torque_mode && finite && s->stop_torque_mode.load() == 0) torque_mode = true;   // Kinova's wait
            if (torque_mode && s->stop_torque_mode.load() == 1) { torque_mode = false; left_torque = true; }
            for (int i = 0; i < N; ++i) {
                const double g = G[i] * std::sin(q[i]);
                const double tau = torque_mode ? (std::isfinite(tau_cmd[i]) ? tau_cmd[i] : 0.0) : g;  // position mode holds
                const double qdd = (tau - g - 2.0 * qd[i]) / 0.5;
                qd[i] += qdd * 0.001;
                q[i] += qd[i] * 0.001;
                s->joint_positions[i].store((float)q[i]);
                // A real torque sensor is never bit-steady (the inner loop treats bit-steady feedback as frozen).
                s->torque[i].store((float)(g + 1e-3 * std::sin(0.37 * ++noise_k + i)));
            }
            if (torque_mode) ++torque_ticks;
        }
        s->alive.store(0);
    }
};

static StateMsg waitState(fic::McuLink& l, double s) {
    std::this_thread::sleep_for(std::chrono::duration<double>(s));
    StateMsg st;
    double age = 0;
    l.latestState(st, age);
    return st;
}

int main(int argc, char** argv) {
    if (argc < 3) { std::printf("usage: %s addr:port /segment\n", argv[0]); return 2; }
    setenv("FIC_SHM_FEEDBACK_NAME", argv[2], 1);
    SharedFeedback* seg = fic::attachFeedbackSegment(nullptr);
    CHECK(seg != nullptr);
    if (!seg) return 1;
    FakeDriver drv;
    std::thread driver([&] { drv.loop(seg); });

    fic::McuLink link;
    CHECK(link.open(argv[1]));
    auto ack = link.probe(500);
    CHECK(ack.has_value());
    if (!ack) { drv.run = false; driver.join(); return 1; }
    std::printf("probe: %s v%u adapters 0x%X\n", ack->fw_name, ack->fw_version, ack->adapters);
    CHECK(ack->adapters & adapterBit(Adapter::Shm));

    Config c{};
    c.config_epoch = 0x51;
    c.adapter = (uint32_t)Adapter::Shm;
    c.n_joints = N;
    c.hold_after_s = 0.05f; c.ramp_after_s = 1.0f;
    c.deadband_joint_tau = 1.5f; c.dt = 0.001f;
    c.torque_cutoff_hz = 200; c.state_cutoff_hz = 10;
    c.damping_alpha = (float)std::exp(-2.0 * M_PI * 20.0 / 1000.0);
    c.restart_on_stall = 1; c.planner_max_hz = 8.75f; c.planner_speed_margin = 1.5708f;
    c.task_db_force = 4.0f; c.task_db_torque = 0.2f;
    for (int i = 0; i < N; ++i) {
        c.lower[i] = -3; c.upper[i] = 3; c.fault_lower[i] = -3; c.fault_upper[i] = 3;
        c.torque_limit[i] = 39; c.torque_rate_limit[i] = 2; c.torque_sign[i] = 1;
    }
    CHECK(link.sendConfig(c));
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
    StateMsg st = waitState(link, 0.3);
    std::printf("configured: state %d\n", (int)st.state);
    CHECK(st.state == McuState::Ready);
    CHECK(!drv.torque_mode && !drv.saw_finite_early);           // nothing commanded yet: NaN

    // 1 s of Impedance CMDs holding where the arm is, with its gravity.
    auto cmd = [&](uint8_t flags) {
        CmdMsg m;
        m.config_epoch = c.config_epoch; m.param_epoch = 1; m.n = N;
        m.mode = InnerMode::Impedance; m.flags = flags | kCmdHasGravJac;
        for (int i = 0; i < N; ++i) {
            const double qi = drv.q[i];
            m.q_target[i] = (float)qi; m.q0[i] = (float)qi;
            m.g0[i] = (float)(drv.G[i] * std::sin(qi));
            m.dgdq[i * N + i] = (float)(drv.G[i] * std::cos(qi));
        }
        link.sendCmd(m);
    };
    auto next = Clock::now();
    for (int k = 0; k < 1000; ++k) { cmd(kCmdEnable); next += std::chrono::milliseconds(1); std::this_thread::sleep_until(next); }
    double age = 0;
    CHECK(link.latestState(st, age) && age < 0.05);
    std::printf("running: state %d, driver in torque %d for %d ticks, faults 0x%X\n", (int)st.state,
                (int)drv.torque_mode, drv.torque_ticks.load(), st.faults);
    CHECK(st.state == McuState::Running && drv.torque_mode && drv.torque_ticks > 500 && st.faults == 0);

    // fic_control goes quiet: HOLD, then RAMP, then the stop flag and the driver leaves torque mode.
    st = waitState(link, 0.2);
    CHECK(st.state == McuState::Hold);
    st = waitState(link, 2.5);
    std::printf("silent 2.7 s: state %d, stop flag %u, driver left torque %d\n", (int)st.state,
                seg->stop_torque_mode.load(), (int)drv.left_torque);
    CHECK(st.state == McuState::Ready && seg->stop_torque_mode.load() == 1 && drv.left_torque);

    // Torque again, then a direct E-STOP (the Studio's) while CMDs keep coming: RAMP, never back to Running.
    for (int k = 0; k < 500; ++k) { cmd(kCmdEnable); next = Clock::now() + std::chrono::milliseconds(1); std::this_thread::sleep_until(next); }
    CHECK(link.latestState(st, age) && st.state == McuState::Running && seg->stop_torque_mode.load() == 0);
    CHECK(link.sendEstop());
    bool back_to_running = false;
    for (int k = 0; k < 300; ++k) {
        cmd(kCmdEnable);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (link.latestState(st, age) && k > 20 && st.state == McuState::Running) back_to_running = true;
    }
    std::printf("after estop: state %d faults 0x%X\n", (int)st.state, st.faults);
    CHECK(!back_to_running && (st.faults & kFaultEstop));

    // A new CONFIG clears the direct E-STOP; torque again; then the driver dies (alive -> 0) while torqued:
    // the inner loop must latch a FAULT (review 19 Oct, I4), not reconnect and torque again by itself when a
    // driver comes back.
    c.config_epoch = 0x52;
    CHECK(link.sendConfig(c));
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
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    for (int k = 0; k < 500; ++k) { cmd(kCmdEnable); std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
    CHECK(link.latestState(st, age) && st.state == McuState::Running);
    drv.run = false;
    driver.join();                                             // alive -> 0
    for (int k = 0; k < 200; ++k) { cmd(kCmdEnable); std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
    CHECK(link.latestState(st, age));
    std::printf("driver died: state %d faults 0x%X, command %f\n", (int)st.state, st.faults,
                seg->commanded_torque[0].load());
    CHECK(st.state == McuState::Fault && (st.faults & kFaultArmLink));
    CHECK(std::isnan(seg->commanded_torque[0].load()));     // a returning driver must not find a torque
    FakeDriver drv2;                                           // a driver comes back
    std::thread driver2([&] { drv2.loop(seg); });
    for (int k = 0; k < 300; ++k) { cmd(kCmdEnable); std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
    CHECK(link.latestState(st, age) && st.state == McuState::Fault && !drv2.torque_mode);

    // A normal Stop: the ramp finishes, the stop flag goes up, and the Kinova driver leaves torque mode and
    // exits. That is no fault (the status must not say FAULT after every Stop), but the loop still waits for a
    // new CONFIG (Start) before it torques again, so a driver that comes back never gets torque by itself.
    c.config_epoch = 0x53;
    CHECK(link.sendConfig(c));
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
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    for (int k = 0; k < 500; ++k) { cmd(kCmdEnable); std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
    CHECK(link.latestState(st, age) && st.state == McuState::Running);
    st = waitState(link, 2.7);                                 // quiet: HOLD, RAMP, released, stop flag
    CHECK(st.state == McuState::Ready && seg->stop_torque_mode.load() == 1 && drv2.left_torque);
    drv2.run = false;
    driver2.join();                                            // the driver exits after leaving torque mode
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    CHECK(link.latestState(st, age));
    std::printf("driver exited after a Stop: state %d faults 0x%X\n", (int)st.state, st.faults);
    CHECK(st.state != McuState::Fault && st.faults == 0);
    FakeDriver drv3;                                           // it comes back; the node is still commanding
    std::thread driver3([&] { drv3.loop(seg); });
    for (int k = 0; k < 300; ++k) { cmd(kCmdEnable); std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
    CHECK(link.latestState(st, age));
    std::printf("driver back, no new CONFIG: state %d, torqued %d\n", (int)st.state, (int)drv3.torque_mode);
    CHECK(st.state != McuState::Running && !drv3.torque_mode);
    drv3.run = false;
    driver3.join();
    shm_unlink(argv[2]);
    std::printf(g_fail ? "%d check(s) FAILED\n" : "inner-loop process OK\n", g_fail);
    return g_fail ? 1 : 0;
}
