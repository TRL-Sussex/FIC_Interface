// fic_inner_loop --adapter sim: the inner loop ticks once per simulator step (SimStep -> SimTorque), on
// simulated time; the task node talks FIC-link to it as on hardware. This test is both: a fake plant
// (decoupled joints) stepping 2 ms of simulated time as fast as it can, and a McuLink client.
//
//   test_inner_loop_sim <port>      (ctest starts fic_inner_loop --adapter sim on that port)

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#include <fic/mcu_link.hpp>

using namespace fic::link;

static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { std::printf("  FAIL line %d: %s\n", __LINE__, #c); ++g_fail; } } while (0)
static const int N = 7;

struct Plant {
    double q[N], qd[N];
    const double G[N] = {10, 30, 5, 15, 2, 3, 0.5};
    double q_hold[N];
    bool held = false;
    Plant() { for (int i = 0; i < N; ++i) { q[i] = 0.2 + 0.1 * i; qd[i] = 0; } }
    double g(int i) const { return G[i] * std::sin(q[i]); }
    void step(const SimTorqueMsg& t, double dt) {
        if (!t.torque_mode && !held) { for (int i = 0; i < N; ++i) q_hold[i] = q[i]; held = true; }
        if (t.torque_mode) held = false;
        for (int i = 0; i < N; ++i) {
            const double tau = t.torque_mode ? t.tau[i] : g(i) + 200 * (q_hold[i] - q[i]) - 20 * qd[i];
            qd[i] += (tau - g(i) - 2.0 * qd[i]) / 0.5 * dt;
            q[i] += qd[i] * dt;
        }
    }
};

int main(int argc, char** argv) {
    if (argc < 2) return 2;
    const int port = std::atoi(argv[1]);
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in to{};
    to.sin_family = AF_INET;
    to.sin_port = htons(port);
    to.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    timeval tv{0, 200000};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    fic::McuLink link;
    CHECK(link.open("127.0.0.1:" + std::to_string(port)));
    auto ack = link.probe(500);
    CHECK(ack.has_value() && (ack->adapters & adapterBit(Adapter::HostSim)));
    if (!ack) return 1;

    // The simulator runs before the controller starts, at its own step (1 ms here), whatever CONFIG says.
    {
        uint8_t b0[kMaxDatagram];
        for (int k = 0; k < 5; ++k) {
            SimStepMsg s0;
            s0.t_us = 1000ull * k; s0.n = N;
            const size_t n0 = encodeSimStep(s0, 1, b0, sizeof(b0));
            ::sendto(fd, b0, n0, 0, (sockaddr*)&to, sizeof(to));
            ::recv(fd, b0, sizeof(b0), 0);
        }
    }
    Config c{};
    c.config_epoch = 0x61;
    c.adapter = (uint32_t)Adapter::HostSim;
    c.n_joints = N;
    c.hold_after_s = 0.05f; c.ramp_after_s = 2.0f;
    c.deadband_joint_tau = 1.5f; c.dt = 0.002f;
    c.torque_cutoff_hz = 10; c.state_cutoff_hz = 10;
    c.damping_alpha = (float)std::exp(-2.0 * M_PI * 20.0 * 0.002);
    c.restart_on_stall = 1; c.planner_max_hz = 8.75f; c.planner_speed_margin = 1.5708f;
    c.task_db_force = 4.0f; c.task_db_torque = 0.2f;
    for (int i = 0; i < N; ++i) {
        c.lower[i] = -3; c.upper[i] = 3; c.fault_lower[i] = -3; c.fault_upper[i] = 3;
        c.torque_limit[i] = 39; c.torque_sign[i] = -1;
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
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    Plant pl;
    double target[N];
    for (int i = 0; i < N; ++i) target[i] = pl.q[i] + 0.2;
    uint64_t t_us = 5000;
    const uint64_t step_us = 1000;                     // the simulator's step
    uint8_t buf[kMaxDatagram];
    int replies = 0, torqued = 0, mismatched = 0;
    // One simulated step: SimStep -> SimTorque; optionally a CMD from the task node beforehand.
    auto step = [&](bool with_cmd) {
        if (with_cmd) {
            CmdMsg m;
            m.config_epoch = c.config_epoch; m.param_epoch = 1; m.n = N;
            m.mode = InnerMode::Impedance; m.flags = kCmdEnable | kCmdHasGravJac;
            for (int i = 0; i < N; ++i) {
                m.q_target[i] = (float)target[i]; m.q0[i] = (float)pl.q[i];
                m.g0[i] = (float)pl.g(i); m.dgdq[i * N + i] = (float)(pl.G[i] * std::cos(pl.q[i]));
            }
            link.sendCmd(m);
        }
        SimStepMsg s;
        s.t_us = t_us; s.n = N;
        for (int i = 0; i < N; ++i) { s.q[i] = (float)pl.q[i]; s.tau[i] = (float)pl.g(i); }
        const size_t n = encodeSimStep(s, 1, buf, sizeof(buf));
        ::sendto(fd, buf, n, 0, (sockaddr*)&to, sizeof(to));
        SimTorqueMsg tq;
        for (;;) {                                      // the reply (skip anything else)
            const ssize_t r = ::recv(fd, buf, sizeof(buf), 0);
            if (r <= 0) { std::printf("  no SimTorque\n"); ++g_fail; return; }
            Reader rd;
            if (rd.open(buf, (size_t)r) && rd.type() == MsgType::SimTorque && decodeSimTorque(rd, tq)) break;
        }
        if (tq.t_us != t_us) ++mismatched;              // each reply answers the step just sent
        ++replies;
        torqued += tq.torque_mode;
        pl.step(tq, step_us * 1e-6);
        t_us += step_us;
    };

    for (int k = 0; k < 1500; ++k) step(true);         // 3 s of simulated time
    StateMsg st;
    double age = 0;
    CHECK(link.latestState(st, age));
    double err = 0;
    for (int i = 0; i < N; ++i) err = std::fmax(err, std::fabs(target[i] - pl.q[i]));
    std::printf("running: state %d, %d replies, %d torqued, tracking error %.4f rad\n", (int)st.state, replies,
                torqued, err);
    CHECK(st.state == McuState::Running && torqued > 1400 && err < 0.15);
    std::printf("replies naming another step: %d\n", mismatched);
    CHECK(mismatched == 0);
    std::printf("the step the inner loop runs at: %.4f s (CONFIG said 0.002)\n", st.dt);
    CHECK(std::fabs(st.dt - 0.001f) < 1e-6);          // the simulator's step, not CONFIG's

    // The simulator resets (its time back to 0: SimulationReset, the viewer's Backspace, TS-1's jumps) while
    // the task node keeps commanding: the inner loop keeps running, it does not take the jump back for silence.
    t_us = 0;
    int left_running = 0;
    for (int k = 0; k < 300; ++k) {
        step(true);
        std::this_thread::sleep_for(std::chrono::microseconds(200));   // its STATE for this step
        if (link.latestState(st, age) && st.state != McuState::Running) ++left_running;
    }
    std::printf("after a simulator reset: %d of 300 steps not running\n", left_running);
    CHECK(left_running == 0);

    for (int k = 0; k < 100; ++k) step(false);         // 100 ms of simulated time without a CMD
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    CHECK(link.latestState(st, age));
    std::printf("100 ms (sim) silent: state %d\n", (int)st.state);
    CHECK(st.state == McuState::Hold);
    int last_mode = 1;
    for (int k = 0; k < 3000; ++k) step(false);        // 3 s more: RAMP, then out of torque
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    CHECK(link.latestState(st, age));
    last_mode = pl.held ? 0 : 1;
    std::printf("3.1 s (sim) silent: state %d, plant holding itself %d\n", (int)st.state, (int)pl.held);
    CHECK(st.state == McuState::Ready && last_mode == 0);

    // The topic plant (the Newton backend): the task node's own McuLink is the plant too,
    // sending SimStep from the latest joint_states and reading SimTorque.
    c.config_epoch = 0x62;
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
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    int via_link = 0;
    for (int k = 0; k < 400; ++k) {
        CmdMsg m;
        m.config_epoch = c.config_epoch; m.param_epoch = 1; m.n = N;
        m.mode = InnerMode::Impedance; m.flags = kCmdEnable | kCmdHasGravJac;
        for (int i = 0; i < N; ++i) {
            m.q_target[i] = (float)pl.q[i]; m.q0[i] = (float)pl.q[i]; m.g0[i] = (float)pl.g(i);
            m.dgdq[i * N + i] = (float)(pl.G[i] * std::cos(pl.q[i]));
        }
        link.sendCmd(m);
        SimStepMsg s;
        s.t_us = t_us; s.n = N;
        for (int i = 0; i < N; ++i) { s.q[i] = (float)pl.q[i]; s.tau[i] = (float)pl.g(i); }
        CHECK(link.sendSimStep(s));
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        SimTorqueMsg tq;
        double tq_age = 0;
        if (link.latestSimTorque(tq, tq_age) && tq_age < 0.05) { ++via_link; pl.step(tq, 0.001); }
        t_us += 1000;
    }
    CHECK(link.latestState(st, age));
    std::printf("topic plant via McuLink: %d SimTorque, state %d\n", via_link, (int)st.state);
    CHECK(via_link > 300 && st.state == McuState::Running);

    ::close(fd);
    std::printf(g_fail ? "%d check(s) FAILED\n" : "inner-loop sim OK\n", g_fail);
    return g_fail ? 1 : 0;
}
