// fic_inner_loop — the FIC joint inner loop as a process on the PC that drives a robot.
//
// The same code the inner-loop MCU runs (fic/mcu_node.h -> fic/inner_loop.h -> fic/fic_joint_core.h), on a
// 1 kHz timer, answering FIC-link on UDP, with the vendor driver's shared memory (fic/shm_adapter.h) as its
// adapter. fic_control drives it exactly as it drives the MCU: it finds it by HELLO (Adapter::Shm), sends
// CONFIG / PARAMS, one CMD per tick, and reads STATE. If fic_control stops or stalls the inner loop holds
// the arm (50 ms) and then ramps it down and hands it back (2 s), as on the MCU.
//
//   fic_inner_loop [--port 7360] [--rt] [--quiet]
//   fic_inner_loop --adapter sim [--station N | --port P] [--quiet]   (port: fic::link::simInnerLoopPort(N))
//
// --adapter sim: the plant is a simulator in lockstep (mujoco_native, or the task node's topic bridge). There is
// no timer: each SimStep from the plant is one tick, on the step's simulated time (so the hold / ramp watchdogs
// count simulated time), answered by one SimTorque; STATE goes to the task node. No shared memory is touched,
// so simulated stations run beside the real robot's fic_inner_loop on the same PC.
//
// --rt: SCHED_FIFO priority 80 and locked memory, when the user may (a warning otherwise).
// The segment is /robot_feedback_memory, or FIC_SHM_FEEDBACK_NAME (tests). On SIGINT / SIGTERM a torquing arm
// is ramped down first (up to 3 s), then the process exits.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <fic/fic_link.h>
#include <fic/mcu_node.h>
#include <fic/shm_adapter.h>

using namespace fic::link;

static std::atomic<bool> g_run{true};
static void onSignal(int) { g_run = false; }

static uint64_t nowUs() {
    timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000000ull + t.tv_nsec / 1000;
}

static void goRealtime() {
    sched_param sp{};
    sp.sched_priority = 80;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0)
        std::fprintf(stderr, "fic_inner_loop: --rt: SCHED_FIFO not permitted (%s); running at normal priority\n",
                     std::strerror(errno));
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        std::fprintf(stderr, "fic_inner_loop: --rt: mlockall failed (%s)\n", std::strerror(errno));
}

// --adapter sim: one tick per SimStep, on simulated time.
static int runSim(int port, bool quiet) {
    static fic::mcu::McuNode node("fic_inner_loop", 3, adapterBit(Adapter::HostSim));
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in me{};
    me.sin_family = AF_INET;
    me.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    me.sin_port = htons(port);
    if (fd < 0 || ::bind(fd, (sockaddr*)&me, sizeof(me)) != 0) {
        std::perror("fic_inner_loop: bind");
        return 1;
    }
    timeval tv{0, 200000};                     // wake up to notice a stop request
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    std::printf("fic_inner_loop: simulator in lockstep, FIC-link on 127.0.0.1:%d\n", port);
    std::fflush(stdout);

    sockaddr_in task{};
    bool have_task = false;
    uint8_t rx[2048], tx[kMaxDatagram];
    uint64_t sim_us = 0, last_print = 0, prev_step_us = 0, time_offset = 0;
    uint32_t steps = 0;
    // The simulator's own step (the time between two SimSteps): the loop runs at it, whatever CONFIG says
    // (the scene's timestep, 1 ms, or an experiment's sim_dt 2 ms). A CONFIG that arrives before the step is
    // known waits for it.
    float plant_dt = 0.0f;
    bool config_pending = false;
    Config pending{};
    sockaddr_in pending_from{};
    auto applyConfig = [&](Config c, const sockaddr_in& from) {
        fic::mcu::retimeConfig(c, plant_dt);
        const size_t cl = encodeFixed(MsgType::Config, c, 0, tx, sizeof(tx));
        if (cl) node.onDatagram(tx, cl, sim_us, tx, sizeof(tx));
        task = from;
        have_task = true;
    };
    while (g_run) {
        sockaddr_in from{};
        socklen_t fl = sizeof(from);
        const ssize_t n = ::recvfrom(fd, rx, sizeof(rx), 0, (sockaddr*)&from, &fl);
        if (n <= 0) continue;
        Reader r;
        if (r.open(rx, (size_t)n) && r.type() == MsgType::SimStep) {
            SimStepMsg st;
            if (!decodeSimStep(r, st)) continue;
            // The loop's clock is the simulator's time made monotonic: a simulator reset (its time back to 0)
            // continues one step on, so the commands' age never runs backwards and the arm stays driven.
            uint64_t t = st.t_us + time_offset;
            if (prev_step_us && t <= prev_step_us) {
                const uint64_t dt_us = plant_dt > 0.0f ? uint64_t(plant_dt * 1e6f + 0.5f) : 1000;
                time_offset += prev_step_us + dt_us - t;
                t = st.t_us + time_offset;
            } else if (prev_step_us && t - prev_step_us < 100000) {
                plant_dt = float(t - prev_step_us) * 1e-6f;
            }
            prev_step_us = t;
            sim_us = t;
            if (config_pending && plant_dt > 0.0f) {
                config_pending = false;
                applyConfig(pending, pending_from);
            }
            fic::mcu::ArmFeedback fb;
            fb.valid = node.hasConfig() && st.n == node.config().n_joints;
            for (int i = 0; i < st.n; ++i) { fb.q[i] = st.q[i]; fb.tau[i] = st.tau[i]; }
            for (int k = 0; k < 6; ++k) fb.wrench[k] = st.wrench[k];
            if (node.hasConfig()) node.loop().adapterReady(true);      // a stepping simulator is connected
            fic::mcu::ArmCommand cmd;
            node.tick(fb, sim_us, cmd);
            SimTorqueMsg out;
            out.step = ++steps;
            out.t_us = st.t_us;                                  // the simulator matches it to its step
            out.torque_mode = cmd.torque_mode ? 1 : 0;
            out.n = st.n;
            for (int i = 0; i < st.n; ++i) out.tau[i] = cmd.tau[i];
            const size_t ol = encodeSimTorque(out, steps, tx, sizeof(tx));
            if (ol) ::sendto(fd, tx, ol, 0, (sockaddr*)&from, fl);
            if (have_task && node.hasConfig()) {
                const size_t sl = node.buildState(tx, sizeof(tx), 0);
                if (sl) ::sendto(fd, tx, sl, 0, (sockaddr*)&task, sizeof(task));
            }
            if (!quiet && sim_us - last_print > 5000000) {
                last_print = sim_us;
                static const char* names[] = {"idle", "connecting", "ready", "RUNNING", "HOLD", "RAMP", "FAULT"};
                std::printf("fic_inner_loop (sim): %s at %.1f s simulated, faults 0x%02X\n",
                            names[(int)node.loop().state()], sim_us * 1e-6, node.loop().faults());
                std::fflush(stdout);
            }
            continue;
        }
        // The task node (HELLO, CONFIG, PARAMS, CMD, Estop): on simulated time too.
        if (r.open(rx, (size_t)n) && r.type() == MsgType::Config) {
            Config c;
            if (decodeFixed(r, c)) {
                if (plant_dt > 0.0f) applyConfig(c, from);
                else { pending = c; pending_from = from; config_pending = true; task = from; have_task = true; }
            }
            continue;
        }
        task = from;
        have_task = true;
        const size_t rl = node.onDatagram(rx, (size_t)n, sim_us, tx, sizeof(tx));
        if (rl) ::sendto(fd, tx, rl, 0, (sockaddr*)&task, sizeof(task));
    }
    std::printf("fic_inner_loop (sim): stopped\n");
    return 0;
}

int main(int argc, char** argv) {
    int port = 7360;
    bool rt = false, quiet = false;
    std::string adapter = "shm";
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--port" && i + 1 < argc) port = std::atoi(argv[++i]);
        else if (a == "--station" && i + 1 < argc) port = fic::link::simInnerLoopPort(std::atoi(argv[++i]));
        else if (a == "--adapter" && i + 1 < argc) adapter = argv[++i];
        else if (a == "--rt") rt = true;
        else if (a == "--quiet") quiet = true;
        else {
            std::printf("usage: %s [--adapter shm|sim] [--port 7360 | --station N] [--rt] [--quiet]\n", argv[0]);
            return a == "--help" ? 0 : 2;
        }
    }
    if (adapter == "sim") return runSim(port, quiet);
    if (adapter != "shm") {
        std::fprintf(stderr, "fic_inner_loop: unknown adapter '%s' (shm | sim)\n", adapter.c_str());
        return 2;
    }

    static fic::mcu::McuNode node("fic_inner_loop", 3, adapterBit(Adapter::Shm));
    static fic::mcu::ShmAdapter shm;
    if (!shm.attach()) {
        std::fprintf(stderr, "fic_inner_loop: cannot map the shared memory %s\n", feedbackSegmentName());
        return 1;
    }

    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in me{};
    me.sin_family = AF_INET;
    me.sin_addr.s_addr = htonl(INADDR_LOOPBACK);      // fic_control runs on this PC
    me.sin_port = htons(port);
    if (fd < 0 || ::bind(fd, (sockaddr*)&me, sizeof(me)) != 0) {
        std::perror("fic_inner_loop: bind");
        return 1;
    }
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    if (rt) goRealtime();
    std::printf("fic_inner_loop: FIC-link on 127.0.0.1:%d, shared memory %s\n", port, feedbackSegmentName());
    std::fflush(stdout);

    sockaddr_in peer{};
    bool have_peer = false;
    uint8_t rx[2048], tx[kMaxDatagram];
    uint64_t last_print = nowUs(), last_remap = nowUs(), stop_by = 0;
    uint32_t driver_seen_config = 0;          // the CONFIG under which the driver was last seen up
    uint16_t jitter_max = 0;

    timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    for (;;) {
        next.tv_nsec += 1000000;
        if (next.tv_nsec >= 1000000000) { next.tv_nsec -= 1000000000; ++next.tv_sec; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
        const uint64_t now = nowUs();
        const uint64_t due = (uint64_t)next.tv_sec * 1000000ull + next.tv_nsec / 1000;
        const uint64_t late = now > due ? now - due : 0;
        if (late > jitter_max) jitter_max = late > 65535 ? 65535 : (uint16_t)late;

        // Asked to stop: ramp a torquing arm down first (E-STOP path), at most 3 s.
        if (!g_run && !stop_by) {
            node.loop().estop();
            stop_by = now + 3000000;
        }
        const McuState s = node.loop().state();
        const bool torquing = s == McuState::Running || s == McuState::Hold || s == McuState::Ramp;
        if (stop_by && (!torquing || now > stop_by)) break;

        for (;;) {
            sockaddr_in from{};
            socklen_t fl = sizeof(from);
            const ssize_t n = ::recvfrom(fd, rx, sizeof(rx), MSG_DONTWAIT, (sockaddr*)&from, &fl);
            if (n <= 0) break;
            peer = from;
            have_peer = true;
            const size_t rl = node.onDatagram(rx, (size_t)n, now, tx, sizeof(tx));
            if (rl) ::sendto(fd, tx, rl, 0, (sockaddr*)&peer, sizeof(peer));
        }

        if (now - last_remap > 500000) { shm.checkRemap(); last_remap = now; }
        const int n = node.hasConfig() ? node.config().n_joints : 0;
        fic::mcu::ArmFeedback fb = shm.read(n);
        if (node.hasConfig()) {
            // Until the driver is up under this CONFIG the loop waits (Connecting). Once it has been up, losing
            // it is not "connecting again": the feedback goes invalid and the loop's arm-link fault latches
            // until a new CONFIG (Start), so a driver that comes back never gets torque by itself.
            if (shm.ready()) {
                node.loop().adapterReady(true);
                driver_seen_config = node.configChanges();
            } else if (driver_seen_config != node.configChanges()) {
                node.loop().adapterReady(false);
            }
        }
        fic::mcu::ArmCommand cmd;
        node.tick(fb, now, cmd);
        if (n > 0) {
            // Not torquing: the arm's gravity at its pose (the old idle baseline, for a driver that stays in
            // torque mode, libfranka) while the stop flag tells a Kinova driver to leave torque mode.
            float idle[fic::link::kLinkMaxJoints];
            for (int i = 0; i < n; ++i) idle[i] = static_cast<float>(node.loop().gravity(i));
            shm.write(cmd, n, node.loop().released(), node.loop().state() == McuState::Fault ? nullptr : idle);
        }

        if (have_peer && node.hasConfig()) {
            const size_t sl = node.buildState(tx, sizeof(tx), jitter_max);
            if (sl) ::sendto(fd, tx, sl, 0, (sockaddr*)&peer, sizeof(peer));
        }

        if (!quiet && now - last_print > 5000000) {
            last_print = now;
            static const char* names[] = {"idle", "connecting", "ready", "RUNNING", "HOLD", "RAMP", "FAULT"};
            std::printf("fic_inner_loop: %s, driver %s, faults 0x%02X, jitter %u us\n",
                        names[(int)node.loop().state()], shm.ready() ? "up" : "not running",
                        node.loop().faults(), jitter_max);
            std::fflush(stdout);
            jitter_max = 0;
        }
    }
    // Leaving: whatever the ramp reached is held, and the driver is told to leave torque mode.
    fic::mcu::ArmCommand none;
    const int n = node.hasConfig() ? node.config().n_joints : 0;
    if (n > 0) shm.write(none, n, true);
    std::printf("fic_inner_loop: stopped\n");
    return 0;
}
