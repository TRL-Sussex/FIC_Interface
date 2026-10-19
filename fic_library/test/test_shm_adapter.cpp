// The PC inner loop's adapter on a vendor driver's shared memory (fic/shm_adapter.h): the driver contract
// (NaN until torque is wanted, the last torque held after a release, the stop flag once released), feedback
// validity (driver alive and a first real sample), a driver that leaves, and a segment that is recreated.
// Never touches /robot_feedback_memory: a private block, and a private segment name.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include <unistd.h>

#include <fic/shm_adapter.h>

using fic::mcu::ArmCommand;
using fic::mcu::ArmFeedback;
using fic::mcu::ShmAdapter;

static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); ++g_fail; } } while (0)
static const int N = 7;

static void driverWrites(SharedFeedback& s, double q0) {
    for (int i = 0; i < N; ++i) {
        s.joint_positions[i].store(float(q0 + 0.1 * i));
        s.torque[i].store(float(-1.0 * i));
    }
}

static void testContract() {
    std::printf("driver contract on a private block\n");
    static SharedFeedback seg;
    for (uint32_t i = 0; i < kMaxJoints; ++i) seg.commanded_torque[i].store(std::nanf(""));
    ShmAdapter a(&seg);
    CHECK(a.attach());
    ArmCommand idle;                                   // torque_mode false
    // (1) no driver yet
    CHECK(!a.ready());
    CHECK(!a.read(N).valid);
    a.write(idle, N, false);
    CHECK(std::isnan(seg.commanded_torque[0].load()));
    // (2) driver alive, still the zero fill: not a sample
    seg.alive.store(1);
    CHECK(!a.read(N).valid);
    driverWrites(seg, 0.3);                            // the first real sample
    ArmFeedback fb = a.read(N);
    CHECK(fb.valid && a.ready());
    CHECK(std::fabs(fb.q[2] - 0.5f) < 1e-6f && fb.tau[3] == -3.0f);
    // the gripper slot (robot with fewer joints than the slot)
    seg.joint_positions[kGripperFeedbackSlot].store(0.04f);
    seg.torque[kGripperFeedbackSlot].store(120.0f);
    fb = a.read(N);
    CHECK(fb.gripper_pos == 0.04f && fb.gripper_current == 120.0f);
    // still not torquing: NaN (a Kinova-style driver must not see a finite torque yet)
    a.write(idle, N, false);
    CHECK(std::isnan(seg.commanded_torque[0].load()));
    // (3) torque
    ArmCommand c;
    c.torque_mode = true;
    for (int i = 0; i < N; ++i) c.tau[i] = float(2 + i);
    c.gripper_width = 0.06f;
    a.write(c, N, false);
    CHECK(seg.commanded_torque[4].load() == 6.0f && seg.commanded_gripper_width.load() == 0.06f);
    CHECK(seg.stop_torque_mode.load() == 0);
    // (4) out of torque without the ramp's release (a fault, a new CONFIG): the last torque held AND the stop
    // flag at once — a driver that honours it (Kinova) must not keep applying a stale torque (seen on 19 Oct
    // when a reset loop left the driver torquing).
    a.write(idle, N, false);
    CHECK(seg.commanded_torque[4].load() == 6.0f && seg.stop_torque_mode.load() == 1);
    // (5) released after a ramp: the same
    a.write(c, N, false);
    CHECK(seg.stop_torque_mode.load() == 0);
    a.write(idle, N, true);
    CHECK(seg.stop_torque_mode.load() == 1 && seg.commanded_torque[4].load() == 6.0f);
    // torque wanted again: the flag cleared
    a.write(c, N, false);
    CHECK(seg.stop_torque_mode.load() == 0);
    // (6) the driver leaves: not ready, and NaN again (a restarting driver must not pick up the old torque)
    seg.alive.store(0);
    CHECK(!a.read(N).valid && !a.ready());
    a.write(idle, N, false);
    CHECK(std::isnan(seg.commanded_torque[0].load()));
    // it comes back: needs a fresh first sample
    seg.alive.store(1);
    for (int i = 0; i < N; ++i) seg.joint_positions[i].store(0.0f);
    CHECK(!a.read(N).valid);
    driverWrites(seg, 0.2);
    CHECK(a.read(N).valid);
}

static void testRemap() {
    std::printf("a recreated segment is followed\n");
    const std::string name = "/fic_test_shm_" + std::to_string(getpid());
    setenv("FIC_SHM_FEEDBACK_NAME", name.c_str(), 1);
    ShmAdapter a;
    CHECK(a.attach());
    // The driver recreates the segment (libfranka's reflex recovery does).
    shm_unlink(name.c_str());
    ino_t ino = 0;
    SharedFeedback* fresh = fic::attachFeedbackSegment(&ino);
    CHECK(fresh != nullptr);
    fresh->alive.store(1);
    for (int i = 0; i < N; ++i) fresh->joint_positions[i].store(0.7f);
    a.checkRemap();
    ArmFeedback fb = a.read(N);
    CHECK(fb.valid && fb.q[0] == 0.7f);
    shm_unlink(name.c_str());
}

// Review 19 Oct (phase 2), C2: a driver re-initialising the SAME segment (libfranka's reflex recovery
// calls cleanupSharedMemory(); initSharedMemory(); back to back; a restarted driver) must not be read as
// feedback, and the inner loop must never write torque over the driver's fresh NaN.
static void testDriverReinitBlip() {
    std::printf("driver re-initialises the same segment\n");
    static SharedFeedback seg;
    seg.alive.store(1);
    driverWrites(seg, 0.4);
    ShmAdapter a(&seg);
    CHECK(a.attach());
    CHECK(a.read(N).valid);
    ArmCommand c;
    c.torque_mode = true;
    for (int i = 0; i < N; ++i) c.tau[i] = 5.0f;
    a.write(c, N, false);
    // the blip: zero positions and NaN commands, alive back to 1 before the next poll
    for (int i = 0; i < N; ++i) {
        seg.joint_positions[i].store(0.0f);
        seg.torque[i].store(0.0f);
        seg.commanded_torque[i].store(std::nanf(""));
    }
    CHECK(!a.read(N).valid);                          // the zero fill is not feedback
    a.write(c, N, false);                             // the loop may still want torque for a tick
    CHECK(std::isnan(seg.commanded_torque[0].load()));   // never over the driver's NaN
    driverWrites(seg, 0.4);                           // the driver's first real sample again
    CHECK(a.read(N).valid);
    // NaN reappearing in the commands alone (a driver re-init with a stale position) also resets.
    a.write(c, N, false);
    CHECK(seg.commanded_torque[0].load() == 5.0f);
    seg.commanded_torque[0].store(std::nanf(""));
    a.write(c, N, false);
    CHECK(std::isnan(seg.commanded_torque[0].load()));
}

// I3: frozen feedback while alive stays 1 (a hung or killed driver) becomes invalid after 200 ms.
static void testFrozenFeedback() {
    std::printf("frozen feedback\n");
    static SharedFeedback seg;
    seg.alive.store(1);
    driverWrites(seg, 0.3);
    ShmAdapter a(&seg);
    CHECK(a.attach());
    int valid = 0;
    for (int k = 0; k < 250; ++k) valid += a.read(N).valid;
    CHECK(valid >= 199 && valid <= 201);              // valid for the first ~200 ticks only
    seg.joint_positions[2].store(0.51f);              // it moves again
    CHECK(a.read(N).valid);
}

// I7: out of torque after torque, the arm's gravity at its pose is written (the old idle baseline), not a
// frozen torque — libfranka ignores the stop flag and keeps applying what is there.
static void testIdleGravity() {
    std::printf("idle torque after a release\n");
    static SharedFeedback seg;
    seg.alive.store(1);
    driverWrites(seg, 0.3);
    ShmAdapter a(&seg);
    CHECK(a.attach());
    a.read(N);
    ArmCommand c, idle;
    c.torque_mode = true;
    for (int i = 0; i < N; ++i) c.tau[i] = 5.0f;
    a.write(c, N, false);
    float g[kMaxJoints] = {};
    for (int i = 0; i < N; ++i) g[i] = 1.5f + i;
    a.write(idle, N, true, g);
    CHECK(seg.commanded_torque[3].load() == 4.5f && seg.stop_torque_mode.load() == 1);
    a.write(idle, N, true);                           // no idle torque given: the last one held
    CHECK(seg.commanded_torque[3].load() == 4.5f);
}

// I8: a restarted inner loop finds the driver alive and torquing (its own last commands, finite): it adopts
// them and hands the arm back (stop flag), never NaN (a torquing Kinova reads NaN as 0 N m: the arm drops).
static void testAdoptOnAttach() {
    std::printf("attach while the driver is torquing\n");
    static SharedFeedback seg;
    seg.alive.store(1);
    driverWrites(seg, 0.3);
    for (int i = 0; i < N; ++i) seg.commanded_torque[i].store(2.0f + i);
    ShmAdapter a(&seg);
    CHECK(a.attach());
    ArmCommand idle;
    a.write(idle, N, false);
    CHECK(seg.commanded_torque[1].load() == 3.0f && seg.stop_torque_mode.load() == 1);
}

int main() {
    testContract();
    testRemap();
    testDriverReinitBlip();
    testFrozenFeedback();
    testIdleGravity();
    testAdoptOnAttach();
    if (g_fail) { std::printf("%d check(s) FAILED\n", g_fail); return 1; }
    std::printf("PASS\n");
    return 0;
}
