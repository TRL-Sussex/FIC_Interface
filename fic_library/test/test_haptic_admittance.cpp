// Haptic joint admittance (mode 4, and H-PMP's joint layer): the offset is driven
// by the interaction torque alone, desired minus measured, outside the deadband.
//
//  1. balanced  -- measured equals desired, desired non-zero (e.g. gravity):
//                  nothing pushes the joint, so its offset must not move.
//  2. push      -- measured exceeds desired by more than the deadband: the offset
//                  moves so as to relieve the push.
//  3. release   -- balanced again after the push: the offset holds where the push
//                  left it (no creep), inside its bound.
// The portable core (fic_joint_core.h, the MCU) runs the same drive: its offset must match.
//
// Needs Eigen and tinyxml2 (fic_control.h pulls in the XML loader).

#include <cmath>
#include <cstdio>
#include <vector>

#include <fic/fic_control.h>
#include <fic/fic_joint_core.h>
#include <fic/joint_params_pod.h>

static const int N = 7;
static int failures = 0;

static void check(bool ok, const char* what, double value) {
    std::printf("  %-58s %s (%.6g)\n", what, ok ? "ok" : "FAIL", value);
    if (!ok) ++failures;
}

struct Rig {
    fic::ficControl<double> ref{4.0, true, 2.0, 20, true};
    fic::core::JointCore<double> core;
    std::vector<double> q = std::vector<double>(N, 0.0);
    Rig() {
        std::vector<double> lower(N, -2.5), upper(N, 2.5);
        ref.setJointLimits(lower, upper);
        ref.setJointContinuous(std::vector<bool>(N, false));
        ref.setDeadbands(1.5, 4.0, 0.2);
        core.deadband_joint_tau = 1.5;
        core.dt = fic::dt;
        fic::ScalarParams s; s.stiffness = 200;
        fic::ListParams l;
        l.asymmetric = false;
        l.error_max = {0.05, 0.1, 0.2};
        l.force_max = {10, 20, 39};
        l.saturation_ratios = {0.9};
        fic::PlannerParams p;
        p.Plannerstiffness = 15000; p.vel_desired = 1.0; p.acc_max = 5000;
        p.damping_ratio = 0.5; p.frequency = 20;
        fic::AdmittanceParams a;
        a.InvM = 1.0; a.damping = 9.0; a.dFlow = {-5, 5}; a.flow = {-1.2, 1.2}; a.pose = {-0.2, 0.2};
        for (int i = 0; i < N; ++i) {
            core.setJointLimits(i, lower[i], upper[i]);
            core.setJointContinuous(i, false);
            ref.initJointController(i, s, l);
            ref.initJointPlanner(i, q[i], p);
            ref.initJointAdmittance(i, l, p, a);
            fic::core::JointParamsPOD pod;
            fic::core::toPOD(s, l, p, a, pod);
            core.initJoint(i, pod, q[i]);
        }
    }
    // One tick of joint 0 in haptic mode (2); returns the offset q_target_adm - q_target
    // of fic_control.h and stores the core's in *core_offset.
    double tick(double desired, double measured, double* core_offset, double release = 0.0) {
        std::vector<double> traj(N * 8, 0.0), torq(N, 0.0);
        double qt = 0.0;
        if (release > 0.0)   // H-PMP's call, with hysteresis
            ref.handleJointAdmitanceControl(0, traj, torq, qt, q[0], desired, measured, 2, N, false, release);
        else
            ref.JC.admittance(0, traj, torq, qt, q[0], desired, measured, 2, N, false);
        fic::core::JointDiag d;
        double qc = 0.0;
        core.admittance(0, qc, q[0], desired, measured, 2, false, d, release);
        *core_offset = d.q_target_adm;
        return traj[0 + 5 * N];
    }
};

int main() {
    std::printf("haptic admittance (mode 2)\n");
    Rig r;
    double off = 0, core_off = 0;
    // 1. balanced, desired = 10 N m (a gravity load), measured = 10 N m, 2 s
    for (int k = 0; k < 2000; ++k) off = r.tick(10.0, 10.0, &core_off);
    check(std::fabs(off) < 1e-9, "balanced: offset stays at 0", off);
    check(core_off == off, "balanced: core offset identical", core_off - off);
    // 2. push: measured 5 N m above desired (outside the 1.5 N m deadband), 0.5 s
    for (int k = 0; k < 500; ++k) off = r.tick(10.0, 15.0, &core_off);
    const double pushed = off;
    check(pushed < -1e-3, "push: offset moves to relieve it (negative)", pushed);
    check(pushed >= -0.2 - 1e-12, "push: offset within its bound", pushed);
    check(core_off == off, "push: core offset identical", core_off - off);
    // 3. release: balanced again, 2 s
    for (int k = 0; k < 2000; ++k) off = r.tick(10.0, 10.0, &core_off);
    check(std::fabs(off - pushed) < 0.01, "release: offset holds (no creep)", off - pushed);
    check(core_off == off, "release: core offset identical", core_off - off);
    // 4. hysteresis (H-PMP: deadband 1.5, release 0.375 N m), on a fresh rig
    std::printf("haptic admittance with deadband hysteresis\n");
    Rig h;
    const double rel = 0.25 * 1.5;
    double o = 0, oc = 0;
    for (int k = 0; k < 500; ++k) o = h.tick(10.0, 11.0, &oc, rel);          // gap -1.0: inside
    check(std::fabs(o) < 1e-12, "gap inside the deadband at rest: no yield", o);
    for (int k = 0; k < 200; ++k) o = h.tick(10.0, 12.0, &oc, rel);          // gap -2.0: engages
    const double o1 = o;
    check(o1 < -1e-4, "gap reaches the deadband: yields", o1);
    for (int k = 0; k < 200; ++k) o = h.tick(10.0, 11.0, &oc, rel);          // gap -1.0: between release and deadband
    check(o < o1 - 1e-4, "gap back inside the deadband: keeps yielding", o - o1);
    for (int k = 0; k < 1000; ++k) o = h.tick(10.0, 10.3, &oc, rel);         // gap -0.3: below release (1 s: the
                                                                             // virtual mass coasts out first)
    const double o2 = o;
    for (int k = 0; k < 500; ++k) o = h.tick(10.0, 10.3, &oc, rel);
    check(std::fabs(o - o2) < 1e-3, "gap below release: stops", o - o2);
    check(oc == o, "core offset identical (hysteresis)", oc - o);

    std::printf(failures ? "FAILED (%d)\n" : "passed\n", failures);
    return failures ? 1 : 0;
}
