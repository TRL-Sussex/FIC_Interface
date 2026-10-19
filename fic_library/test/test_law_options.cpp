// The FIC law options of 5.11.0, in FIC.cpp and fic_joint_core.h (the MCU's and the headset's copy):
//
//   1. Restart on a stall: an overdamped joint stepped toward a target used to rest at half the step
//      with no torque; it now reaches the target. Off, the law is the one before.
//   2. A lightly damped joint, and a joint held still while the law still pushes, are left alone.
//   3. The planner's acc_max caps its acceleration (0: no cap); the shipped 5000 never binds.
//   4. The planner's frequency cap and speed margin are settings (setShape / setLawOptions).
//   5. The MCU's copy gives the same torques as the library with the options on and off.
//   6. The joint planners never plan faster than the robot's max joint speed; the Cartesian
//      planners (metres, on the same channels) are not capped by it.
//   7. No restart where an admittance moves the goal: joint admittance gives the same torques with
//      the option on and off (library and MCU copy), and the restart is back on afterwards.

#include <cmath>
#include <cstdio>
#include <vector>

#include <fic/fic_control.h>
#include <fic/fic_joint_core.h>
#include <fic/joint_params_pod.h>

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { std::printf("FAIL: " __VA_ARGS__); std::printf("\n"); ++failures; } \
                              else { std::printf("ok:   " __VA_ARGS__); std::printf("\n"); } } while (0)

namespace {

using Ctrl = FIC<double>::Controller<double>;
using Plan = FIC<double>::Planner<double>;

// A Kinova-like joint profile: K0 200 N m/rad, bands at 0.05 / 0.1 / 0.2 rad, 10 / 20 / 39 N m.
void kinovaProfile(Ctrl& c) {
    Eigen::VectorXd xb(3); xb << 0.05, 0.1, 0.2;
    Eigen::VectorXd fm(3); fm << 10, 20, 39;
    c.set_x_b(xb); c.setE_max(fm);
    c.setS_Start(Eigen::VectorXd::Constant(1, 0.9));
    c.setStiffness(Eigen::VectorXd::Constant(2, 200.0));
}

struct Run { double error; double hold_torque; };

// One joint (inertia 0.3 kg m^2, viscous damping b, Coulomb friction fc) stepped by `step` rad at
// 1 kHz, the law driving it straight (no planner); `hold` > 0 clamps the joint still there instead.
Run stepResponse(bool restart, double b, double fc, double step, double seconds, double hold = 0.0) {
    FIC<double> f(6, false, 3, 1, false);
    Ctrl& c = f.getController(0);
    kinovaProfile(c);
    c.setRestartOnStall(restart);
    double q = 0, v = 0, tau = 0;
    const double I = 0.3, dt = 0.001;
    for (int k = 0; k < static_cast<int>(seconds / dt); ++k) {
        tau = c.control(0, step - q);
        if (hold > 0 && step - q <= hold) { v = 0; continue; }
        double a = tau - b * v;
        if (std::fabs(v) < 1e-6 && std::fabs(a) <= fc) { v = 0; a = 0; }
        else a -= fc * ((v != 0 ? v : a) > 0 ? 1 : -1);
        v += dt * a / I;
        q += dt * v;
    }
    return {step - q, tau};
}

void restartOnStall() {
    const double step = 0.0436;   // 2.5 deg
    Run before = stepResponse(false, 30, 0, step, 5);
    Run after = stepResponse(true, 30, 0, step, 5);
    CHECK(std::fabs(before.error - step / 2) < 1e-4, "off: overdamped joint rests at half the step (%.5f rad)", before.error);
    CHECK(std::fabs(before.hold_torque) < 0.01, "off: with no torque (%.4f N m)", before.hold_torque);
    CHECK(std::fabs(after.error) < 1e-4, "on: it reaches the target (%.6f rad left)", after.error);

    Run big = stepResponse(true, 30, 0, 0.3, 5);
    CHECK(std::fabs(big.error) < 1e-4, "on: a 0.3 rad step too (%.6f rad left)", big.error);

    Run fr = stepResponse(true, 30, 0.5, step, 5);
    CHECK(std::fabs(fr.error) < 0.005, "on, with friction 0.5 N m: stops within the friction band (%.5f rad)", fr.error);
}

void ordinaryMovesUnchanged() {
    for (double b : {0.5, 5.0}) {
        Run off = stepResponse(false, b, 0, 0.3, 3), on = stepResponse(true, b, 0, 0.3, 3);
        CHECK(std::fabs(off.error - on.error) < 1e-5, "lightly damped (b = %.1f): same end with and without (%.2g vs %.2g)",
              b, off.error, on.error);
    }
    // Held still 0.28 rad short of a 0.3 rad step: the law still pushes with more than half its
    // peak, so there is no restart, and the torque is the same with the option on and off.
    Run off = stepResponse(false, 30, 0, 0.3, 2, 0.28), on = stepResponse(true, 30, 0, 0.3, 2, 0.28);
    CHECK(off.hold_torque == on.hold_torque, "held still against the law: same torque (%.3f vs %.3f N m)",
          off.hold_torque, on.hold_torque);
}

// The planner moving 0.5 rad with vel_desired 3 at 1 kHz: its largest acceleration.
double peakAcceleration(double acc_max) {
    Plan p;
    p.setVelDesired(3.0); p.setDampingRatio(0.01); p.setAccMax(acc_max);
    p.setPosLimits(-10, 10); p.resetState(0);
    double last = 0, lastv = 0, peak = 0;
    for (int k = 0; k < 3000; ++k) {
        const double x = p.plan(0.5, 0.001), v = (x - last) / 0.001;
        if (k > 1) peak = std::max(peak, std::fabs(v - lastv) / 0.001);
        last = x; lastv = v;
    }
    return peak;
}

void accelerationCap() {
    const double none = peakAcceleration(0), shipped = peakAcceleration(5000), capped = peakAcceleration(10);
    CHECK(none == shipped, "acc_max 5000 never binds (peak %.1f rad/s^2 either way)", shipped);
    CHECK(capped <= 10.0 + 1e-6 && capped < shipped, "acc_max 10 caps the peak (%.2f against %.1f rad/s^2)", capped, shipped);
}

void plannerShape() {
    Plan p;
    CHECK(p.getMaxFrequency() == 8.75 && p.getSpeedMargin() == 1.5708, "defaults are the old constants");
    p.setShape(-1, std::nan(""));
    CHECK(p.getMaxFrequency() == 8.75 && p.getSpeedMargin() == 1.5708, "bad values keep the ones before");

    // A lower speed margin: the set-point moves no faster than margin x vel_desired.
    auto peakSpeed = [](double margin) {
        Plan q; q.setVelDesired(1.0); q.setDampingRatio(0.01); q.setPosLimits(-10, 10); q.resetState(0);
        q.setShape(8.75, margin);
        double last = 0, peak = 0;
        for (int k = 0; k < 3000; ++k) { const double x = q.plan(1.0, 0.001); peak = std::max(peak, (x - last) / 0.001); last = x; }
        return peak;
    };
    const double slow = peakSpeed(0.5), shipped = peakSpeed(1.5708);
    CHECK(slow <= 0.5 + 1e-9 && slow < shipped, "margin 0.5: the set-point stays within 0.5 x vel_desired "
          "(%.3f rad/s; %.3f with 1.5708)", slow, shipped);

    fic::ficControl<double> c(4.0, true, 2.0, 3, true);
    c.setLawOptions(false, 5.0, 1.2);
    CHECK(!c.controller(2).getRestartOnStall() && c.planner(2).getMaxFrequency() == 5.0 &&
          c.planner(2).getSpeedMargin() == 1.2, "setLawOptions reaches every channel");
}

void jointSpeedCap() {
    fic::PlannerParams pp; pp.Plannerstiffness = 15000; pp.vel_desired = 1.7453; pp.acc_max = 5000;
    pp.damping_ratio = 0.005; pp.frequency = 150;
    fic::CartesianAxisParams ax; ax.stiffness = 500; ax.error_max = {0.05}; ax.force_max = {20};
    ax.vel_desired = 1.7453; ax.acc_max = 5000; ax.damping_ratio = 0.005;

    auto peak = [](fic::ficControl<double>& c, double goal) {
        double last = c.planner(0).plan(0.0, 0.001), top = 0;
        for (int k = 0; k < 3000; ++k) { const double x = c.planner(0).plan(goal, 0.001); top = std::max(top, (x - last) / 0.001); last = x; }
        return top;
    };
    fic::ficControl<double> c(4.0, true, 2.0, 1, true);
    c.setJointLimits({-3.0}, {3.0});
    c.initJointPlanner(0, 0.0, pp);
    const double free = peak(c, 2.0);
    c.setJointSpeedLimit(1.0);
    c.initJointPlanner(0, 0.0, pp);
    const double capped = peak(c, 2.0);
    CHECK(free > 1.5 && capped <= 1.0 + 1e-9, "joint planner capped at the max joint speed (%.3f rad/s; %.3f without)",
          capped, free);

    c.initJointPlanner(0, 0.0, pp);
    c.setJointSpeedLimit(0.5);                           // the Studio changes it while running
    CHECK(peak(c, 2.0) <= 0.5 + 1e-9, "a change applies to the running joint planner");

    c.initCartesianPlanner(0, 0.0, ax);
    CHECK(peak(c, 2.0) > 1.5, "a Cartesian planner on the same channel is not capped (%.3f m/s)", peak(c, 2.0));
    c.setJointSpeedLimit(0.3);
    CHECK(peak(c, 4.0) > 1.5, "nor by a change while it runs");
}

// The MCU's copy against the library: same joint, same inputs, options on and off.
void coreMatchesLibrary() {
    for (bool restart : {true, false}) {
        fic::ScalarParams s{200.0, 100.0};
        fic::ListParams l; l.asymmetric = false; l.error_max = {0.05, 0.1, 0.2}; l.force_max = {10, 20, 39};
        l.saturation_ratios = {0.9};
        fic::PlannerParams pp; pp.Plannerstiffness = 15000; pp.vel_desired = 1.0; pp.acc_max = 30;
        pp.damping_ratio = 0.01; pp.frequency = 150;
        fic::AdmittanceParams ap{1.0, 10.0, {-5, 5}, {-1, 1}, {-0.1, 0.1}};

        fic::ficControl<double> c(4.0, true, 2.0, 1, true);
        c.setJointLimits({-2.5}, {2.5});
        c.setLawOptions(restart, 6.0, 1.3);
        c.setJointSpeedLimit(restart ? 0.04 : 0.0);
        c.initJointController(0, s, l);
        c.initJointPlanner(0, 0.0, pp);
        c.initJointAdmittance(0, l, pp, ap);

        fic::core::JointCore<double> core;
        core.restart_on_stall = restart; core.planner_max_hz = 6.0; core.planner_speed_margin = 1.3;
        core.joint_speed_limit = restart ? 0.04 : 0.0;
        core.setJointLimits(0, -2.5, 2.5);
        fic::core::JointParamsPOD pod;
        fic::core::toPOD(s, l, pp, ap, pod);
        core.initJoint(0, pod, 0.0);

        double q = 0, v = 0, worst = 0;
        for (int k = 0; k < 4000; ++k) {
            std::vector<double> traj(8, 0.0), torq(1, 0.0);
            const double goal = (k / 1000) * 0.0436;
            c.JC.impedanceWithPlanner(0, traj, torq, goal, q, 1);
            fic::core::JointDiag d;
            const double tc = core.impedanceWithPlanner(0, goal, q, d);
            worst = std::max(worst, std::fabs(tc - torq[0]));
            v += 0.001 * (torq[0] - 30 * v) / 0.3;
            q += 0.001 * v;
        }
        CHECK(worst == 0.0, "MCU copy matches the library, restart %s, speed cap %s (largest difference %.3g N m)",
              restart ? "on" : "off", restart ? "0.04 rad/s" : "off", worst);
    }
}

// A slow, damped push in Joint Admittance (mode 3) through the library and the MCU copy.
void admittanceUntouched() {
    std::vector<double> torques[2];
    for (int on = 0; on < 2; ++on) {
        fic::ScalarParams s{200.0, 100.0};
        fic::ListParams l; l.asymmetric = false; l.error_max = {0.05, 0.1, 0.2}; l.force_max = {10, 20, 39};
        l.saturation_ratios = {0.9};
        fic::PlannerParams pp; pp.Plannerstiffness = 15000; pp.vel_desired = 3.0; pp.acc_max = 5000;
        pp.damping_ratio = 0.01; pp.frequency = 150;
        fic::AdmittanceParams ap{1.0, 10.0, {-5, 5}, {-1, 1}, {-0.1, 0.1}};
        fic::ficControl<double> c(4.0, true, 2.0, 1, true);
        c.setJointLimits({-2.5}, {2.5});
        c.setLawOptions(on == 1, 8.75, 1.5708);
        c.initJointController(0, s, l); c.initJointPlanner(0, 0.0, pp); c.initJointAdmittance(0, l, pp, ap);
        fic::core::JointCore<double> core;
        core.restart_on_stall = (on == 1);
        core.setJointLimits(0, -2.5, 2.5);
        fic::core::JointParamsPOD pod; fic::core::toPOD(s, l, pp, ap, pod); core.initJoint(0, pod, 0.0);
        double q = 0, v = 0, qt = 0, qc = 0, worst = 0;
        for (int k = 0; k < 6000; ++k) {
            const double push = (k > 500 && k < 2500) ? -6.0 : 0.0;     // N m on the joint
            std::vector<double> traj(8, 0.0), torq(1, 0.0);
            c.JC.admittance(0, traj, torq, qt, q, 0.0, -push, 1, 1, true);
            fic::core::JointDiag d;
            worst = std::max(worst, std::fabs(core.admittance(0, qc, q, 0.0, -push, 1, true, d) - torq[0]));
            torques[on].push_back(torq[0]);
            v += 0.001 * (torq[0] + push - 40 * v) / 0.3;
            q += 0.001 * v;
        }
        CHECK(worst == 0.0, "mode 3, restart %s: MCU copy matches the library", on ? "on" : "off");
        if (on) CHECK(c.controller(0).getRestartOnStall(), "the restart is back on after the admittance path");
    }
    CHECK(torques[0] == torques[1], "mode 3: same torques with the restart on and off");
}

}  // namespace

int main() {
    restartOnStall();
    ordinaryMovesUnchanged();
    accelerationCap();
    plannerShape();
    jointSpeedCap();
    coreMatchesLibrary();
    admittanceUntouched();
    std::printf(failures ? "%d FAILED\n" : "all passed\n", failures);
    return failures ? 1 : 0;
}
