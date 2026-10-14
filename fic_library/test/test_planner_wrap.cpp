// Joint planner on a continuous joint crossing +-pi.
//
// A step of the goal across +-pi (as the admittance-shifted IK target of joint 3 of the Kinova,
// which rests at +-pi, makes) must be tracked like the same step anywhere else: the planner's
// distance to the goal, and its "goal moved" check, have to use the wrapped difference.
// Unwrapped, the 2*pi seam looks like a 6.28 rad move: the planner retunes for it and lags.
//
// Needs Eigen and tinyxml2 (fic_control.h pulls in the XML loader).

#include <cmath>
#include <cstdio>
#include <vector>

#include <fic/fic_control.h>

static const int N = 7;
static const int J = 2;   // the continuous joint under test

static double wrapPi(double a) { return std::atan2(std::sin(a), std::cos(a)); }

// Wrapped |planner output - goal| 0.2 s after the goal steps by +step from start (the planner and
// its goal on opposite sides of +-pi when start is just below pi), and the largest excursion away
// from the goal on the far side of start (moving the wrong way), over 1 s.
static void stepResponse(double start, double step, double* lag_at_200ms, double* wrong_way) {
    fic::ficControl<double> ref(4.0, true, 2.0, 20, true);
    std::vector<double> lower(N, -2.5), upper(N, 2.5);
    std::vector<bool> cont(N, false);
    cont[J] = true; lower[J] = -M_PI; upper[J] = M_PI;
    ref.setJointLimits(lower, upper);
    ref.setJointContinuous(cont);
    fic::ScalarParams s; s.stiffness = 200;
    fic::ListParams l; l.asymmetric = false; l.error_max = {0.05, 0.1, 0.2};
    l.force_max = {10, 20, 39}; l.saturation_ratios = {0.9};
    fic::PlannerParams p; p.Plannerstiffness = 15780; p.vel_desired = 3.0; p.acc_max = 5000;
    p.damping_ratio = 0.01; p.frequency = 20;
    std::vector<double> q(N, 0.0);
    q[J] = wrapPi(start);
    for (int i = 0; i < N; ++i) {
        ref.initJointController(i, s, l);
        ref.initJointPlanner(i, q[i], p);
    }
    const double goal = wrapPi(start + step);
    *wrong_way = 0;
    for (int k = 0; k <= 1000; ++k) {             // 1 s at 1 kHz
        std::vector<double> traj(N * 8, 0.0), torq(N, 0.0);
        ref.JC.impedanceWithPlanner(J, traj, torq, k < 10 ? wrapPi(start) : goal, q[J], N);
        const double plan = traj[J + 0 * N];
        const double back = -wrapPi(plan - wrapPi(start)) * (step > 0 ? 1 : -1);   // > 0: behind start
        *wrong_way = std::fmax(*wrong_way, back);
        if (k == 210) *lag_at_200ms = std::fabs(wrapPi(plan - goal));
        q[J] = plan;                              // an ideal joint that follows its planner
    }
}

int main() {
    int failures = 0;
    double lag_away, wrong_away, lag_seam, wrong_seam;
    stepResponse(0.0, 0.08, &lag_away, &wrong_away);               // 0 -> 0.08 rad
    stepResponse(M_PI - 0.04, 0.08, &lag_seam, &wrong_seam);       // pi-0.04 -> -pi+0.04 (same 0.08 rad)
    std::printf("planner wrap, 0.08 rad step: lag after 0.2 s %.4g rad away from +-pi, %.4g rad across +-pi; "
                "wrong-way excursion %.4g / %.4g rad\n", lag_away, lag_seam, wrong_away, wrong_seam);
    if (!(lag_seam < 2.0 * lag_away + 1e-3)) {
        std::printf("  FAIL: a step across +-pi is tracked more slowly than the same step elsewhere\n");
        ++failures;
    }
    if (!(wrong_seam < wrong_away + 1e-3)) {
        std::printf("  FAIL: across +-pi the planner moves away from its goal\n");
        ++failures;
    }
    std::printf(failures ? "FAILED\n" : "passed\n");
    return failures ? 1 : 0;
}
