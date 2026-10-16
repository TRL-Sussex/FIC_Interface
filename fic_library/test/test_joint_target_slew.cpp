// Joint target speed (fic/joint_target_slew.h): the target the joint law sees moves at most `speed` rad/s.
#include <cmath>
#include <cstdio>
#include <vector>

#include <fic/joint_target_slew.h>

static int failures = 0;
static void check(bool ok, const char* what, double v) {
    std::printf("  %-60s %s (%.6g)\n", what, ok ? "ok" : "FAIL", v);
    if (!ok) ++failures;
}

int main() {
    const double dt = 0.001;
    const std::vector<bool> cont = {true, false};
    Eigen::VectorXd q(2), target(2);
    q << 0.0, 0.0;
    target << 1.0, -0.2;

    fic::JointTargetSlew off;
    check(off.apply(target, q, 1, cont, dt).isApprox(target), "off: the commanded target as it is", 0);

    fic::JointTargetSlew s;
    s.speed = 0.5;
    Eigen::VectorXd out;
    for (int k = 0; k < 1000; ++k) out = s.apply(target, q, 1, cont, dt);          // 1 s
    check(std::abs(out[0] - 0.5) < 1e-9, "1 s at 0.5 rad/s: half of a 1 rad step", out[0]);
    check(std::abs(out[1] + 0.2) < 1e-9, "a 0.2 rad step reached within 1 s", out[1]);
    for (int k = 0; k < 1000; ++k) out = s.apply(target, q, 1, cont, dt);
    check((out - target).norm() < 1e-9, "then on the target and held there", (out - target).norm());

    Eigen::VectorXd q_far(2);
    q_far << 0.3, 0.3;
    out = s.apply(target, q_far, 2, cont, dt);                                        // mode change
    check(std::abs(out[0] - 0.3005) < 1e-9 && std::abs(out[1] - 0.2995) < 1e-9,
          "a change of mode starts again from the arm", out[0]);

    fic::JointTargetSlew w;
    w.speed = 1.0;
    Eigen::VectorXd a(2), t2(2);
    a << 3.1, 0.0;
    t2 << -3.1, 0.0;                                                                  // 0.083 rad the short way
    for (int k = 0; k < 200; ++k) out = w.apply(t2, a, 1, cont, dt);
    check(std::abs(std::remainder(out[0] - t2[0], 2 * M_PI)) < 1e-9, "continuous joint: the short way round", out[0]);

    std::printf(failures ? "FAILED (%d)\n" : "PASSED\n", failures);
    return failures ? 1 : 0;
}
