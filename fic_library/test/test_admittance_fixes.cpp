// The library fixes of 18 Oct 2026, each in fic_control.h / FIC.cpp and, where it has one, in
// fic_joint_core.h (the MCU's and the headset's copy):
//
//   1. Push-and-stay joint admittance on a joint whose range leaves out zero (Franka joints 4
//      and 6) no longer moves the arm by itself as it starts.
//   2. A profile with fewer band starts than bands keeps 0.9 for the rest (it read past the end).
//   3. Two controllers in one process no longer share joint limits or Cartesian admittance state.
//   4. The Cartesian admittance bounds are per robot (setCartesianAdmittanceLimits).

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

struct Joint { fic::ScalarParams s; fic::ListParams l; fic::PlannerParams p; fic::AdmittanceParams a; };

Joint frankaLikeJoint() {
    Joint j;
    j.s.stiffness = 200;
    j.l.asymmetric = false; j.l.error_max = {0.05, 0.5}; j.l.force_max = {10, 20}; j.l.saturation_ratios = {0.9};
    j.p.Plannerstiffness = 15000; j.p.vel_desired = 3; j.p.acc_max = 5000; j.p.damping_ratio = 0.01; j.p.frequency = 150;
    j.a.InvM = 1; j.a.damping = 10; j.a.dFlow = {-5, 5}; j.a.flow = {-1, 1}; j.a.pose = {-0.1, 0.1};
    return j;
}

// 1. Franka joint 4 ([-3.0421, -0.1518] rad) and joint 6 ([0.5445, 4.5169] rad) in mode 3.
void offsetOnARangeWithoutZero() {
    const double ranges[2][2] = {{-3.0421, -0.1518}, {0.5445, 4.5169}};
    const double start[2] = {-2.2, 1.9};
    for (int r = 0; r < 2; ++r) {
        fic::ficControl<double> c(4.0, true, 2.0, 1, true);
        c.setJointLimits({ranges[r][0]}, {ranges[r][1]});
        c.setJointContinuous({false});
        c.setDeadbands(1.5, 4.0, 0.2);
        Joint j = frankaLikeJoint();
        c.initJointController(0, j.s, j.l);
        c.initJointPlanner(0, start[r], j.p);
        c.initJointAdmittance(0, j.l, j.p, j.a);
        fic::core::JointCore<double> core;
        core.setJointLimits(0, ranges[r][0], ranges[r][1]);
        core.deadband_joint_tau = 1.5;
        fic::core::JointParamsPOD pod;
        fic::core::toPOD(j.s, j.l, j.p, j.a, pod);
        core.initJoint(0, pod, start[r]);

        std::vector<double> traj(8, 0.0), torq(1, 0.0);
        double worst_host = 0, worst_core = 0;
        for (int k = 0; k < 2000; ++k) {           // 2 s at rest, no interaction torque
            double qt = start[r], qc = start[r];
            fic::core::JointDiag d;
            c.JC.admittance(0, traj, torq, qt, start[r], 0.0, 0.0, 1, 1, true);
            core.admittance(0, qc, start[r], 0.0, 0.0, 1, true, d);
            worst_host = std::fmax(worst_host, std::fabs(traj[5] - start[r]));
            worst_core = std::fmax(worst_core, std::fabs(d.q_target_adm - start[r]));
        }
        CHECK(worst_host == 0.0 && worst_core == 0.0,
              "mode 3 on [%.4f, %.4f] holds the commanded target with no push (moved %.3g / %.3g rad; was 0.15 / 0.42 before the fix)",
              ranges[r][0], ranges[r][1], worst_host, worst_core);

        // A push still yields, and the joint limit still stops it without winding up.
        const double push = r == 0 ? 15.0 : -15.0;   // toward the limit nearer zero
        double q = start[r];
        for (int k = 0; k < 20000; ++k) {
            double qt = start[r];
            c.JC.admittance(0, traj, torq, qt, q, push, 0.0, 1, 1, false);
            q = traj[5];
        }
        const double edge = r == 0 ? ranges[0][1] : ranges[1][0];
        // At the limit the step that would pass it is taken back, so the target rests at the limit
        // or one step inside it.
        CHECK(std::fabs(q - edge) < 2e-3, "a long push reaches the joint limit %.4f and stops there (%.6f)", edge, q);
        double qt = start[r];
        for (int k = 0; k < 50; ++k) c.JC.admittance(0, traj, torq, qt, q, -push, 0.0, 1, 1, false);
        CHECK(std::fabs(traj[5] - edge) > 1e-4, "pushing back leaves the limit at once (no wound-up offset): %.6f", traj[5]);
    }
}

// 2. Six bands, three band starts.
void shortBandStartList() {
    FIC<double>::Controller<double> c(6.0);
    Eigen::VectorXd xb(3); xb << 0.02, 0.05, 0.1;
    Eigen::VectorXd fm(3); fm << 10, 20, 30;
    Eigen::VectorXd ss(3); ss << 0.7, 0.8, 0.85;
    c.set_x_b(xb); c.setE_max(fm); c.setS_Start(ss);
    c.setStiffness(Eigen::VectorXd::Constant(2, 500.0));
    const Eigen::VectorXd s = c.getS_Start();
    CHECK(s.size() == 6 && s(3) == 0.9 && s(4) == 0.9 && s(5) == 0.9 && s(0) == 0.7,
          "three band starts for six bands: the rest are 0.9 (size %d)", (int)s.size());

    fic::core::SatController<double> p;
    p.reconfigure(6, false, 3);
    const double xbv[3] = {0.02, 0.05, 0.1}, fmv[3] = {10, 20, 30}, ssv[3] = {0.7, 0.8, 0.85};
    p.set_x_b(xbv, 3); p.setE_max(fmv, 3); p.setS_Start(ssv, 3); p.setStiffness(500.0);
    int mismatch = 0;
    for (int k = 0; k < 4000; ++k) {
        const double e = 0.12 * std::sin(0.003 * k) * std::cos(0.0007 * k);
        const double a = c.control(0, e), b = p.control(e);
        if (a != b || !std::isfinite(a)) ++mismatch;
    }
    CHECK(mismatch == 0, "the host law and the MCU's agree exactly on that profile (%d differences)", mismatch);
}

// 3. Two controllers in one process.
void instancesAreIndependent() {
    fic::ficControl<double> a(4.0, true, 2.0, 7, true), b(4.0, true, 2.0, 7, true);
    a.setJointLimits(std::vector<double>(7, -1.0), std::vector<double>(7, 1.0));
    fic::ficControl<double> c(4.0, true, 2.0, 7, true);   // constructing another used to reset them
    (void)c;
    b.setJointLimits(std::vector<double>(7, -0.2), std::vector<double>(7, 0.2));
    CHECK(a.jointUpper_[0] == 1.0 && b.jointUpper_[0] == 0.2, "joint limits are each robot's own (%g, %g)",
          a.jointUpper_[0], b.jointUpper_[0]);

    // Cartesian admittance state: a push on robot a does not move robot b's goal.
    fic::CartesianAllParams cart;
    std::vector<fic::ScalarParams> js(7); std::vector<fic::ListParams> jl(7);
    std::vector<fic::PlannerParams> jp(7); std::vector<fic::AdmittanceParams> ja(7);
    fic::ParameterLoader::loadHardcodedParams(7, js, jl, jp, ja, cart);
    for (int i = 0; i < 3; ++i) { cart.pos_axes[i].admittance = {1.0, 10.0, {-5, 5}, {-1, 1}, {-0.1, 0.1}};
                                  cart.pos_axes[i].saturation_ratios = {0.9}; }
    cart.rot_axis.admittance = {1.0, 10.0, {-5, 5}, {-1, 1}, {-0.1, 0.1}};
    cart.rot_axis.saturation_ratios = {0.9};
    auto setUp = [&](fic::ficControl<double>& r) {
        for (int ax = 0; ax < 3; ++ax) { r.initCartesianController(ax, cart.pos_axes[ax]); r.initCartesianPlanner(ax, 0.4, cart.pos_axes[ax]);
                                         r.initCartesianAdmittance(ax, cart.pos_axes[ax]); }
        r.initCartesianController(3, cart.rot_axis);
        for (int ax = 3; ax < 6; ++ax) r.initCartesianAdmittance(ax, cart.rot_axis);
    };
    setUp(a); setUp(b);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(7), qt = Eigen::VectorXd::Zero(7), g = Eigen::VectorXd::Zero(7), V = Eigen::VectorXd::Zero(6);
    Eigen::MatrixXd J = Eigen::MatrixXd::Identity(6, 7);
    const std::vector<double> X = {0.4, 0, 0.5, 0, 0, 0, 1}, Xt = X;
    std::vector<double> traj, torq, meas(7, 0.0);
    for (int k = 0; k < 500; ++k) a.RC.admittance(traj, torq, q, g, X, Xt, J, 1, {-20, 0, 0, 0, 0, 0}, meas, qt, V, 7);
    b.RC.admittance(traj, torq, q, g, X, Xt, J, 1, {0, 0, 0, 0, 0, 0}, meas, qt, V, 7);
    CHECK(std::fabs(a.cart_adm_offset_[0]) > 0.01 && b.cart_adm_offset_[0] == 0.0 && traj[7 * 8] == 0.4,
          "robot a pushed %.3f m; robot b's goal did not move (trajectory_data goal x %.3f)", a.cart_adm_offset_[0], traj[7 * 8]);
}

// 4. The Cartesian admittance bounds per robot.
void cartesianLimits() {
    fic::ficControl<double> c(4.0, true, 2.0, 7, true);
    CHECK(c.cart_adm_push_m_ == 0.30 && c.cart_adm_haptic_m_ == 0.05 && c.cart_adm_push_rad_ == M_PI / 6.0 &&
          c.cart_adm_haptic_rad_ == M_PI / 36.0, "defaults: 0.30 m / 30 deg push-and-stay, 0.05 m / 5 deg haptic");
    c.setCartesianAdmittanceLimits(0.10, -1.0, NAN, 0.02);
    CHECK(c.cart_adm_push_m_ == 0.10 && c.cart_adm_push_rad_ == M_PI / 6.0 && c.cart_adm_haptic_m_ == 0.05 &&
          c.cart_adm_haptic_rad_ == 0.02, "set: bad values keep the previous bound");

    fic::CartesianAllParams cart;
    std::vector<fic::ScalarParams> js(7); std::vector<fic::ListParams> jl(7);
    std::vector<fic::PlannerParams> jp(7); std::vector<fic::AdmittanceParams> ja(7);
    fic::ParameterLoader::loadHardcodedParams(7, js, jl, jp, ja, cart);
    for (int i = 0; i < 3; ++i) { cart.pos_axes[i].admittance = {1.0, 10.0, {-5, 5}, {-1, 1}, {-0.1, 0.1}};
                                  cart.pos_axes[i].saturation_ratios = {0.9}; }
    for (int ax = 0; ax < 3; ++ax) { c.initCartesianController(ax, cart.pos_axes[ax]); c.initCartesianPlanner(ax, 0.4, cart.pos_axes[ax]);
                                     c.initCartesianAdmittance(ax, cart.pos_axes[ax]); }
    cart.rot_axis.saturation_ratios = {0.9};
    c.initCartesianController(3, cart.rot_axis);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(7), qt = Eigen::VectorXd::Zero(7), g = Eigen::VectorXd::Zero(7), V = Eigen::VectorXd::Zero(6);
    Eigen::MatrixXd J = Eigen::MatrixXd::Identity(6, 7);
    const std::vector<double> X = {0.4, 0, 0.5, 0, 0, 0, 1};
    std::vector<double> traj, torq, meas(7, 0.0);
    for (int k = 0; k < 5000; ++k) c.RC.admittance(traj, torq, q, g, X, X, J, 1, {-40, 0, 0}, meas, qt, V, 7);
    CHECK(std::fabs(c.cart_adm_offset_[0] - 0.10) < 1e-9, "push-and-stay stops at the robot's 0.10 m (%.4f)", c.cart_adm_offset_[0]);
}

} // namespace

int main() {
    offsetOnARangeWithoutZero();
    shortBandStartList();
    instancesAreIndependent();
    cartesianLimits();
    std::printf("%s (%d failure%s)\n", failures ? "FAILED" : "PASSED", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
