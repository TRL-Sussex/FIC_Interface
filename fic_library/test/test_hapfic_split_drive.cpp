// HAP-FIC's split admittance drive (fic_control.h cartesianHapfic, hapfic_split_drive_): the joint
// torque gap u = tau_des - tau_meas is split into the part that produces an end-effector wrench,
// which passes a deadband in task space, and the rest, which passes the joint deadband.
//
// At the goal (tau_des = 0, so the gap is the measured torque) on a fixed 6x7 Jacobian, with the
// Kinova deadbands (joint 2 N m; task 1 N, released below 0.25 N):
// (1) a gap that is a 3 N end-effector force but below 2 N m on every joint: the split drive yields,
//     the per-joint drive (the former one) does not;
// (2) a gap with no end-effector wrench (the null space of J) below the joint deadband: no yield;
// (3) the same direction above the joint deadband: yields;
// (4) hysteresis: once yielding, a force gap between the release (0.25 N) and engage (1 N) levels keeps
//     yielding, and below the release the offsets stop;
// (5) without deadbands the split drive equals the per-joint drive, tick for tick.
//
// Needs Eigen and tinyxml2 (fic_control.h pulls in the XML loader).

#include <cmath>
#include <cstdio>
#include <vector>

#include <Eigen/SVD>
#include <fic/fic_control.h>

using FC = fic::ficControl<double>;
static const int N = 7;
static int failures = 0;

static void check(bool ok, const char* what, double value) {
    std::printf("  %-70s %s (%.6g)\n", what, ok ? "ok" : "FAIL", value);
    if (!ok) ++failures;
}

static fic::CartesianAxisParams axis(bool rot) {
    fic::CartesianAxisParams a;
    a.stiffness = rot ? 50 : 200;
    a.diff_effort_max = 100;
    a.error_max = {0.05, 0.1, 0.2};
    a.force_max = rot ? std::vector<double>{2, 5, 10} : std::vector<double>{13, 20, 39};
    a.saturation_ratios = {0.9};
    a.n_saturations = 6; a.asymmetric = false; a.n_neg_sat = 3;
    a.stiffness_P = 15753; a.vel_desired = rot ? 0.3 : 1.0; a.acc_max = 5000;
    a.damping_ratio = 0.015; a.frequency = 150;
    a.admittance.InvM = 1.0; a.admittance.damping = rot ? 15.0 : 20.0;
    a.admittance.dFlow = {-5.2, 5.2};
    a.admittance.flow = rot ? std::vector<double>{-1.0, 1.0} : std::vector<double>{-0.5, 0.5};
    a.admittance.pose = rot ? std::vector<double>{-0.5, 0.5} : std::vector<double>{-0.3, 0.3};
    return a;
}

struct Rig {
    FC c{4.0, false, 2.0, 23, true};
    std::vector<double> X = {0.5, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0};
    Eigen::VectorXd q = Eigen::VectorXd::Zero(N), q_ik = Eigen::VectorXd::Zero(N), g = Eigen::VectorXd::Zero(N);
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, N);
    fic::CartesianAllParams cp;
    Rig(bool split, double joint_db = 2.0, double task_f = 1.0, double task_t = 0.1) {
        std::vector<double> lower(N, -2.5), upper(N, 2.5);
        c.setJointLimits(lower, upper);
        c.setJointContinuous(std::vector<bool>(N, false));
        c.setDeadbands(joint_db, 4.0, 0.2);
        fic::ScalarParams s; s.stiffness = 200;
        fic::ListParams l; l.asymmetric = false;
        l.error_max = {0.1, 0.2, 0.4}; l.force_max = {10, 20, 39}; l.saturation_ratios = {0.9};
        fic::PlannerParams p; p.Plannerstiffness = 15000; p.vel_desired = 1.0; p.acc_max = 5000;
        p.damping_ratio = 0.5; p.frequency = 20;
        fic::AdmittanceParams a; a.InvM = 1.0; a.damping = 54.0; a.dFlow = {-5, 5}; a.flow = {-1.2, 1.2};
        a.pose = {-0.4, 0.4};
        for (int k = 0; k < 3; ++k) cp.pos_axes[k] = axis(false);
        cp.rot_axis = axis(true);
        for (int r = 0; r < 6; ++r) for (int col = 0; col < N; ++col)     // a fixed, well-conditioned 6x7 J
            J(r, col) = (r == col ? 0.6 : 0.0) + 0.05 * std::sin(1.0 + r * 1.7 + col * 0.9);
        for (int i = 0; i < N; ++i) {
            c.initJointController(i, s, l);
            c.initJointPlanner(i, 0.0, p);
            c.initJointAdmittance(i, l, p, a);
        }
        c.initHapficCartesian(cp, X);
        c.setHapficSplitDrive(split, task_f, task_t);
    }
    // One HAP-FIC tick (host mode 6) with the measured torque `meas` (the Cartesian-mode convention, so
    // the gap is tau_des + meas = meas at the goal); returns each joint's admittance set-point.
    Eigen::VectorXd tick(const Eigen::VectorXd& meas) {
        std::vector<double> traj(N * 8 + 16, 0.0), torq(N, 0.0), w(6, 0.0), m(meas.data(), meas.data() + N);
        c.cartesianHapfic(traj, torq, q, g, X, X, J, 0, w, m, q_ik, N);
        Eigen::VectorXd qa(N);
        for (int i = 0; i < N; ++i) qa[i] = traj[i + 5 * N];
        return qa;
    }
};

static Eigen::VectorXd force_gap(const Eigen::MatrixXd& J, double fx) {
    Eigen::Matrix<double, 6, 1> f = Eigen::Matrix<double, 6, 1>::Zero();
    f[0] = fx;
    return J.transpose() * f;
}

int main() {
    std::printf("(1) a 3 N end-effector force, below 2 N m on every joint\n");
    {
        Rig split(true), joint(false);
        const Eigen::VectorXd u = force_gap(split.J, 3.0);
        check(u.cwiseAbs().maxCoeff() < 2.0, "every joint's share is inside the 2 N m joint deadband", u.cwiseAbs().maxCoeff());
        Eigen::VectorXd a, b;
        for (int k = 0; k < 300; ++k) { a = split.tick(u); b = joint.tick(u); }
        check(a.cwiseAbs().maxCoeff() > 1e-4, "split drive: the joints yield", a.cwiseAbs().maxCoeff());
        check(b.cwiseAbs().maxCoeff() == 0.0, "per-joint drive (former): nothing moves", b.cwiseAbs().maxCoeff());
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Rig(true).J, Eigen::ComputeFullV);
    Eigen::VectorXd n = svd.matrixV().col(N - 1);     // J n = 0: no end-effector wrench
    n /= n.cwiseAbs().maxCoeff();

    std::printf("(2) no end-effector wrench, below the joint deadband\n");
    {
        Rig split(true);
        Eigen::VectorXd a;
        for (int k = 0; k < 300; ++k) a = split.tick(1.5 * n);
        check(a.cwiseAbs().maxCoeff() == 0.0, "no yield", a.cwiseAbs().maxCoeff());
    }

    std::printf("(3) no end-effector wrench, above the joint deadband\n");
    {
        Rig split(true);
        Eigen::VectorXd a;
        for (int k = 0; k < 300; ++k) a = split.tick(2.5 * n);
        check(a.cwiseAbs().maxCoeff() > 1e-4, "yields", a.cwiseAbs().maxCoeff());
    }

    std::printf("(4) hysteresis: engaged at 1 N, released below 0.25 N\n");
    {
        Rig split(true);
        Eigen::VectorXd a;
        for (int k = 0; k < 100; ++k) a = split.tick(force_gap(split.J, 3.0));
        Eigen::VectorXd a0 = a;
        for (int k = 0; k < 200; ++k) a = split.tick(force_gap(split.J, 0.5));
        Eigen::VectorXd a1 = a;
        for (int k = 0; k < 100; ++k) a = split.tick(force_gap(split.J, 0.5));
        check((a - a1).norm() > 1e-6, "a 0.5 N gap keeps yielding once engaged", (a - a1).norm());
        for (int k = 0; k < 600; ++k) a = split.tick(force_gap(split.J, 0.2));
        Eigen::VectorXd a2 = a;
        for (int k = 0; k < 100; ++k) a = split.tick(force_gap(split.J, 0.2));
        check((a - a2).norm() < 1e-9, "below 0.25 N the offsets stop", (a - a2).norm());
        Rig fresh(true);
        for (int k = 0; k < 300; ++k) a = fresh.tick(force_gap(fresh.J, 0.5));
        check(a.cwiseAbs().maxCoeff() == 0.0, "a 0.5 N gap from rest does not engage", a.cwiseAbs().maxCoeff());
        (void)a0;
    }

    std::printf("(5) without deadbands the split drive equals the per-joint drive\n");
    {
        Rig split(true, 0.0, 0.0, 0.0), joint(false, 0.0, 0.0, 0.0);
        Eigen::VectorXd u(N);
        for (int i = 0; i < N; ++i) u[i] = 0.7 * std::sin(0.3 + 1.1 * i);
        double diff = 0.0;
        for (int k = 0; k < 300; ++k) diff = std::fmax(diff, (split.tick(u) - joint.tick(u)).cwiseAbs().maxCoeff());
        check(diff < 1e-9, "same admittance set-points, every tick", diff);
    }

    std::printf(failures ? "FAILED (%d)\n" : "passed\n", failures);
    return failures ? 1 : 0;
}
