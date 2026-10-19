// H-PMP's joint layer in the inner loop (JointCore: splitDrive + admittance with the drive, as
// inner_loop.h runs it for Hapfic with the Jacobian) against fic_control.h's cartesianHpmp on the same
// inputs: same params, same Jacobian, same desired and measured torques, every tick. The joint torques
// must agree to 1e-9 N m (Eigen's LDLT against JointCore's own 6x6 solve).
//
// Needs Eigen and tinyxml2 (fic_control.h pulls in the XML loader).

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include <fic/fic_control.h>
#include <fic/fic_joint_core.h>
#include <fic/joint_params_pod.h>

using FC = fic::ficControl<double>;
static const int N = 7;

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

int main() {
    const double joint_db = 2.0, task_f = 4.0, task_t = 0.2;
    fic::ScalarParams s; s.stiffness = 200;
    fic::ListParams l; l.asymmetric = false;
    l.error_max = {0.1, 0.2, 0.4}; l.force_max = {10, 20, 39}; l.saturation_ratios = {0.9};
    fic::PlannerParams p; p.Plannerstiffness = 15000; p.vel_desired = 1.0; p.acc_max = 5000;
    p.damping_ratio = 0.5; p.frequency = 20;
    fic::AdmittanceParams a; a.InvM = 1.0; a.damping = 54.0; a.dFlow = {-5, 5}; a.flow = {-1.2, 1.2};
    a.pose = {-0.4, 0.4};
    std::vector<double> X = {0.5, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0};
    fic::CartesianAllParams cp;
    for (int k = 0; k < 3; ++k) cp.pos_axes[k] = axis(false);
    cp.rot_axis = axis(true);

    // The desktop H-PMP (full) and the same controller exporting tau_des only (what the task-space node
    // sends the inner loop).
    FC full{4.0, false, 2.0, 23, true}, exp{4.0, false, 2.0, 23, true};
    fic::core::JointCore<double> core;
    core.deadband_joint_tau = joint_db;
    core.task_db_force = task_f;
    core.task_db_torque = task_t;
    for (FC* c : {&full, &exp}) {
        c->setJointLimits(std::vector<double>(N, -2.5), std::vector<double>(N, 2.5));
        c->setJointContinuous(std::vector<bool>(N, false));
        c->setDeadbands(joint_db, 4.0, 0.2);
        for (int i = 0; i < N; ++i) {
            c->initJointController(i, s, l);
            c->initJointPlanner(i, 0.0, p);
            c->initJointAdmittance(i, l, p, a);
        }
        c->initHpmpCartesian(cp, X);
        c->setHpmpSplitDrive(true, task_f, task_t);
    }
    exp.hpmp_export_only_ = true;
    for (int i = 0; i < N; ++i) {
        core.setJointLimits(i, -2.5, 2.5);
        fic::core::JointParamsPOD pod;
        if (!fic::core::toPOD(s, l, p, a, pod) || !core.initJoint(i, pod, 0.0)) {
            std::printf("FAIL: params\n");
            return 1;
        }
    }

    // A Jacobian already rounded to float, so both sides see the same numbers (FIC-link sends floats).
    Eigen::MatrixXd J(6, N);
    float jac[6 * N];
    for (int r = 0; r < 6; ++r) for (int c = 0; c < N; ++c) {
        jac[r * N + c] = (float)((r == c ? 0.6 : 0.0) + 0.05 * std::sin(1.0 + r * 1.7 + c * 0.9));
        J(r, c) = jac[r * N + c];
    }
    const Eigen::VectorXd xb = full.controller(0).get_x_b();
    const double window = FC::setpointWindow(xb(xb.size() - 1));

    double worst = 0.0, drive_max = 0.0;
    for (int k = 0; k < 3000; ++k) {
        const double t = k * 0.001;
        Eigen::VectorXd q(N), q_ik(N), g = Eigen::VectorXd::Zero(N);
        for (int i = 0; i < N; ++i) { q[i] = 0.05 * std::sin(0.7 * t + i); q_ik[i] = 0.02 * i; }
        // Pushes: an end-effector force (on, then between release and engage, then off) and a null-space gap.
        Eigen::Matrix<double, 6, 1> f = Eigen::Matrix<double, 6, 1>::Zero();
        f[0] = (k < 800) ? 6.0 : (k < 1400) ? 2.0 : (k < 2000) ? 0.5 : 0.0;
        f[4] = (k > 2200) ? 0.3 : 0.0;
        Eigen::VectorXd meas = J.transpose() * f;
        for (int i = 0; i < N; ++i) meas[i] += (k > 2500) ? 3.0 * std::sin(i + 0.5) : 0.0;

        std::vector<double> traj(N * 8 + 16, 0.0), torq(N, 0.0), w(6, 0.0), m(meas.data(), meas.data() + N);
        std::vector<double> traj2 = traj, torq2 = torq;
        Eigen::VectorXd qf = q, qi = q_ik, qe = q, qie = q_ik;
        full.cartesianHpmp(traj, torq, qf, g, X, X, J, 0, w, m, qi, N);
        exp.cartesianHpmp(traj2, torq2, qe, g, X, X, J, 0, w, m, qie, N);
        const Eigen::VectorXd& tau_des = exp.hpmp_tau_des_;

        double u[N], drive[N];
        for (int i = 0; i < N; ++i) u[i] = tau_des[i] + meas[i];
        core.splitDrive(jac, u, N, drive);
        for (int i = 0; i < N; ++i) drive_max = std::max(drive_max, std::abs(drive[i]));
        for (int i = 0; i < N; ++i) {
            double qt = FC::limitSetpoint(q_ik[i], q[i], window, false);
            fic::core::JointDiag d;
            const double effort = core.admittance(i, qt, q[i], tau_des[i], -meas[i], 2, true, d,
                                                  0.25 * joint_db, drive[i]);
            worst = std::max(worst, std::abs(effort - torq[i]));
        }
    }
    std::printf("hpmp joint layer, inner loop vs cartesianHpmp: worst |diff| %.3e N m (largest drive %.2f N m)\n", worst, drive_max);
    if (!(worst < 1e-9) || !(drive_max > 1.0)) { std::printf("FAIL\n"); return 1; }
    std::printf("PASS\n");
    return 0;
}
