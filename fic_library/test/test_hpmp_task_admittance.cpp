// H-PMP on host modes 7/8 (fic_control.h cartesianHpmp): the host's task-space admittance
// shifts the task layer's goal, on H-PMP's own integrators, and the IK set-point follows it to
// first order (hpmp_q_shift_ = J# dx). On host mode 6 nothing changes: the wrench is unused and
// the shift is zero.
//
// Checks: (1) mode 6 ignores the wrench (outputs identical with and without it, zero IK shift);
// (2) mode 2 (M2) shifts the goal exactly as cartesianAdmittanceControl does for the same wrench
// and parameters, within +-5 cm; (3) the IK shift equals J^T (J J^T + lambda^2 I)^-1 dx; (4) mode 1
// (M1) keeps the shift after the push ends; (5) a wrench inside the deadband shifts nothing;
// (6) switching back to mode 6 clears the shift. Desired wrench (setDesiredWrench): (7) zero at
// construction, and a short or non-finite wrench is refused; (8) a desired wrench equal to the measured
// one shifts nothing; (9) a desired wrench w_d with no measured wrench drives the H-PMP admittance
// exactly as a measured wrench -w_d, and (10) the host admittance (mode 8) likewise. (11) Host mode 5
// (kHpmpHostImpedance): the task layer acts on the goal itself, no planner, while on host mode 6
// the planner starts from where the arm is.
//
// Needs Eigen and tinyxml2 (fic_control.h pulls in the XML loader).

#include <cmath>
#include <cstdio>
#include <vector>

#include <fic/fic_control.h>

using FC = fic::ficControl<double>;
static const int N = 7;
static int failures = 0;

static void check(bool ok, const char* what, double value) {
    std::printf("  %-66s %s (%.6g)\n", what, ok ? "ok" : "FAIL", value);
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
    explicit Rig(bool hpmp) {
        std::vector<double> lower(N, -2.5), upper(N, 2.5);
        c.setJointLimits(lower, upper);
        c.setJointContinuous(std::vector<bool>(N, false));
        c.setDeadbands(2.0, 4.0, 0.2);
        fic::ScalarParams s; s.stiffness = 200;
        fic::ListParams l; l.asymmetric = false;
        l.error_max = {0.1, 0.2, 0.4}; l.force_max = {10, 20, 39}; l.saturation_ratios = {0.9};
        fic::PlannerParams p; p.Plannerstiffness = 15000; p.vel_desired = 1.0; p.acc_max = 5000;
        p.damping_ratio = 0.5; p.frequency = 20;
        fic::AdmittanceParams a; a.InvM = 1.0; a.damping = 54.0; a.dFlow = {-5, 5}; a.flow = {-1.2, 1.2};
        a.pose = {-0.4, 0.4};
        for (int k = 0; k < 3; ++k) cp.pos_axes[k] = axis(false);
        cp.rot_axis = axis(true);
        // A fixed, well-conditioned 6x7 Jacobian.
        for (int r = 0; r < 6; ++r) for (int col = 0; col < N; ++col)
            J(r, col) = (r == col ? 0.6 : 0.0) + 0.05 * std::sin(1.0 + r * 1.7 + col * 0.9);
        if (hpmp) {
            for (int i = 0; i < N; ++i) {
                c.initJointController(i, s, l);
                c.initJointPlanner(i, 0.0, p);
                c.initJointAdmittance(i, l, p, a);
            }
            c.initHpmpCartesian(cp, X);
        } else {   // the host Cartesian admittance (modes 7/8), as fic_simulation initCartesianControllers
            for (int i = 0; i < 3; ++i) {
                c.initCartesianController(i, cp.pos_axes[i]);
                c.initCartesianPlanner(i, X[i], cp.pos_axes[i]);
                c.initCartesianAdmittance(i, cp.pos_axes[i]);
            }
            for (int i = 3; i < 6; ++i) {
                c.initCartesianController(i, cp.rot_axis);
                c.initCartesianAdmittance(i, cp.rot_axis);
            }
        }
    }
    // One tick of H-PMP; returns the task goal (traj base+0..2) minus the commanded goal, and the
    // planned position (base+9..11) minus the goal in *planned.
    Eigen::Vector3d hpmp(int mode, const std::vector<double>& wrench, std::vector<double>* torq_out = nullptr,
                           Eigen::Vector3d* planned = nullptr) {
        std::vector<double> traj(N * 8 + 16, 0.0), torq(N, 0.0), meas(N, 0.0);
        c.cartesianHpmp(traj, torq, q, g, X, X, J, mode, wrench, meas, q_ik, N);
        if (torq_out) *torq_out = torq;
        const size_t b = N * 8;
        if (planned) *planned = Eigen::Vector3d(traj[b + 9] - X[0], traj[b + 10] - X[1], traj[b + 11] - X[2]);
        return Eigen::Vector3d(traj[b + 0] - X[0], traj[b + 1] - X[1], traj[b + 2] - X[2]);
    }
    // One tick of the host admittance mode; returns its planned position (traj base+9..11, the
    // planner output towards its admittance target) minus the goal.
    Eigen::Vector3d host(int mode, const std::vector<double>& wrench) {
        std::vector<double> traj(N * 8 + 16, 0.0), torq(N, 0.0), meas(N, 0.0);
        Eigen::VectorXd V = Eigen::VectorXd::Zero(6);
        c.cartesianAdmittanceControl(traj, torq, q, g, X, X, J, mode, wrench, meas, q_ik, V, N);
        const size_t b = N * 8;
        return Eigen::Vector3d(traj[b + 9] - X[0], traj[b + 10] - X[1], traj[b + 11] - X[2]);
    }
};

int main() {
    const std::vector<double> push = {10.0, 0.0, -6.0, 0.0, 0.0, 0.0};   // x and z above the 4 N deadband
    const std::vector<double> none(6, 0.0);

    std::printf("(1) host mode 6: the wrench is unused\n");
    {
        Rig a(true), b(true);
        double diff = 0.0;
        for (int k = 0; k < 500; ++k) {
            std::vector<double> ta, tb;
            a.hpmp(0, push, &ta);
            b.hpmp(0, none, &tb);
            for (int i = 0; i < N; ++i) diff = std::fmax(diff, std::fabs(ta[i] - tb[i]));
        }
        check(diff == 0.0, "torque identical with and without a wrench", diff);
        check(a.c.hpmp_q_shift_.norm() == 0.0, "IK shift zero", a.c.hpmp_q_shift_.norm());
    }

    std::printf("(2)-(3) host mode 8 (M2): goal shifted as cartesianAdmittanceControl, IK shift = J# dx\n");
    {
        Rig h(true), m8(false);
        double max_diff = 0.0;
        Eigen::Vector3d sh, ph, pm;
        for (int k = 0; k < 2000; ++k) {
            sh = h.hpmp(2, push, nullptr, &ph);
            pm = m8.host(2, push);
            max_diff = std::fmax(max_diff, (ph - pm).norm());
        }
        check(max_diff < 1e-12, "same planned path as the host admittance (same goal), every tick", max_diff);
        check((ph - sh).norm() < 1e-3, "planner has reached the shifted goal", (ph - sh).norm());
        check(sh.norm() > 0.04 && std::fabs(sh.x()) <= 0.05 + 1e-12 && std::fabs(sh.z()) <= 0.05 + 1e-12,
              "shift reached and within the +-5 cm bound", sh.norm());
        check(std::fabs(sh.y()) < 1e-12, "no shift on the unpushed axis", sh.y());
        Eigen::Matrix<double,6,1> dx = Eigen::Matrix<double,6,1>::Zero();
        dx.head<3>() = sh;
        const Eigen::MatrixXd JJt = h.J * h.J.transpose() + 1e-4 * Eigen::MatrixXd::Identity(6, 6);
        const Eigen::VectorXd expect = h.J.transpose() * JJt.ldlt().solve(dx);
        check((h.c.hpmp_q_shift_ - expect).norm() < 1e-12, "IK shift = J^T (J J^T + 1e-4 I)^-1 dx",
              (h.c.hpmp_q_shift_ - expect).norm());
        check((h.J * h.c.hpmp_q_shift_ - dx).norm() < 1e-3, "J q_shift reproduces dx", (h.J * h.c.hpmp_q_shift_ - dx).norm());
    }

    std::printf("(4) host mode 7 (M1): the shift stays after the push\n");
    {
        Rig h(true);
        Eigen::Vector3d during, settled, later;
        for (int k = 0; k < 500; ++k) during = h.hpmp(1, push);
        for (int k = 0; k < 500; ++k) settled = h.hpmp(1, none);   // coasts ~1/(InvM Kv) = 50 ms
        for (int k = 0; k < 1000; ++k) later = h.hpmp(1, none);
        check(during.norm() > 0.05, "pushed further than M2's bound", during.norm());
        check((later - settled).norm() < 1e-5 && later.norm() <= 0.30 * std::sqrt(2.0) + 1e-9,
              "stays where pushed (within the +-30 cm bound)", (later - settled).norm());
    }

    std::printf("(5) a wrench inside the deadband shifts nothing\n");
    {
        Rig h(true);
        Eigen::Vector3d s;
        for (int k = 0; k < 1000; ++k) s = h.hpmp(2, {3.0, -3.5, 2.0, 0.1, -0.15, 0.0});
        check(s.norm() == 0.0 && h.c.hpmp_q_shift_.norm() == 0.0, "goal and IK set-point unchanged", s.norm());
    }

    std::printf("(6) back to host mode 6: the shift is cleared\n");
    {
        Rig h(true);
        for (int k = 0; k < 500; ++k) h.hpmp(2, push);
        const Eigen::Vector3d s = h.hpmp(0, push);
        check(s.norm() == 0.0 && h.c.hpmp_q_shift_.norm() == 0.0, "goal and IK set-point back to the commanded ones", s.norm());
    }

    std::printf("(7) desired wrench: zero by default, bad input refused\n");
    {
        Rig h(true);
        double n0 = 0.0;
        for (double v : h.c.desiredWrench()) n0 += std::fabs(v);
        check(h.c.desiredWrench().size() == 6 && n0 == 0.0, "zero at construction", n0);
        const std::vector<double> wd = {5.0, 0.0, -8.0, 0.0, 0.3, 0.0};
        check(h.c.setDesiredWrench(wd), "a 6-value wrench is accepted", 0.0);
        check(!h.c.setDesiredWrench({1.0, 2.0, 3.0}), "a 3-value wrench is refused", 0.0);
        check(!h.c.setDesiredWrench({1.0, NAN, 0, 0, 0, 0}), "a non-finite wrench is refused", 0.0);
        check(h.c.desiredWrench() == wd, "the previous wrench is kept", 0.0);
    }

    std::printf("(8) desired wrench equal to the measured one: no shift\n");
    {
        Rig h(true), m8(false);
        h.c.setDesiredWrench(push);
        m8.c.setDesiredWrench(push);
        Eigen::Vector3d sh, pm;
        for (int k = 0; k < 1000; ++k) { sh = h.hpmp(2, push); pm = m8.host(2, push); }
        check(sh.norm() == 0.0 && h.c.hpmp_q_shift_.norm() == 0.0, "H-PMP goal and IK set-point unchanged", sh.norm());
        check(pm.norm() < 1e-12, "host mode 8 plans to the commanded goal", pm.norm());
    }

    std::printf("(9)-(10) desired wrench alone = measured wrench of the opposite sign\n");
    {
        std::vector<double> neg(6);
        for (int i = 0; i < 6; ++i) neg[i] = -push[i];
        Rig hd(true), hm(true), md(false), mm(false);
        hd.c.setDesiredWrench(neg);
        md.c.setDesiredWrench(neg);
        double dh = 0.0, dm = 0.0;
        Eigen::Vector3d sd;
        for (int k = 0; k < 2000; ++k) {
            Eigen::Vector3d pd, pmeas;
            sd = hd.hpmp(2, none, nullptr, &pd);
            hm.hpmp(2, push, nullptr, &pmeas);
            dh = std::fmax(dh, (pd - pmeas).norm());
            dm = std::fmax(dm, (md.host(2, none) - mm.host(2, push)).norm());
        }
        check(dh < 1e-12, "H-PMP: same planned path, every tick", dh);
        check(sd.norm() > 0.04, "H-PMP: the goal is shifted", sd.norm());
        check(dm < 1e-12, "host mode 8: same planned path, every tick", dm);
    }

    std::printf("(11) host mode 5: no planner in the task layer\n");
    {
        Rig r5(true), r6(true);
        std::vector<double> goal = r5.X;
        goal[0] += 0.05;                                   // 5 cm ahead in x
        auto planned = [&](Rig& r, int mode) {
            std::vector<double> traj(N * 8 + 16, 0.0), torq(N, 0.0), meas(N, 0.0);
            r.c.cartesianHpmp(traj, torq, r.q, r.g, r.X, goal, r.J, mode, none, meas, r.q_ik, N);
            const size_t b = N * 8;
            return Eigen::Vector3d(traj[b + 9], traj[b + 10], traj[b + 11]);
        };
        const Eigen::Vector3d p5 = planned(r5, FC::kHpmpHostImpedance);
        const Eigen::Vector3d p6 = planned(r6, 0);
        check(std::fabs(p5.x() - goal[0]) < 1e-12, "host mode 5: planned position is the goal", p5.x() - goal[0]);
        check(std::fabs(p6.x() - r6.X[0]) < 0.01, "host mode 6: the planner starts at the arm", p6.x() - r6.X[0]);
    }

    std::printf(failures ? "FAILED (%d)\n" : "passed\n", failures);
    return failures ? 1 : 0;
}
