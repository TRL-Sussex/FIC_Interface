// Golden dump of the FIC library: every control path driven with the same seeded inputs, every
// output printed exactly (hex floats). Built against two copies of fic_library/include, the
// dumps must be identical: a refactor of the library that changes any number shows up here.
//
//   g++ -std=c++17 -O2 -ffp-contract=off -I <include> -I /usr/include/eigen3
//       test_golden_dump.cpp -ltinyxml2 -o golden
//   ./golden joint; ./golden cart 0 ... ./golden cart 4; ./golden hpmp
//
// Joint limits contain zero in every case here; joints whose range leaves out zero are what
// the admittance-offset fix of 18 Oct 2026 changes, so they are not part of the golden set.
//
// -DGOLDEN_NO_RESTART turns off the restart on a stall (FIC 5.11.0), the one change to the law
// since, so the dump can be compared with a copy of the library from before it.
// -DGOLDEN_NODE_GATING switches the restart per mode each tick as the nodes do
// (setRestartAllowed), so the Cartesian admittance scenarios run their elbow and postural terms
// as on the robot.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <random>
#include <vector>

#include <fic/fic_control.h>
#include <fic/fic_joint_core.h>
#include <fic/joint_params_pod.h>

namespace {

const int N = 7;

void putv(const char* tag, const std::vector<double>& v) {
    std::printf("%s", tag);
    for (double x : v) std::printf(" %a", x);
    std::printf("\n");
}

struct Joint {
    fic::ScalarParams s; fic::ListParams l; fic::PlannerParams p; fic::AdmittanceParams a;
};

Joint jointParams(std::mt19937& rng, int variant) {
    std::uniform_real_distribution<double> U(0.0, 1.0);
    Joint j;
    j.s.stiffness = 50 + 250 * U(rng);
    if (variant == 0) {
        j.l.asymmetric = false; j.l.error_max = {0.05 + 0.3 * U(rng)}; j.l.force_max = {5 + 30 * U(rng)};
    } else if (variant == 1) {
        const double e1 = 0.05 + 0.1 * U(rng), f1 = 5 + 10 * U(rng);
        j.l.asymmetric = false; j.l.error_max = {e1, e1 + 0.1 + 0.2 * U(rng)};
        j.l.force_max = {f1, f1 + 5 + 10 * U(rng)}; j.l.saturation_ratios = {0.8};
    } else {
        j.l.asymmetric = true; j.l.n_saturations = 3; j.l.n_neg_sat = 1;
        j.l.error_max = {-0.25, 0.08, 0.3}; j.l.force_max = {-18, 8, 20}; j.l.saturation_ratios = {0.7, 0.85, 0.9};
    }
    j.p.Plannerstiffness = 15000; j.p.vel_desired = 0.2 + 0.8 * U(rng); j.p.acc_max = 5000;
    j.p.damping_ratio = 0.3 + 0.7 * U(rng); j.p.frequency = 20;
    j.a.InvM = 0.5 + 1.5 * U(rng); j.a.damping = 5 + 15 * U(rng);
    j.a.dFlow = {-5, 5}; j.a.flow = {-1, 1}; j.a.pose = {-0.1, 0.1};
    return j;
}

void setLimits(fic::ficControl<double>& c, bool continuous_j3) {
    std::vector<double> lo(N, -2.5), hi(N, 2.5);
    std::vector<bool> cont(N, false);
    if (continuous_j3) { cont[2] = true; lo[2] = -M_PI; hi[2] = M_PI; }
    c.setJointLimits(lo, hi);
    c.setJointContinuous(cont);
    c.setDeadbands(1.5, 4.0, 0.2);
#ifdef GOLDEN_NO_RESTART
    c.setLawOptions(false, 8.75, 1.5708);
#endif
}

// Joint modes 1-4 and the H-PMP joint layer (mode 5) of ficControl, and the MCU's JointCore.
void jointModes() {
    for (int mode = 1; mode <= 5; ++mode)
        for (int variant = 0; variant < 3; ++variant)
            for (unsigned seed : {1u, 2u}) {
                std::mt19937 rng(seed * 101 + variant * 7 + mode);
                std::uniform_real_distribution<double> U(-1.0, 1.0);
                fic::ficControl<double> c(4.0, true, 2.0, 20, true);
                fic::core::JointCore<double> core;
                const bool cont = seed == 2;
                setLimits(c, cont);
                core.deadband_joint_tau = 1.5;
#ifdef GOLDEN_NO_RESTART
                core.restart_on_stall = false;
#endif
                std::vector<double> q(N), qt(N);
                for (int i = 0; i < N; ++i) {
                    q[i] = 0.5 * U(rng); qt[i] = q[i];
                    core.setJointLimits(i, (cont && i == 2) ? -M_PI : -2.5, (cont && i == 2) ? M_PI : 2.5);
                    core.setJointContinuous(i, cont && i == 2);
                }
                for (int i = 0; i < N; ++i) {
                    Joint j = jointParams(rng, variant);
                    c.initJointController(i, j.s, j.l);
                    c.initJointPlanner(i, q[i], j.p);
                    c.initJointAdmittance(i, j.l, j.p, j.a);
                    fic::core::JointParamsPOD pod;
                    fic::core::toPOD(j.s, j.l, j.p, j.a, pod);
                    core.initJoint(i, pod, q[i]);
                }
                std::printf("# joint mode %d variant %d seed %u\n", mode, variant, seed);
                for (int k = 0; k < 3000; ++k) {
                    if (k % 700 == 0) for (int i = 0; i < N; ++i) qt[i] = 0.8 * U(rng);
                    std::vector<double> traj(N * 8, 0.0), torq(N, 0.0), core_out(N, 0.0);
                    for (int i = 0; i < N; ++i) {
                        const double tau_meas = 10 * std::sin(0.003 * k + i) + 3 * U(rng);
                        const double des = 8 * std::cos(0.002 * k + 2 * i);
                        double qt_i = qt[i], qc = qt[i];
                        fic::core::JointDiag d;
                        switch (mode) {
                        case 1: c.JC.impedance(i, traj, torq, qt[i], q[i], N);
                                core_out[i] = core.impedance(i, qt[i], q[i], d); break;
                        case 2: c.JC.impedanceWithPlanner(i, traj, torq, qt[i], q[i], N);
                                core_out[i] = core.impedanceWithPlanner(i, qt[i], q[i], d); break;
                        case 3: case 4:
                            c.JC.admittance(i, traj, torq, qt_i, q[i], des, -tau_meas, mode - 2, N, (seed & 1) != 0);
                            core_out[i] = core.admittance(i, qc, q[i], des, -tau_meas, mode - 2, (seed & 1) != 0, d);
                            break;
                        case 5:
                            c.handleJointAdmitanceControl(i, traj, torq, qt_i, q[i], des, tau_meas, 2, N, true,
                                                          0.25 * 1.5);
                            core_out[i] = core.admittance(i, qc, q[i], des, tau_meas, 2, true, d, 0.25 * 1.5);
                            break;
                        }
                        q[i] += 1e-4 * torq[i] + 2e-4 * U(rng);
                        if (cont && i == 2) q[i] = std::atan2(std::sin(q[i]), std::cos(q[i]));
                    }
                    if (k % 10 == 0) {
                        putv("torq", torq); putv("traj", traj); putv("core", core_out);
                        std::vector<double> off(N);
                        for (int i = 0; i < N; ++i) off[i] = c.humanOffset(i);
                        putv("offset", off);
                    }
                }
            }
}

Eigen::MatrixXd randomJacobian(std::mt19937& rng) {
    std::uniform_real_distribution<double> U(-1.0, 1.0);
    Eigen::MatrixXd J(6, N);
    for (int r = 0; r < 6; ++r) for (int col = 0; col < N; ++col) J(r, col) = 0.6 * U(rng);
    return J;
}

std::vector<double> pose(double x, double y, double z, const Eigen::Quaterniond& q) {
    return {x, y, z, q.x(), q.y(), q.z(), q.w()};
}

// Cartesian impedance, planner and admittance (modes 1 and 2), the elbow and the postural term.
void cartesianModes(int only) {
    fic::CartesianAllParams cart;
    std::vector<fic::ScalarParams> js(N); std::vector<fic::ListParams> jl(N);
    std::vector<fic::PlannerParams> jp(N); std::vector<fic::AdmittanceParams> ja(N);
    fic::ParameterLoader::loadHardcodedParams(N, js, jl, jp, ja, cart);
    for (int i = 0; i < 3; ++i) cart.pos_axes[i].admittance = {1.0, 10.0, {-5, 5}, {-1, 1}, {-0.1, 0.1}};
    cart.rot_axis.admittance = {1.0, 10.0, {-5, 5}, {-1, 1}, {-0.1, 0.1}};
    // One ratio per profile: the built-in profiles give fewer ratios than breakpoints, which the
    // library before 18 Oct 2026 read past the end of (undefined), so they are no golden case.
    for (int i = 0; i < 3; ++i) cart.pos_axes[i].saturation_ratios = {0.85};
    cart.rot_axis.saturation_ratios = {0.85};
    for (auto& l : jl) l.saturation_ratios = {0.85};
    for (int scenario = only; scenario == only; ++scenario) {
        std::mt19937 rng(500 + scenario);
        std::uniform_real_distribution<double> U(-1.0, 1.0);
        fic::ficControl<double> c(4.0, true, 2.0, 32, true);
        setLimits(c, scenario == 4);
        for (int a = 0; a < 3; ++a) { c.initCartesianController(a, cart.pos_axes[a]); c.initCartesianPlanner(a, 0.4, cart.pos_axes[a]); }
        c.initCartesianController(3, cart.rot_axis);
        for (int a = 0; a < 3; ++a) c.initCartesianAdmittance(a, cart.pos_axes[a]);
        for (int a = 3; a < 6; ++a) c.initCartesianAdmittance(a, cart.rot_axis);
        c.initElbowController(cart, 6);
        c.postural.init(N, js[0], jl[0]);
        if (scenario == 3) c.setDesiredWrench({2.0, -1.0, 6.0, 0.1, 0.0, -0.3});
        std::printf("# cartesian scenario %d\n", scenario);
        Eigen::VectorXd q = Eigen::VectorXd::Zero(N), qt = Eigen::VectorXd::Zero(N), grav(N);
        for (int i = 0; i < N; ++i) { q[i] = 0.4 * U(rng); qt[i] = 0.4 * U(rng); grav[i] = 3 * U(rng); }
        const Eigen::MatrixXd J = randomJacobian(rng), Je = randomJacobian(rng);
        const Eigen::Quaterniond q0(Eigen::AngleAxisd(0.3, Eigen::Vector3d(0.2, 0.9, 0.1).normalized()));
        for (int k = 0; k < 2500; ++k) {
            const double t = 0.001 * k;
            const Eigen::Quaterniond qc(Eigen::AngleAxisd(0.3 + 0.2 * std::sin(t), Eigen::Vector3d(0.2, 0.9, 0.1 + 0.1 * std::cos(t)).normalized()));
            // Sign of the goal quaternion flipped half the time (double cover; the law is under review).
            Eigen::Quaterniond qg = q0; if ((k / 300) % 2) qg.coeffs() *= -1.0;
            const auto X = pose(0.4 + 0.05 * std::sin(t), 0.1 * std::cos(2 * t), 0.5, qc);
            const auto Xt = pose(0.45, 0.0, 0.52 + 0.03 * std::sin(3 * t), qg);
            std::vector<double> wrench(6), meas(N);
            for (int a = 0; a < 6; ++a) wrench[a] = (a < 3 ? 12.0 : 0.6) * std::sin(0.004 * k + a) + 0.5 * U(rng);
            for (int i = 0; i < N; ++i) meas[i] = 5 * std::sin(0.002 * k + i);
            std::vector<double> traj, torq;
            Eigen::VectorXd V = Eigen::VectorXd::Zero(6);
#ifdef GOLDEN_NODE_GATING
            const int node_mode = scenario == 0 ? 5 : scenario == 1 ? 6 : (scenario == 4 && (k / 600) % 2) ? 8 : 7;
            c.setRestartAllowed(fic::ficControl<double>::restartAllowedInMode(node_mode, false));
#endif
            switch (scenario) {
            case 0: c.RC.impedance(traj, torq, q, grav, X, Xt, J, 0, wrench, meas, qt, N); break;
            case 1: c.RC.planner(traj, torq, q, grav, X, Xt, J, 0, wrench, meas, qt, N); break;
            case 2: case 3: c.RC.admittance(traj, torq, q, grav, X, Xt, J, 1, wrench, meas, qt, V, N); break;
            case 4: c.RC.admittance(traj, torq, q, grav, X, Xt, J, (k / 600) % 2 ? 2 : 1, wrench, meas, qt, V, N); break;
            }
            c.applyElbowControl(torq, q, J, Je, pose(0.2, 0.1 * std::sin(t), 0.3, qc), pose(0.25, 0.0, 0.3, qc), N, 6);
            c.postural.apply(q, qt, true, N, torq);
            if (k % 5 == 0) { putv("ctorq", torq); putv("ctraj", traj); }
            for (int i = 0; i < N; ++i) q[i] += 1e-5 * torq[i];
        }
    }
}

// H-PMP: host modes 5 (no planner), 6 (no admittance), 7 and 8 (task-space admittance), with and
// without the split drive.
void hpmpModes() {
    fic::CartesianAllParams cart;
    std::vector<fic::ScalarParams> js(N); std::vector<fic::ListParams> jl(N);
    std::vector<fic::PlannerParams> jp(N); std::vector<fic::AdmittanceParams> ja(N);
    fic::ParameterLoader::loadHardcodedParams(N, js, jl, jp, ja, cart);
    for (int i = 0; i < 3; ++i) cart.pos_axes[i].admittance = {1.0, 10.0, {-5, 5}, {-1, 1}, {-0.1, 0.1}};
    cart.rot_axis.admittance = {1.0, 10.0, {-5, 5}, {-1, 1}, {-0.2, 0.2}};
    for (int i = 0; i < 3; ++i) cart.pos_axes[i].saturation_ratios = {0.85};
    cart.rot_axis.saturation_ratios = {0.85};
    const int modes[] = {fic::ficControl<double>::kHpmpHostImpedance, 0, 1, 2};
    for (int split = 0; split < 2; ++split)
        for (int m : modes) {
            std::mt19937 rng(900 + 10 * split + m);
            std::uniform_real_distribution<double> U(-1.0, 1.0);
            fic::ficControl<double> c(4.0, true, 2.0, 32, true);
            setLimits(c, m == 1);
            c.setHpmpSplitDrive(split == 1, 1.0, 0.1);
            Eigen::VectorXd q(N), qt(N), grav(N);
            for (int i = 0; i < N; ++i) { q[i] = 0.4 * U(rng); qt[i] = q[i] + 0.05 * U(rng); grav[i] = 3 * U(rng); }
            for (int i = 0; i < N; ++i) {
                Joint j = jointParams(rng, i % 3);
                c.initJointController(i, j.s, j.l);
                c.initJointPlanner(i, q[i], j.p);
                c.initJointAdmittance(i, j.l, j.p, j.a);
            }
            c.initHpmpCartesian(cart, {0.4, 0.0, 0.5});
            std::printf("# hpmp mode %d split %d\n", m, split);
            const Eigen::MatrixXd J = randomJacobian(rng);
            for (int k = 0; k < 2500; ++k) {
                const double t = 0.001 * k;
                const Eigen::Quaterniond qc(Eigen::AngleAxisd(0.2 + 0.1 * std::sin(t), Eigen::Vector3d::UnitZ()));
                const Eigen::Quaterniond qg(Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ()));
                const auto X = pose(0.4 + 0.02 * std::sin(t), 0.0, 0.5, qc);
                const auto Xt = pose(0.42, 0.01, 0.5, qg);
                std::vector<double> wrench(6), meas(N);
                for (int a = 0; a < 6; ++a) wrench[a] = (a < 3 ? 8.0 : 0.5) * std::sin(0.005 * k + a);
                for (int i = 0; i < N; ++i) meas[i] = -grav[i] + 4 * std::sin(0.003 * k + i) + 0.3 * U(rng);
                std::vector<double> traj, torq;
                c.RC.hpmp(traj, torq, q, grav, X, Xt, J, m, wrench, meas, qt, N);
                if (k % 5 == 0) { putv("htorq", torq); putv("htraj", traj); }
                for (int i = 0; i < N; ++i) q[i] += 1e-5 * torq[i];
            }
        }
}

} // namespace

// One part per run (a fresh process each), so state a library keeps beyond an instance (the
// Cartesian admittance's, before 18 Oct 2026) cannot carry from one scenario into the next:
//   golden joint | golden cart N (N = 0..4) | golden hpmp
int main(int argc, char** argv) {
    const std::string part = argc > 1 ? argv[1] : "joint";
    if (part == "joint") jointModes();
    else if (part == "cart") cartesianModes(argc > 2 ? std::atoi(argv[2]) : 0);
    else if (part == "hpmp") hpmpModes();
    else return 2;
    return 0;
}
