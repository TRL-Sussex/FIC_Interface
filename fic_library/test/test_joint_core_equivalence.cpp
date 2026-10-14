// Equivalence: fic_joint_core.h (portable, runs on the MCU) vs the original
// joint path in fic_control.h / FIC.cpp (runs in fic_hardware today).
//
// Same parameters, same feedback, same targets, thousands of ticks, every
// joint mode — the torques and diagnostics must be bit-identical. If this
// fails, the MCU no longer runs the controller the lab has tuned.
//
// Needs Eigen and tinyxml2 (fic_control.h pulls in the XML loader).

#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include <fic/fic_control.h>
#include <fic/fic_joint_core.h>
#include <fic/joint_params_pod.h>

static const int N = 7;

struct JointSet {
    std::vector<fic::ScalarParams> s;
    std::vector<fic::ListParams> l;
    std::vector<fic::PlannerParams> p;
    std::vector<fic::AdmittanceParams> a;
};

static JointSet randomParams(std::mt19937& rng, int variant) {
    std::uniform_real_distribution<double> U(0.0, 1.0);
    JointSet js;
    js.s.resize(N); js.l.resize(N); js.p.resize(N); js.a.resize(N);
    for (int i = 0; i < N; ++i) {
        js.s[i].stiffness = 50 + 250 * U(rng);
        auto& l = js.l[i];
        if (variant == 0) {                      // symmetric, one saturation
            l.asymmetric = false;
            l.error_max = {0.05 + 0.3 * U(rng)};
            l.force_max = {5 + 30 * U(rng)};
            l.saturation_ratios = {};
        } else if (variant == 1) {               // symmetric, two saturations
            l.asymmetric = false;
            const double e1 = 0.05 + 0.1 * U(rng);
            l.error_max = {e1, e1 + 0.1 + 0.2 * U(rng)};
            const double f1 = 5 + 10 * U(rng);
            l.force_max = {f1, f1 + 5 + 10 * U(rng)};
            l.saturation_ratios = {0.8};
        } else {                                 // asymmetric: 1 negative, 2 positive
            // FIC.cpp treats any even split (N_neg*2 == N_S) as symmetric
            // regardless of the flag, so a true asymmetric set has an odd split.
            l.asymmetric = true;
            l.n_saturations = 3;
            l.n_neg_sat = 1;
            l.error_max = {-0.25, 0.08, 0.3};
            l.force_max = {-18, 8, 20};
            l.saturation_ratios = {0.7, 0.85, 0.9};
        }
        js.p[i].Plannerstiffness = 15000;
        js.p[i].vel_desired = 0.2 + 0.8 * U(rng);
        js.p[i].acc_max = 5000;
        js.p[i].damping_ratio = 0.3 + 0.7 * U(rng);
        js.p[i].frequency = 20;
        js.a[i].InvM = 0.5 + 1.5 * U(rng);
        js.a[i].damping = 5 + 15 * U(rng);
        js.a[i].dFlow = {-5, 5};
        js.a[i].flow = {-1, 1};
        js.a[i].pose = {-0.1, 0.1};
    }
    return js;
}

// mode 1..4 as fic_hardware; 5 = the hapfic joint layer (admittance mode 2,
// desired effort from the Cartesian law, measured torque with + sign).
static int runCase(int mode, int variant, unsigned seed, bool continuous_j3) {
    std::mt19937 rng(seed);
    JointSet js = randomParams(rng, variant);
    std::uniform_real_distribution<double> U(-1.0, 1.0);

    fic::ficControl<double> ref(4.0, true, 2.0, 20, true);
    fic::core::JointCore<double> core;
    std::vector<double> lower(N), upper(N);
    std::vector<bool> cont(N, false);
    for (int i = 0; i < N; ++i) { lower[i] = -2.5; upper[i] = 2.5; }
    if (continuous_j3) { cont[2] = true; lower[2] = -M_PI; upper[2] = M_PI; }
    ref.setJointLimits(lower, upper);
    ref.setJointContinuous(cont);
    ref.setDeadbands(1.5, 4.0, 0.2);
    core.deadband_joint_tau = 1.5;
    core.dt = fic::dt;
    for (int i = 0; i < N; ++i) {
        core.setJointLimits(i, lower[i], upper[i]);
        core.setJointContinuous(i, cont[i]);
    }

    std::vector<double> q(N), qt(N);
    for (int i = 0; i < N; ++i) { q[i] = 0.5 * U(rng); qt[i] = q[i]; }
    if (continuous_j3) q[2] = 3.1;              // start near the wrap

    for (int i = 0; i < N; ++i) {
        ref.initJointController(i, js.s[i], js.l[i]);
        ref.initJointPlanner(i, q[i], js.p[i]);
        ref.initJointAdmittance(i, js.l[i], js.p[i], js.a[i]);
        fic::core::JointParamsPOD pod;
        if (!fic::core::toPOD(js.s[i], js.l[i], js.p[i], js.a[i], pod)) {
            std::printf("  toPOD failed\n");
            return 1;
        }
        if (!core.initJoint(i, pod, q[i])) {
            std::printf("  initJoint rejected params the original accepted\n");
            return 1;
        }
    }

    int mismatches = 0;
    double worst = 0;
    const bool use_planner = (seed & 1) != 0;
    for (int k = 0; k < 6000; ++k) {
        if (k % 700 == 0) for (int i = 0; i < N; ++i) qt[i] = 0.8 * U(rng);   // target steps
        std::vector<double> traj(N * 8, 0.0), torq(N, 0.0);
        for (int i = 0; i < N; ++i) {
            // A rough, noisy "measurement" so every branch gets exercised.
            const double tau_meas = 10 * std::sin(0.003 * k + i) + 3 * U(rng);
            const double des = 8 * std::cos(0.002 * k + 2 * i);
            fic::core::JointDiag d;
            double t_core = 0, qt_i = qt[i];
            switch (mode) {
            case 1:
                ref.JC.impedance(i, traj, torq, qt[i], q[i], N);
                t_core = core.impedance(i, qt[i], q[i], d);
                break;
            case 2:
                ref.JC.impedanceWithPlanner(i, traj, torq, qt[i], q[i], N);
                t_core = core.impedanceWithPlanner(i, qt[i], q[i], d);
                break;
            case 3: case 4: {
                const int m = mode == 3 ? 1 : 2;
                ref.JC.admittance(i, traj, torq, qt_i, q[i], des, -tau_meas, m, N, use_planner);
                double qc = qt[i];
                t_core = core.admittance(i, qc, q[i], des, -tau_meas, m, use_planner, d);
                break;
            }
            case 5: {
                ref.JC.admittance(i, traj, torq, qt_i, q[i], des, tau_meas, 2, N, true);
                double qc = qt[i];
                t_core = core.admittance(i, qc, q[i], des, tau_meas, 2, true, d);
                break;
            }
            }
            const double pairs[][2] = {
                {torq[i], t_core},
                {traj[i + 1 * N], d.q_current}, {traj[i + 2 * N], d.error},
                {traj[i + 3 * N], d.q_target}, {traj[i + 4 * N], d.effort},
                {mode >= 2 ? traj[i] : 0.0, mode >= 2 ? d.planner_target : 0.0},
                {mode >= 3 ? traj[i + 5 * N] : 0.0, mode >= 3 ? d.q_target_adm : 0.0},
                {mode >= 3 ? ref.humanOffset(i) : 0.0, mode >= 3 ? core.humanOffset(i) : 0.0},
            };
            for (const auto& pr : pairs) {
                const bool both_nan = std::isnan(pr[0]) && std::isnan(pr[1]);
                if (pr[0] != pr[1] && !both_nan) {
                    ++mismatches;
                    worst = std::fmax(worst, std::isfinite(pr[0] - pr[1]) ? std::fabs(pr[0] - pr[1]) : INFINITY);
                }
            }
            // Move the joint toward the torque a little (crude plant).
            q[i] += 1e-4 * torq[i] + 2e-4 * U(rng);
            if (cont[i]) q[i] = std::atan2(std::sin(q[i]), std::cos(q[i]));
        }
    }
    std::printf("  mode %d variant %d seed %u%s: %s (worst %.3g)\n", mode, variant, seed,
                continuous_j3 ? " +continuous" : "", mismatches ? "MISMATCH" : "identical", worst);
    return mismatches ? 1 : 0;
}

// Params the original rejects (throws) must be rejected by the port too.
static int rejectsWhatOriginalRejects() {
    fic::ScalarParams sp; sp.stiffness = 100;
    fic::ListParams lp;
    lp.asymmetric = true; lp.n_saturations = 4; lp.n_neg_sat = 2;   // even split
    lp.error_max = {-0.4, -0.1, 0.08, 0.3};
    lp.force_max = {-25, -10, 8, 20};
    fic::PlannerParams pp; pp.vel_desired = 0.5; pp.acc_max = 5000; pp.damping_ratio = 0.7;
    fic::AdmittanceParams ap; ap.dFlow = {-5, 5}; ap.flow = {-1, 1};
    bool ref_threw = false;
    try {
        fic::ficControl<double> ref(4.0, true, 2.0, 20, true);
        ref.initJointController(0, sp, lp);
    } catch (const std::exception&) { ref_threw = true; }
    fic::core::JointCore<double> core;
    fic::core::JointParamsPOD pod;
    fic::core::toPOD(sp, lp, pp, ap, pod);
    const bool core_rejected = !core.initJoint(0, pod, 0.0);
    std::printf("  even-split asymmetric params: original %s, port %s\n",
                ref_threw ? "throws" : "accepts", core_rejected ? "rejects" : "accepts");
    return (ref_threw == core_rejected) ? 0 : 1;
}

int main() {
    int fails = rejectsWhatOriginalRejects();
    for (int mode = 1; mode <= 5; ++mode)
        for (int variant = 0; variant < 3; ++variant)
            for (unsigned seed : {1u, 2u})
                fails += runCase(mode, variant, seed + 10 * mode + 100 * variant, seed == 2);
    if (fails) { std::printf("%d case(s) differ\n", fails); return 1; }
    std::printf("joint core is bit-identical to fic_control.h\n");
    return 0;
}
