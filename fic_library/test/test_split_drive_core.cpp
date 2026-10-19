// JointCore::splitDrive (portable, runs on the MCU) against fic_control.h's cartesianHpmp split
// (Eigen, fic_control.h:1338-1360), on the same J and u, with its hysteresis state.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>

#include <Eigen/Dense>

#include <fic/fic_joint_core.h>

static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); ++g_fail; } } while (0)
static const int N = 7;

// fic_control.h:1338-1360, the same maths
struct Ref {
    double db_f = 4.0, db_t = 0.2, db_j = 2.0, rel = 0.25;
    bool on_f = false, on_t = false;
    bool on_j[N] = {};
    Eigen::VectorXd drive(const Eigen::MatrixXd& J, const Eigen::VectorXd& u) {
        const Eigen::MatrixXd JJt = J * J.transpose() + 0.01 * 0.01 * Eigen::MatrixXd::Identity(6, 6);
        const Eigen::Matrix<double, 6, 1> w = JJt.ldlt().solve(J * u);
        const Eigen::VectorXd un = u - J.transpose() * w;
        const double fn = w.head<3>().norm(), tn = w.tail<3>().norm();
        if (!on_f && fn >= db_f) on_f = true; else if (on_f && fn < rel * db_f) on_f = false;
        if (!on_t && tn >= db_t) on_t = true; else if (on_t && tn < rel * db_t) on_t = false;
        Eigen::Matrix<double, 6, 1> wd = Eigen::Matrix<double, 6, 1>::Zero();
        if (on_f) wd.head<3>() = w.head<3>();
        if (on_t) wd.tail<3>() = w.tail<3>();
        Eigen::VectorXd d = J.transpose() * wd;
        for (int i = 0; i < N; ++i) {
            if (!on_j[i] && std::abs(un[i]) >= db_j) on_j[i] = true;
            else if (on_j[i] && std::abs(un[i]) < rel * db_j) on_j[i] = false;
            if (on_j[i]) d[i] += un[i];
        }
        return d;
    }
};

int main() {
    std::mt19937 rng(42);
    std::normal_distribution<double> G(0.0, 1.0);
    Ref ref;
    fic::core::JointCore<double> core;
    core.deadband_joint_tau = 2.0;
    core.task_db_force = 4.0;
    core.task_db_torque = 0.2;
    Eigen::MatrixXd J(6, N);
    Eigen::VectorXd u = Eigen::VectorXd::Zero(N);
    float jac[6 * N];
    double worst = 0;
    int task_on = 0, joint_on = 0;
    for (int k = 0; k < 5000; ++k) {
        if (k % 250 == 0)
            for (int r = 0; r < 6; ++r) for (int c = 0; c < N; ++c) J(r, c) = 0.4 * G(rng);
        if (k == 2000) J.setZero();              // solver not ready
        if (k == 3000) J.row(5) = J.row(4);      // rank-deficient
        for (int r = 0; r < 6; ++r) for (int c = 0; c < N; ++c) jac[r * N + c] = (float)J(r, c);
        Eigen::MatrixXd Jf(6, N);                // the reference sees the float J too
        for (int r = 0; r < 6; ++r) for (int c = 0; c < N; ++c) Jf(r, c) = jac[r * N + c];
        for (int i = 0; i < N; ++i) u[i] = 0.98 * (u[i] + 0.6 * G(rng));
        const Eigen::VectorXd want = ref.drive(Jf, u);
        task_on += ref.on_f || ref.on_t;
        for (int i = 0; i < N; ++i) joint_on += ref.on_j[i];
        double got[N];
        core.splitDrive(jac, u.data(), N, got);
        for (int i = 0; i < N; ++i) {
            CHECK(std::isfinite(got[i]));
            worst = std::max(worst, std::abs(got[i] - want[i]));
        }
    }
    std::printf("split drive: worst |diff| %.3e N m (task engaged %d ticks, joint %d joint-ticks)\n",
                worst, task_on, joint_on);
    CHECK(worst < 1e-9);
    CHECK(task_on > 100 && task_on < 4900);      // both deadbands switched both ways
    CHECK(joint_on > 100);
    if (g_fail) { std::printf("%d check(s) FAILED\n", g_fail); return 1; }
    std::printf("PASS\n");
    return 0;
}
