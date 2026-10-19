// The joint diagnostics (trajectory_data's 8 blocks per joint) filled from the inner loop's STATE: the joint
// law runs there, so the task node shows what it reports (fic/state_diag.h).
#include <cmath>
#include <cstdio>
#include <vector>

#include <fic/state_diag.h>

using namespace fic::link;
static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); ++g_fail; } } while (0)

int main() {
    const int N = 3;
    StateMsg s;
    s.n = N;
    for (int i = 0; i < N; ++i) {
        s.q[i] = 0.1f * (i + 1); s.tau_cmd[i] = 10.0f + i; s.g_used[i] = 4.0f; s.tau_meas[i] = -2.0f;
        s.q_target_adm[i] = 0.5f; s.tau_law[i] = 9.0f + i;
    }
    const double target[N] = {0.4, 0.5, 0.6};
    std::vector<double> traj(5 * N, 0.0);
    fic::fillJointDiag(s, target, N, traj);
    CHECK(traj.size() == 8u * N);
    auto blk = [&](int k, int i) { return traj[i + k * N]; };
    CHECK(blk(0, 1) == 0.5 && blk(3, 1) == 0.5);                    // set-point, target
    CHECK(std::fabs(blk(1, 0) - 0.1) < 1e-6);                       // position
    CHECK(std::fabs(blk(2, 2) - (0.6 - 0.3)) < 1e-6);               // error
    CHECK(blk(4, 0) == 9.0 - 4.0);                                  // the law's effort, without gravity
    CHECK(blk(5, 2) == 0.5);                                        // admittance target
    CHECK(blk(6, 0) == 4.0 && blk(7, 0) == 2.0);                    // desired (gravity), measured (-tau_meas)
    std::vector<double> big(8 * N + 16, 7.0);                       // Cartesian layout: tail kept
    fic::fillJointDiag(s, target, N, big);
    CHECK(big.size() == 8u * N + 16 && big[8 * N + 3] == 7.0);
    if (g_fail) { std::printf("%d check(s) FAILED\n", g_fail); return 1; }
    std::printf("PASS\n");
    return 0;
}
