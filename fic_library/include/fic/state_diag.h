#ifndef FIC_STATE_DIAG_H
#define FIC_STATE_DIAG_H

// The joint diagnostics of trajectory_data (8 blocks per joint, fic_control.h's joint handlers:
// 0 planner set-point, 1 position, 2 error, 3 target, 4 effort, 5 admittance target, 6 desired effort,
// 7 measured effort) filled from the inner loop's STATE: the joint law runs there, so the task node shows
// what the inner loop reports. Blocks beyond 8n (the Cartesian modes' tail) are kept.

#include <cstddef>
#include <vector>

#include "fic_link.h"

namespace fic {

inline void fillJointDiag(const fic::link::StateMsg& s, const double* target, int n, std::vector<double>& traj) {
    if (n <= 0 || s.n < n) return;
    if (traj.size() < static_cast<size_t>(8 * n)) traj.resize(8 * n, 0.0);
    auto at = [&](int k, int i) -> double& { return traj[static_cast<size_t>(i + k * n)]; };
    for (int i = 0; i < n; ++i) {
        const double q = s.q[i], t = target ? target[i] : q;
        at(0, i) = t;                                   // the inner loop's planner set-point is not reported
        at(1, i) = q;
        at(2, i) = t - q;
        at(3, i) = t;
        at(4, i) = static_cast<double>(s.tau_law[i]) - s.g_used[i];   // the law's effort, without gravity
        at(5, i) = s.q_target_adm[i];
        at(6, i) = s.g_used[i];                         // the joint admittance's desired effort: gravity
        at(7, i) = -static_cast<double>(s.tau_meas[i]); // measured, in the joint admittance's sign
    }
}

}  // namespace fic

#endif  // FIC_STATE_DIAG_H
