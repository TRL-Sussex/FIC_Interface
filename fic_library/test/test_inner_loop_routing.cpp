// The routing table (fic/inner_loop_routing.h): how each node control mode reaches the motors through the
// joint inner loop. One case per row, plus idle and hard fault. Portable: no Eigen.
#include <cstdio>

#include <fic/inner_loop_routing.h>

using namespace fic::link;
static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); ++g_fail; } } while (0)
static const int N = 3;

int main() {
    double torque[N] = {11, 12, 13}, g[N] = {1, 2, 3}, q[N] = {0.1, 0.2, 0.3}, qj[N] = {0.4, 0.5, 0.6},
           qt[N] = {0.7, 0.8, 0.9}, td[N] = {21, 22, 23}, wall[N] = {0.5, 0, 0}, zone[N] = {0, 0.25, 0},
           jac[6 * N];
    for (int k = 0; k < 6 * N; ++k) jac[k] = k;
    RouteInput in;
    in.n = N; in.torque = torque; in.gravity = g; in.q_now = q; in.q_joint = qj; in.q_task = qt;
    in.hpmp_tau_des = td; in.wall = wall; in.zone = zone; in.jac = jac; in.joint_target_speed = 0.5;
    CmdMsg m;

    in.mode = 2; route(in, m);                                   // joint impedance with planner
    CHECK(m.mode == InnerMode::ImpedancePlanner && m.q_target[0] == 0.4f);
    CHECK(m.tau_ff[0] == 0.5f && m.tau_ff[1] == 0.25f);          // wall and zone torque both reach it
    CHECK(m.joint_target_speed == 0.5f && !(m.flags & kCmdHasJacobian));

    in.mode = 4; in.use_planner = true; m = CmdMsg(); route(in, m);
    CHECK(m.mode == InnerMode::AdmittanceHaptic && (m.flags & kCmdUsePlanner));
    CHECK(m.joint_target_speed == 0.0f);                         // only modes 1/2

    in.mode = 8; in.hpmp = false; m = CmdMsg(); route(in, m);    // Cartesian: pass-through
    CHECK(m.mode == InnerMode::Torque && m.tau_des[2] == 10.0f && m.tau_ff[0] == 0.0f);
    CHECK(!(m.flags & kCmdUsePlanner) && !(m.flags & kCmdHasJacobian));

    in.mode = 7; in.hpmp = true; m = CmdMsg(); route(in, m);     // H-PMP
    CHECK(m.mode == InnerMode::Hapfic && m.tau_des[0] == 20.0f && m.q_target[1] == 0.8f);
    CHECK((m.flags & kCmdHasJacobian) && m.jac[5] == 5.0f && m.tau_ff[0] == 11.0f);

    // H-PMP without a Jacobian: still Hapfic (its tau_des and set-point; the node's torque holds no J^T w and
    // no gravity), without the split drive: the inner loop falls back to the per-joint deadband.
    in.jac = nullptr; m = CmdMsg(); route(in, m);
    CHECK(m.mode == InnerMode::Hapfic && !(m.flags & kCmdHasJacobian));
    CHECK(m.tau_des[0] == 20.0f && m.q_target[1] == 0.8f && m.tau_ff[0] == 11.0f);
    in.jac = jac;

    in.mode = 0; in.hpmp = false; m = CmdMsg(); route(in, m);    // idle
    CHECK(m.mode == InnerMode::Torque && m.tau_des[1] == 10.0f && m.q_target[1] == 0.2f);

    in.mode = 3; in.hard_fault = true; m = CmdMsg(); route(in, m);
    CHECK(m.mode == InnerMode::Torque && m.tau_des[0] == 10.0f && m.tau_ff[0] == 0.0f);

    // Flags the node sets itself (enable, gravity slope) survive routing.
    in.hard_fault = false; in.mode = 1; m = CmdMsg(); m.flags = kCmdEnable | kCmdHasGravJac | kCmdHasJacobian;
    route(in, m);
    CHECK((m.flags & kCmdEnable) && (m.flags & kCmdHasGravJac) && !(m.flags & kCmdHasJacobian));

    // Stop on the inner-loop path: the ramp-down starts from the torque the arm last got (STATE tau_cmd,
    // gravity included), not from the node's torque (no joint law, no gravity on this path).
    {
        StateMsg st;
        st.n = N; st.state = McuState::Running;
        st.tau_cmd[0] = 5.0f; st.tau_cmd[1] = -7.0f; st.tau_cmd[2] = 0.5f;
        double out[N] = {9, 9, 9};
        CHECK(rampStartFromState(st, 0.01, N, out));
        CHECK(out[0] == 5.0 && out[1] == -7.0 && out[2] == 0.5);
        double keep[N] = {9, 9, 9};
        CHECK(!rampStartFromState(st, 0.5, N, keep) && keep[0] == 9.0);   // stale STATE: not used
        CHECK(!rampStartFromState(st, 0.01, N + 1, keep));                // other joint count
        st.state = McuState::Ready;                                       // not torquing: nothing to ramp from
        CHECK(!rampStartFromState(st, 0.01, N, keep));
        st.state = McuState::Hold;
        CHECK(rampStartFromState(st, 0.01, N, keep) && keep[1] == -7.0);
    }
    if (g_fail) { std::printf("%d check(s) FAILED\n", g_fail); return 1; }
    std::printf("PASS\n");
    return 0;
}
