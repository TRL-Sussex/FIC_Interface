// The topic plant's sample (Newton): the robot's joints picked by name out of a
// joint_states message. A message missing any of them changes nothing: the last complete sample stands.

#include <cstdio>
#include <string>
#include <vector>

#include <fic/plant_sample.h>

static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { std::printf("  FAIL line %d: %s\n", __LINE__, #c); ++g_fail; } } while (0)

int main() {
    const std::vector<std::string> robot = {"j1", "j2", "j3"};
    std::vector<double> q, tau;

    // Complete, in another order, with the scene's prefix and an extra joint.
    CHECK(fic::pickJoints(robot, "r0_", {"r0_j3", "gripper", "r0_j1", "r0_j2"}, {3, 9, 1, 2}, {30, 90, 10, 20},
                          q, tau));
    CHECK(q == (std::vector<double>{1, 2, 3}) && tau == (std::vector<double>{10, 20, 30}));

    // j2 missing: nothing changes (no 0 rad written for it).
    CHECK(!fic::pickJoints(robot, "r0_", {"r0_j1", "r0_j3"}, {1.5, 3.5}, {15, 35}, q, tau));
    CHECK(q == (std::vector<double>{1, 2, 3}) && tau == (std::vector<double>{10, 20, 30}));

    // No efforts in the message: positions taken, torques 0.
    CHECK(fic::pickJoints(robot, "", {"j1", "j2", "j3"}, {4, 5, 6}, {}, q, tau));
    CHECK(q == (std::vector<double>{4, 5, 6}) && tau == (std::vector<double>{0, 0, 0}));

    std::printf(g_fail ? "%d check(s) FAILED\n" : "plant sample OK\n", g_fail);
    return g_fail ? 1 : 0;
}
