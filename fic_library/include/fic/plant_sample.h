#ifndef FIC_PLANT_SAMPLE_H
#define FIC_PLANT_SAMPLE_H

// The topic plant (Newton, or any simulator that publishes joint_states): the robot's joints picked by name, with the scene's prefix,
// out of a joint_states message. All of them or nothing: a message missing any joint leaves q and tau as they
// were, so the inner loop never gets 0 rad for a joint that was not in it.

#include <cstddef>
#include <string>
#include <vector>

namespace fic {

inline bool pickJoints(const std::vector<std::string>& joints, const std::string& prefix,
                       const std::vector<std::string>& names, const std::vector<double>& position,
                       const std::vector<double>& effort, std::vector<double>& q, std::vector<double>& tau) {
    if (joints.empty()) return false;
    std::vector<double> nq(joints.size(), 0.0), nt(joints.size(), 0.0);
    for (size_t i = 0; i < joints.size(); ++i) {
        const std::string want = prefix + joints[i];
        size_t k = 0;
        while (k < names.size() && names[k] != want) ++k;
        if (k == names.size() || k >= position.size()) return false;
        nq[i] = position[k];
        if (k < effort.size()) nt[i] = effort[k];
    }
    q.swap(nq);
    tau.swap(nt);
    return true;
}

}  // namespace fic

#endif  // FIC_PLANT_SAMPLE_H
