#ifndef FIC_JOINT_PARAMS_POD_H
#define FIC_JOINT_PARAMS_POD_H

// Linux-side converter: ParameterLoader structs (std::vector based, loaded from
// config.xml) -> fic::core::JointParamsPOD (fixed-size, shippable to the MCU).

#include <algorithm>
#include <vector>

#include "fic_joint_core.h"
#include "fic_param_loader.hpp"

namespace fic {
namespace core {

// Copies up to kMaxSat values; returns false if the source had to be truncated.
inline bool copyList(const std::vector<double>& src, double* dst, int32_t& n) {
    n = static_cast<int32_t>(std::min<size_t>(src.size(), kMaxSat));
    std::copy(src.begin(), src.begin() + n, dst);
    return src.size() <= static_cast<size_t>(kMaxSat);
}

inline bool toPOD(const fic::ScalarParams& s, const fic::ListParams& l,
                  const fic::PlannerParams& p, const fic::AdmittanceParams& a,
                  JointParamsPOD& out) {
    bool ok = true;
    out = JointParamsPOD{};
    out.stiffness = s.stiffness;

    out.n_saturations = l.n_saturations;
    out.n_neg_sat = l.n_neg_sat;
    out.asymmetric = l.asymmetric ? 1 : 0;
    ok &= copyList(l.error_max, out.error_max, out.n_error_max);
    ok &= copyList(l.force_max, out.force_max, out.n_force_max);
    ok &= copyList(l.saturation_ratios, out.saturation_ratios, out.n_saturation_ratios);

    out.planner_stiffness = p.Plannerstiffness;
    out.vel_desired = p.vel_desired;
    out.acc_max = p.acc_max;
    out.damping_ratio = p.damping_ratio;
    out.frequency = p.frequency;

    out.inv_m = a.InvM;
    out.adm_damping = a.damping;
    // fic_control.h reads dFlow[0..1] / flow[0..1] unchecked; missing entries
    // are a config error there too.
    if (a.dFlow.size() < 2 || a.flow.size() < 2) return false;
    out.dflow[0] = a.dFlow[0]; out.dflow[1] = a.dFlow[1];
    out.flow[0] = a.flow[0];   out.flow[1] = a.flow[1];
    return ok;
}

} // namespace core
} // namespace fic

#endif // FIC_JOINT_PARAMS_POD_H
