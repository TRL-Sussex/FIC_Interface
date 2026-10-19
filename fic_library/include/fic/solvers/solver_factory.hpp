// solver_factory.hpp — creates the kinematics solver selected per robot via
// the `kinematics_solver:` key in fic_library/config/robots_config.yaml.
//
// To plug in your own solver (see docs/custom-kinematics-solver.md):
//   1. Implement fic::KinematicsSolver in a header, e.g.
//      fic_library/include/fic/solvers/my_solver.hpp
//   2. Add its #include and a name branch at the two USER SOLVERS markers
//      below.
//   3. Rebuild: colcon build --packages-select fic_control fic_control
//   4. Set `kinematics_solver: my_solver` for your robot in robots_config.yaml.
#pragma once

#include <memory>
#include <string>

#include <fic/kinematics_solver.hpp>

#ifdef FIC_WITH_PINOCCHIO
#include <fic/solvers/pinocchio_solver.hpp>
#include <fic/solvers/closed_loop_pinocchio_solver.hpp>
#endif
// ── USER SOLVERS: include your solver header here ───────────────────────────
// #include <fic/solvers/my_solver.hpp>

namespace fic {

// Returns the solver registered under `name`, or nullptr:
//   * name "" or "none"  → nullptr with no error (deliberate degraded mode:
//     joint-space control only, Cartesian features disabled)
//   * unknown/unavailable name → nullptr, reason written to *error
inline std::unique_ptr<KinematicsSolver>
createKinematicsSolver(const std::string& name, std::string* error = nullptr) {
    if (error) error->clear();
    if (name.empty() || name == "none") return nullptr;

    if (name == "pinocchio") {
#ifdef FIC_WITH_PINOCCHIO
        return std::make_unique<PinocchioSolver>();
#else
        if (error)
            *error = "solver 'pinocchio' requested but this build has "
                     "FIC_WITH_PINOCCHIO=OFF (pinocchio was not found at build time)";
        return nullptr;
#endif
    }

    if (name == "pinocchio_closed_loop") {
#ifdef FIC_WITH_PINOCCHIO
        // Parallel / closed-loop mechanisms (e.g. the CSPM spherical parallel
        // manipulator). Presents an actuated-joint interface; the FIC node must
        // call configureClosedLoop() with the loop closures + EE from config.
        return std::make_unique<ClosedLoopPinocchioSolver>();
#else
        if (error)
            *error = "solver 'pinocchio_closed_loop' requested but this build has "
                     "FIC_WITH_PINOCCHIO=OFF (pinocchio was not found at build time)";
        return nullptr;
#endif
    }

    // ── USER SOLVERS: register your solver name here, then rebuild ──────────
    // if (name == "my_solver") return std::make_unique<MySolver>();

    if (error) *error = "unknown kinematics_solver '" + name + "'";
    return nullptr;
}

}  // namespace fic
