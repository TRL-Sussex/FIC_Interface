// kinematics_solver.hpp — abstract kinematics/dynamics solver interface.
//
// This is the seam that makes the rigid-body solver pluggable: the FIC nodes
// (fic_control, fic_control) talk only to this interface, and concrete
// solvers (the built-in Pinocchio one, or a user-supplied one) implement it.
// See docs/custom-kinematics-solver.md for a full walkthrough of writing and
// registering your own solver.
//
// Implementations must be Eigen-only from the caller's point of view: this
// header pulls in no ROS and no pinocchio, so custom solvers are compilable
// and unit-testable outside a ROS workspace.
//
// ── Threading contract ──────────────────────────────────────────────────────
// The interface splits into two groups:
//
//  * CACHED path — updateKinematics(), cachedFramePose(), frameJacobian(),
//    gravity(). Called from the single control-loop thread (1 kHz), typically
//    under the node's data mutex. Implementations may keep persistent scratch
//    state (pinocchio: the Data object) refreshed by updateKinematics().
//    NOT required to be thread-safe.
//
//  * SCRATCH path — framePose(), solveIK(). const, and MUST be safe to call
//    concurrently with the cached path and with each other: the simulation
//    node runs solveIK() in a detached worker thread while the control loop
//    spins. Implementations must use local state (pinocchio: a local Data)
//    and must not disturb the cached state above.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace fic {

class KinematicsSolver {
public:
    using Matrix6x = Eigen::Matrix<double, 6, Eigen::Dynamic>;

    // ── Closed-loop / parallel-mechanism configuration (optional) ───────────
    // One kinematic loop closure: two frames (each a fixed point `pos` on a
    // link `body`) that the mechanism holds coincident. E.g. a leg-tip site and
    // the platform-vertex it connects to on a spherical parallel manipulator.
    struct LoopClosure {
        std::string bodyA, bodyB;
        Eigen::Vector3d posA{Eigen::Vector3d::Zero()};
        Eigen::Vector3d posB{Eigen::Vector3d::Zero()};
    };
    // Configure a closed-loop solver: the loop closures, the ACTUATED joints
    // (the reduced coordinates the controller drives), the controlled end-effector
    // frame, and whether it is orientation-only (e.g. a fixed-CoR spherical
    // mechanism). Called once after loadModel(). Default is a NO-OP: serial
    // solvers ignore it, so existing robots are completely unaffected.
    virtual void configureClosedLoop(const std::vector<LoopClosure>& /*loops*/,
                                     const std::vector<std::string>& /*actuated_joints*/,
                                     const std::string& /*ee_frame*/,
                                     bool /*orientation_only*/) {}

    // Reference frame convention for Jacobians (matches pinocchio's naming):
    //  Local             — twist expressed in the frame itself
    //  LocalWorldAligned — twist at the frame origin, axes aligned with world
    enum class JacobianFrame { Local, LocalWorldAligned };

    struct IKOptions {
        double tolerance      = 1e-4;   // convergence threshold on ||log6(error)||
        int    max_iterations = 1000;
        double dt             = 1e-2;   // integration step per iteration
        double damping        = 1e-6;   // damped-least-squares regulariser
        // true  → error/Jacobian anchored at the target frame's parent JOINT
        //         (fic_control flavor). false → anchored at the FRAME itself
        //         in the Local convention (fic_control flavor).
        bool   joint_based        = false;
        // > 0 → clamp ||v|| per iteration to this value (sim uses 0.5).
        double max_joint_velocity = 0.0;
    };

    struct IKResult {
        Eigen::VectorXd q;              // best iterate, returned even on failure
        bool   success    = false;
        double error_norm = 0.0;
        int    iterations = 0;
    };

    virtual ~KinematicsSolver() = default;

    // Short identifier, e.g. "pinocchio" — used in logs.
    virtual std::string name() const = 0;

    // ── Model lifecycle ─────────────────────────────────────────────────────
    // Load a robot model from a URDF file. Returns false on failure (must not
    // throw); the failure reason should be retrievable via lastError().
    virtual bool loadModel(const std::string& urdf_path) = 0;
    virtual bool isLoaded() const = 0;
    // Human-readable reason for the last loadModel() failure ("" if none).
    virtual std::string lastError() const { return {}; }

    virtual int nq() const = 0;   // configuration-vector size
    virtual int nv() const = 0;   // velocity / actuated-DOF size

    // ── Joint metadata ──────────────────────────────────────────────────────
    // Position limits for velocity index i in [0, nv).
    // Position limits of the i-th ACTUATED joint, in control order -- NOT the
    // i-th entry of the configuration vector. The two coincide only when every
    // joint contributes one entry to q; a `continuous` joint contributes two
    // (cos, sin), so nq > nv and every bounded joint after the first continuous
    // one shifts. ClosedLoopPinocchioSolver has always mapped actuated->config
    // internally; PinocchioSolver did not, and silently returned a neighbouring
    // joint's limit (or a cos/sin bound of +-1 read as radians) for any URDF
    // using `continuous`. Callers must have called configureActuatedJoints().
    virtual double lowerPositionLimit(int i) const = 0;
    virtual double upperPositionLimit(int i) const = 0;

    // Declare which joints are actuated, in control order, so the solver can
    // map an actuated index onto the configuration vector. Default no-op: a
    // solver that already knows its actuated set (closed-loop) ignores it.
    virtual void configureActuatedJoints(const std::vector<std::string>& /*joint_names*/) {}
    // Index of a named joint into the q configuration vector; -1 if unknown.
    virtual int jointConfigIndex(const std::string& joint_name) const = 0;
    virtual bool hasFrame(const std::string& frame_name) const = 0;

    // ── Cached kinematics (control-loop path; see threading contract) ───────
    // Run forward kinematics + frame placements on the persistent state.
    virtual void updateKinematics(const Eigen::VectorXd& q) = 0;
    // Pose of a named frame after the last updateKinematics(). Returns false
    // if the frame does not exist.
    virtual bool cachedFramePose(const std::string& frame_name,
                                 Eigen::Isometry3d& out) const = 0;
    // 6×nv Jacobian of a named frame at q (may refresh persistent state).
    // Returns false if the frame does not exist.
    virtual bool frameJacobian(const Eigen::VectorXd& q,
                               const std::string& frame_name,
                               JacobianFrame ref, Matrix6x& out) = 0;

    // ── Dynamics ────────────────────────────────────────────────────────────
    // Generalized gravity torques at q (size nv).
    virtual Eigen::VectorXd gravity(const Eigen::VectorXd& q) = 0;

    // ── Scratch one-offs (thread-safe path; see threading contract) ─────────
    // Forward kinematics of a named frame at q using local scratch state.
    virtual bool framePose(const Eigen::VectorXd& q,
                           const std::string& frame_name,
                           Eigen::Isometry3d& out) const = 0;
    // Iterative inverse kinematics from q_start to a target pose of the named
    // frame. Returns the best iterate even when convergence fails.
    virtual IKResult solveIK(const Eigen::VectorXd& q_start,
                             const Eigen::Isometry3d& target,
                             const std::string& frame_name,
                             const IKOptions& opts) const = 0;
};

}  // namespace fic
