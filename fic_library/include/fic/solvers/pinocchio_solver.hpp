// pinocchio_solver.hpp — built-in KinematicsSolver backed by Pinocchio.
//
// The whole implementation is guarded by FIC_WITH_PINOCCHIO (defined by the
// consuming package's CMake when pinocchio is found), so fic_library remains
// compilable on systems without pinocchio even if this header is included.
//
// The math here is a verbatim port of what previously lived inline in
// fic_hardware.cpp / fic_simulation.cpp — behavior with pinocchio present is
// intentionally identical.
#pragma once

#ifdef FIC_WITH_PINOCCHIO

#include <string>

#include <fic/kinematics_solver.hpp>

#include "pinocchio/parsers/urdf.hpp"
// pinocchio 4 ships the umbrella header pinocchio/multibody.hpp for Model and
// Data; 2.x/3.x (Ubuntu apt / robotpkg) only ship the split headers.
#if __has_include("pinocchio/multibody/model.hpp")
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#else
#include "pinocchio/multibody.hpp"
#endif
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/rnea.hpp"

namespace fic {

class PinocchioSolver final : public KinematicsSolver {
public:
    std::string name() const override { return "pinocchio"; }

    // Configuration index of the i-th actuated joint; identity when the
    // actuated set has not been declared.
    int configIndex(int i) const {
        if (i >= 0 && i < static_cast<int>(act_qidx_.size())) return act_qidx_[i];
        return i;
    }

    bool loadModel(const std::string& urdf_path) override {
        try {
            model_ = pinocchio::Model();
            act_qidx_.clear();          // stale mapping must not outlive the model
            pinocchio::urdf::buildModel(urdf_path, model_);
            data_ = pinocchio::Data(model_);
            loaded_ = true;
            last_error_.clear();
        } catch (const std::exception& e) {
            loaded_ = false;
            last_error_ = e.what();
        }
        return loaded_;
    }

    bool isLoaded() const override { return loaded_; }
    std::string lastError() const override { return last_error_; }

    int nq() const override { return model_.nq; }
    int nv() const override { return model_.nv; }

    // Actuated index -> configuration index. Identity until configured, which
    // preserves the previous behaviour for callers that never declare their
    // actuated set. See the contract on KinematicsSolver::lowerPositionLimit.
    void configureActuatedJoints(const std::vector<std::string>& joint_names) override {
        act_qidx_.clear();
        for (const auto& name : joint_names) {
            const int qi = jointConfigIndex(name);
            if (qi < 0) { act_qidx_.clear(); return; }   // unknown joint -> stay unconfigured
            act_qidx_.push_back(qi);
        }
    }

    double lowerPositionLimit(int i) const override {
        return model_.lowerPositionLimit(configIndex(i));
    }
    double upperPositionLimit(int i) const override {
        return model_.upperPositionLimit(configIndex(i));
    }

    int jointConfigIndex(const std::string& joint_name) const override {
        const auto joint_id = model_.getJointId(joint_name);
        if (joint_id == 0 || joint_id >= model_.joints.size()) return -1;
        return model_.joints[joint_id].idx_q();
    }

    bool hasFrame(const std::string& frame_name) const override {
        // pinocchio returns frames.size() (not -1) for a missing frame.
        return model_.getFrameId(frame_name) < model_.frames.size();
    }

    // ── Cached path (control loop) ──────────────────────────────────────────

    void updateKinematics(const Eigen::VectorXd& q) override {
        pinocchio::forwardKinematics(model_, data_, q);
        pinocchio::updateFramePlacements(model_, data_);
    }

    bool cachedFramePose(const std::string& frame_name,
                         Eigen::Isometry3d& out) const override {
        const auto id = model_.getFrameId(frame_name);
        if (id >= model_.frames.size()) return false;
        out = toIsometry(data_.oMf[id]);
        return true;
    }

    bool frameJacobian(const Eigen::VectorXd& q, const std::string& frame_name,
                       JacobianFrame ref, Matrix6x& out) override {
        const auto id = model_.getFrameId(frame_name);
        if (id >= model_.frames.size()) return false;
        out.setZero(6, model_.nv);
        pinocchio::computeFrameJacobian(
            model_, data_, q, id,
            ref == JacobianFrame::LocalWorldAligned ? pinocchio::LOCAL_WORLD_ALIGNED
                                                    : pinocchio::LOCAL,
            out);
        return true;
    }

    Eigen::VectorXd gravity(const Eigen::VectorXd& q) override {
        return pinocchio::computeGeneralizedGravity(model_, data_, q);
    }

    // ── Scratch path (thread-safe one-offs) ─────────────────────────────────

    bool framePose(const Eigen::VectorXd& q, const std::string& frame_name,
                   Eigen::Isometry3d& out) const override {
        const auto id = model_.getFrameId(frame_name);
        if (id >= model_.frames.size()) return false;
        pinocchio::Data local(model_);
        pinocchio::forwardKinematics(model_, local, q);
        pinocchio::updateFramePlacements(model_, local);
        out = toIsometry(local.oMf[id]);
        return true;
    }

    IKResult solveIK(const Eigen::VectorXd& q_start, const Eigen::Isometry3d& target,
                     const std::string& frame_name, const IKOptions& opts) const override {
        IKResult result;
        result.q = q_start;
        if (!loaded_) return result;

        const auto frame_id = model_.getFrameId(frame_name);
        if (frame_id >= model_.frames.size()) return result;

        pinocchio::Data local_data(model_);
        Eigen::VectorXd q = q_start;
        const pinocchio::SE3 oMdes(target.rotation(), target.translation());

        pinocchio::Data::Matrix6x J(6, model_.nv);
        J.setZero();
        Eigen::Matrix<double, 6, 1> err;
        err.setZero();
        Eigen::VectorXd v(model_.nv);

        int i = 0;
        if (opts.joint_based) {
            // fic_hardware flavor: error/Jacobian anchored at the frame's
            // parent joint (oMi + computeJointJacobian).
            const auto JOINT_ID = model_.frames[frame_id].parentJoint;
            // The target is for the FRAME, but this branch drives the
            // frame's parent JOINT. They are not the same place: on the
            // Kinova Gen3, ee_link sits 6.15 cm and a full 180 deg (URDF
            // end_effector joint, rpy="3.14159 0 0") from Actuator7.
            // Solving the joint straight onto the frame's target baked
            // that offset into every answer -- replayed on real targets,
            // a command 1.5 deg from the current pose came back with the
            // wrist turned 179 deg and ee_link 93 mm / 151 deg off target,
            // which postural control then pulled the arm toward.
            //
            // For the frame to land on oMdes, the joint must sit at
            // oMdes * jMf^-1, jMf being the frame's fixed placement in its
            // parent joint. A frame that coincides with its joint has
            // jMf = identity, so robots without an offset are unchanged.
            const pinocchio::SE3 oMdes_joint =
                oMdes * model_.frames[frame_id].placement.inverse();
            for (; i < opts.max_iterations; ++i) {
                pinocchio::forwardKinematics(model_, local_data, q);
                const pinocchio::SE3 iMd = local_data.oMi[JOINT_ID].actInv(oMdes_joint);
                err = pinocchio::log6(iMd).toVector();

                if (err.norm() < opts.tolerance) { result.success = true; break; }

                pinocchio::computeJointJacobian(model_, local_data, q, JOINT_ID, J);
                pinocchio::Data::Matrix6 Jlog;
                pinocchio::Jlog6(iMd.inverse(), Jlog);
                J = -Jlog * J;

                pinocchio::Data::Matrix6 JJt;
                JJt.noalias() = J * J.transpose();
                JJt.diagonal().array() += opts.damping;
                v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
                q = pinocchio::integrate(model_, q, v * opts.dt);
            }
        } else {
            // fic_simulation flavor: error/Jacobian at the frame itself in the
            // Local convention, with a per-iteration joint-velocity clamp.
            for (; i < opts.max_iterations; ++i) {
                pinocchio::forwardKinematics(model_, local_data, q);
                pinocchio::updateFramePlacements(model_, local_data);

                const pinocchio::SE3& oMcurr = local_data.oMf[frame_id];
                const pinocchio::SE3 iMd = oMcurr.actInv(oMdes);
                err = pinocchio::log6(iMd).toVector();

                if (err.norm() < opts.tolerance) { result.success = true; break; }

                pinocchio::computeFrameJacobian(model_, local_data, q, frame_id,
                                                pinocchio::LOCAL, J);
                pinocchio::Data::Matrix6 Jlog;
                pinocchio::Jlog6(iMd, Jlog);
                J = Jlog * J;

                pinocchio::Data::Matrix6 JJt;
                JJt.noalias() = J * J.transpose();
                JJt.diagonal().array() += opts.damping;

                v.noalias() = JJt.ldlt().solve(err);
                v = J.transpose() * v;

                const double v_norm = v.norm();
                if (opts.max_joint_velocity > 0.0 && v_norm > opts.max_joint_velocity)
                    v *= (opts.max_joint_velocity / v_norm);
                q = pinocchio::integrate(model_, q, v * opts.dt);
            }
        }

        result.q = q;
        result.error_norm = err.norm();
        result.iterations = i;
        return result;
    }

private:
    static Eigen::Isometry3d toIsometry(const pinocchio::SE3& m) {
        Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
        out.linear() = m.rotation();
        out.translation() = m.translation();
        return out;
    }

    std::vector<int> act_qidx_;   // actuated index -> configuration index

    pinocchio::Model model_;
    pinocchio::Data data_{model_};
    bool loaded_ = false;
    std::string last_error_;
};

}  // namespace fic

#endif  // FIC_WITH_PINOCCHIO
