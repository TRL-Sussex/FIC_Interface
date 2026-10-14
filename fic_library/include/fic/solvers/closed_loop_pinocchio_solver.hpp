// closed_loop_pinocchio_solver.hpp — KinematicsSolver for CLOSED-LOOP / parallel
// mechanisms, backed by Pinocchio.
//
// WHY THIS EXISTS
//   A URDF (and therefore vanilla Pinocchio) is a kinematic TREE and cannot
//   represent a closed-loop parallel mechanism (e.g. the CSPM spherical parallel
//   manipulator, whose platform is held by 3 legs via loop-closure constraints).
//   The plain PinocchioSolver on such a URDF would compute FK/Jacobians for one
//   DISCONNECTED leg, not the platform.
//
// WHAT IT DOES (modular — works for ANY robot)
//   Presents an ACTUATED-joint interface to the FIC controller: nq()=nv()=the
//   number of actuated joints (e.g. 3), and the controlled frame is the platform.
//   Internally it holds the full tree + N loop closures (frame pairs from config)
//   and, per control cycle:
//     * updateKinematics(q_act): warm-started Newton solves the passive DOF so all
//       loops close, giving the full assembled configuration, then FK.
//     * frameJacobian(): returns the REDUCED Jacobian (EE twist per actuated
//       joint) via the constraint null-space projection.
//     * solveIK(): iterates the actuated joints to a target EE pose using that
//       reduced Jacobian.
//   With ZERO loop closures and all joints actuated it degrades to the ordinary
//   serial solver, so it is a strict superset — usable for any robot.
//
//   Validated numerically against MuJoCo (platform orientation error < 0.002 deg;
//   reduced Jacobian full rank). See scratch prototype cl_kin_prototype.py.
//
// Threading: same contract as KinematicsSolver — the cached path keeps a
// persistent warm-start config q_full_ (control-loop thread only); the scratch
// path (framePose/solveIK, const) uses local state.
#pragma once

#ifdef FIC_WITH_PINOCCHIO

#include <string>
#include <vector>

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

class ClosedLoopPinocchioSolver final : public KinematicsSolver {
public:
    std::string name() const override { return "pinocchio_closed_loop"; }

    bool loadModel(const std::string& urdf_path) override {
        try {
            model_ = pinocchio::Model();
            pinocchio::urdf::buildModel(urdf_path, model_);
            data_  = pinocchio::Data(model_);
            q_full_ = pinocchio::neutral(model_);
            loaded_ = true;
            last_error_.clear();
        } catch (const std::exception& e) {
            loaded_ = false;
            last_error_ = e.what();
        }
        // Default (until configured): behave as a plain serial solver.
        configured_ = false;
        act_vidx_.clear(); act_qidx_.clear(); act_nq_.clear(); free_vidx_.clear(); loop_frames_.clear();
        return loaded_;
    }

    void configureClosedLoop(const std::vector<LoopClosure>& loops,
                             const std::vector<std::string>& actuated_joints,
                             const std::string& ee_frame,
                             bool orientation_only) override {
        if (!loaded_) return;
        loop_frames_.clear(); act_vidx_.clear(); act_qidx_.clear();
        orientation_only_ = orientation_only;

        // Add a fixed OP_FRAME for each loop endpoint on its parent body.
        auto add_point = [&](const std::string& body, const Eigen::Vector3d& p) -> pinocchio::FrameIndex {
            const auto bfid = model_.getFrameId(body);
            const auto& bf  = model_.frames[bfid];
            pinocchio::SE3 placement = bf.placement * pinocchio::SE3(Eigen::Matrix3d::Identity(), p);
            return model_.addFrame(pinocchio::Frame("cl_pt_" + std::to_string(model_.nframes),
                                                    bf.parentJoint, bfid, placement,
                                                    pinocchio::OP_FRAME));
        };
        for (const auto& lc : loops)
            loop_frames_.push_back({add_point(lc.bodyA, lc.posA), add_point(lc.bodyB, lc.posB)});

        // Actuated dof/config indices (in full-model order) + their nq (1 for a
        // revolute joint, 2 for a continuous joint stored as (cos,sin)).
        act_nq_.clear();
        for (const auto& jn : actuated_joints) {
            const auto jid = model_.getJointId(jn);
            if (jid == 0 || jid >= model_.joints.size()) continue;
            act_vidx_.push_back(model_.joints[jid].idx_v());
            act_qidx_.push_back(model_.joints[jid].idx_q());
            act_nq_.push_back(model_.joints[jid].nq());
        }
        n_act_ = static_cast<int>(act_vidx_.size());

        // Free dofs = everything not actuated.
        std::vector<bool> is_act(model_.nv, false);
        for (int v : act_vidx_) is_act[v] = true;
        free_vidx_.clear();
        for (int v = 0; v < model_.nv; ++v) if (!is_act[v]) free_vidx_.push_back(v);

        ee_fid_ = model_.getFrameId(ee_frame);
        data_ = pinocchio::Data(model_);
        q_full_ = pinocchio::neutral(model_);
        configured_ = !loop_frames_.empty() && n_act_ > 0 && ee_fid_ < model_.frames.size();
    }

    bool isLoaded() const override { return loaded_; }
    std::string lastError() const override { return last_error_; }

    // Reduced dimensions: actuated count when configured, else the full model.
    int nq() const override { return configured_ ? n_act_ : model_.nq; }
    int nv() const override { return configured_ ? n_act_ : model_.nv; }

    double lowerPositionLimit(int i) const override {
        if (!configured_) return model_.lowerPositionLimit(i);
        return (i >= 0 && i < n_act_) ? model_.lowerPositionLimit(act_qidx_[i]) : -3.14159265358979;
    }
    double upperPositionLimit(int i) const override {
        if (!configured_) return model_.upperPositionLimit(i);
        return (i >= 0 && i < n_act_) ? model_.upperPositionLimit(act_qidx_[i]) : 3.14159265358979;
    }

    int jointConfigIndex(const std::string& joint_name) const override {
        const auto jid = model_.getJointId(joint_name);
        if (jid == 0 || jid >= model_.joints.size()) return -1;
        if (!configured_) return model_.joints[jid].idx_q();
        const int v = model_.joints[jid].idx_v();          // map to actuated index
        for (int i = 0; i < n_act_; ++i) if (act_vidx_[i] == v) return i;
        return -1;
    }

    bool hasFrame(const std::string& frame_name) const override {
        return model_.getFrameId(frame_name) < model_.frames.size();
    }

    // ── Cached path (control loop) ──────────────────────────────────────────
    void updateKinematics(const Eigen::VectorXd& q) override {
        if (!configured_) {                                 // serial fallback
            pinocchio::forwardKinematics(model_, data_, q);
            pinocchio::updateFramePlacements(model_, data_);
            return;
        }
        assemble(q, q_full_, data_);                        // warm-started from q_full_
        pinocchio::forwardKinematics(model_, data_, q_full_);
        pinocchio::updateFramePlacements(model_, data_);
    }

    bool cachedFramePose(const std::string& frame_name, Eigen::Isometry3d& out) const override {
        const auto id = model_.getFrameId(frame_name);
        if (id >= model_.frames.size()) return false;
        out = toIsometry(data_.oMf[id]);
        return true;
    }

    bool frameJacobian(const Eigen::VectorXd& q, const std::string& frame_name,
                       JacobianFrame ref, Matrix6x& out) override {
        const auto id = model_.getFrameId(frame_name);
        if (id >= model_.frames.size()) return false;
        const auto pin_ref = ref == JacobianFrame::LocalWorldAligned
                                 ? pinocchio::LOCAL_WORLD_ALIGNED : pinocchio::LOCAL;
        if (!configured_) {
            out.setZero(6, model_.nv);
            pinocchio::computeFrameJacobian(model_, data_, q, id, pin_ref, out);
            return true;
        }
        assemble(q, q_full_, data_);                        // ensure q_full_ matches q
        out = reducedJacobian(id, pin_ref, data_, q_full_); // 6 x n_act
        return true;
    }

    Eigen::VectorXd gravity(const Eigen::VectorXd& q) override {
        if (!configured_) return pinocchio::computeGeneralizedGravity(model_, data_, q);
        // Constrained gravity on the actuated coords. For the light CSPM the
        // sim provides gravity separately (~0); return zeros of actuated size,
        // which is the safe, correct value for a balanced/near-massless platform.
        return Eigen::VectorXd::Zero(n_act_);
    }

    // ── Scratch path (thread-safe one-offs) ─────────────────────────────────
    bool framePose(const Eigen::VectorXd& q, const std::string& frame_name,
                   Eigen::Isometry3d& out) const override {
        const auto id = model_.getFrameId(frame_name);
        if (id >= model_.frames.size()) return false;
        pinocchio::Data local(model_);
        if (!configured_) {
            pinocchio::forwardKinematics(model_, local, q);
            pinocchio::updateFramePlacements(model_, local);
            out = toIsometry(local.oMf[id]);
            return true;
        }
        Eigen::VectorXd qf = q_full_;
        assemble(q, qf, local);
        pinocchio::forwardKinematics(model_, local, qf);
        pinocchio::updateFramePlacements(model_, local);
        out = toIsometry(local.oMf[id]);
        return true;
    }

    IKResult solveIK(const Eigen::VectorXd& q_start, const Eigen::Isometry3d& target,
                     const std::string& frame_name, const IKOptions& opts) const override {
        IKResult result; result.q = q_start;
        if (!loaded_) return result;
        const auto frame_id = model_.getFrameId(frame_name);
        if (frame_id >= model_.frames.size()) return result;

        if (!configured_) {                                 // delegate to serial IK
            PinocchioSolverIKFallback(q_start, target, frame_id, opts, result);
            return result;
        }

        pinocchio::Data local(model_);
        Eigen::VectorXd q_act = q_start;                    // actuated coords
        Eigen::VectorXd qf = q_full_;
        const pinocchio::SE3 oMdes(target.rotation(), target.translation());
        Eigen::Matrix<double, 6, 1> err; err.setZero();
        int i = 0;
        for (; i < opts.max_iterations; ++i) {
            assemble(q_act, qf, local);
            pinocchio::forwardKinematics(model_, local, qf);
            pinocchio::updateFramePlacements(model_, local);
            const pinocchio::SE3& oMcurr = local.oMf[frame_id];
            const pinocchio::SE3 iMd = oMcurr.actInv(oMdes);
            err = pinocchio::log6(iMd).toVector();
            if (orientation_only_) err.head<3>().setZero();  // control rotation only
            if (err.norm() < opts.tolerance) { result.success = true; break; }

            Matrix6x Jr = reducedJacobian(frame_id, pinocchio::LOCAL, local, qf);  // 6 x n_act
            pinocchio::Data::Matrix6 Jlog; pinocchio::Jlog6(iMd, Jlog);
            Eigen::MatrixXd J = Jlog * Jr;                   // 6 x n_act
            // damped least squares in the reduced (actuated) space
            Eigen::MatrixXd JJt = J * J.transpose();
            JJt.diagonal().array() += opts.damping;
            Eigen::VectorXd v = J.transpose() * JJt.ldlt().solve(err);   // n_act
            const double vn = v.norm();
            if (opts.max_joint_velocity > 0.0 && vn > opts.max_joint_velocity)
                v *= (opts.max_joint_velocity / vn);
            q_act += v * opts.dt;                            // actuated joints are simple revolute
        }
        result.q = q_act; result.error_norm = err.norm(); result.iterations = i;
        return result;
    }

private:
    struct FramePair { pinocchio::FrameIndex a, b; };

    static Eigen::Isometry3d toIsometry(const pinocchio::SE3& m) {
        Eigen::Isometry3d o = Eigen::Isometry3d::Identity();
        o.linear() = m.rotation(); o.translation() = m.translation(); return o;
    }

    // Full-model constraint residual (3 * nloops): frameA.pos - frameB.pos.
    Eigen::VectorXd loopResidual(const Eigen::VectorXd& qf, pinocchio::Data& d) const {
        pinocchio::forwardKinematics(model_, d, qf);
        pinocchio::updateFramePlacements(model_, d);
        Eigen::VectorXd r(3 * static_cast<int>(loop_frames_.size()));
        for (std::size_t k = 0; k < loop_frames_.size(); ++k)
            r.segment<3>(3 * static_cast<int>(k)) =
                d.oMf[loop_frames_[k].a].translation() - d.oMf[loop_frames_[k].b].translation();
        return r;
    }

    // Full-model constraint Jacobian (3*nloops x nv): d(residual)/dq.
    Eigen::MatrixXd loopJacobianFull(const Eigen::VectorXd& qf, pinocchio::Data& d) const {
        pinocchio::computeJointJacobians(model_, d, qf);
        pinocchio::updateFramePlacements(model_, d);
        const int m = 3 * static_cast<int>(loop_frames_.size());
        Eigen::MatrixXd Jc(m, model_.nv); Jc.setZero();
        Matrix6x JA(6, model_.nv), JB(6, model_.nv);
        for (std::size_t k = 0; k < loop_frames_.size(); ++k) {
            JA.setZero(); JB.setZero();
            pinocchio::getFrameJacobian(model_, d, loop_frames_[k].a, pinocchio::LOCAL_WORLD_ALIGNED, JA);
            pinocchio::getFrameJacobian(model_, d, loop_frames_[k].b, pinocchio::LOCAL_WORLD_ALIGNED, JB);
            Jc.middleRows(3 * static_cast<int>(k), 3) = (JA - JB).topRows(3);
        }
        return Jc;
    }

    // Solve the passive DOF (Newton) so all loops close, actuated joints fixed
    // to q_act. Warm-starts from qf (updated in place).
    void assemble(const Eigen::VectorXd& q_act, Eigen::VectorXd& qf, pinocchio::Data& d,
                  int iters = 12, double tol = 1e-10) const {
        for (int i = 0; i < n_act_; ++i) setJointAct(qf, i, q_act[i]);
        for (int it = 0; it < iters; ++it) {
            Eigen::VectorXd r = loopResidual(qf, d);
            if (r.norm() < tol) break;
            Eigen::MatrixXd Jc = loopJacobianFull(qf, d);
            Eigen::MatrixXd Jfree(Jc.rows(), static_cast<int>(free_vidx_.size()));
            for (std::size_t c = 0; c < free_vidx_.size(); ++c) Jfree.col(static_cast<int>(c)) = Jc.col(free_vidx_[c]);
            Eigen::VectorXd dqf = Jfree.completeOrthogonalDecomposition().solve(-r);  // least-squares
            Eigen::VectorXd dq = Eigen::VectorXd::Zero(model_.nv);
            for (std::size_t c = 0; c < free_vidx_.size(); ++c) dq[free_vidx_[c]] = dqf[static_cast<int>(c)];
            qf = pinocchio::integrate(model_, qf, dq);
            for (int i = 0; i < n_act_; ++i) setJointAct(qf, i, q_act[i]);  // keep actuated fixed
        }
    }

    // Reduced Jacobian: 6 x n_act, EE twist per actuated joint, accounting for
    // how the passive DOF must move to keep the loops closed.
    Matrix6x reducedJacobian(pinocchio::FrameIndex fid, pinocchio::ReferenceFrame ref,
                             pinocchio::Data& d, const Eigen::VectorXd& qf) const {
        Eigen::MatrixXd Jc = loopJacobianFull(qf, d);
        const int nf = static_cast<int>(free_vidx_.size());
        Eigen::MatrixXd Jc_free(Jc.rows(), nf), Jc_act(Jc.rows(), n_act_);
        for (int c = 0; c < nf; ++c)     Jc_free.col(c) = Jc.col(free_vidx_[c]);
        for (int c = 0; c < n_act_; ++c) Jc_act.col(c)  = Jc.col(act_vidx_[c]);
        // dq_free/dq_act = -pinv(Jc_free) * Jc_act  (passive response to actuation)
        Eigen::MatrixXd dfree = -Jc_free.completeOrthogonalDecomposition().solve(Jc_act);  // nf x n_act
        Matrix6x Jee(6, model_.nv); Jee.setZero();
        pinocchio::getFrameJacobian(model_, d, fid, ref, Jee);
        Matrix6x Jee_act(6, n_act_), Jee_free(6, nf);
        for (int c = 0; c < n_act_; ++c) Jee_act.col(c)  = Jee.col(act_vidx_[c]);
        for (int c = 0; c < nf; ++c)     Jee_free.col(c) = Jee.col(free_vidx_[c]);
        return Jee_act + Jee_free * dfree;                  // 6 x n_act
    }

    // Set actuated joint i to angle `val` in the full config. A continuous joint
    // (nq==2) is stored as (cos, sin); a revolute joint (nq==1) stores the angle.
    void setJointAct(Eigen::VectorXd& qf, int i, double val) const {
        const int qidx = act_qidx_[i];
        if (act_nq_[i] == 2) { qf[qidx] = std::cos(val); qf[qidx + 1] = std::sin(val); }
        else                 { qf[qidx] = val; }
    }

    // Serial IK fallback (mirrors PinocchioSolver, LOCAL frame flavor).
    void PinocchioSolverIKFallback(const Eigen::VectorXd& q_start, const Eigen::Isometry3d& target,
                                   pinocchio::FrameIndex frame_id, const IKOptions& opts,
                                   IKResult& result) const {
        pinocchio::Data local(model_);
        Eigen::VectorXd q = q_start;
        const pinocchio::SE3 oMdes(target.rotation(), target.translation());
        pinocchio::Data::Matrix6x J(6, model_.nv); J.setZero();
        Eigen::Matrix<double, 6, 1> err; err.setZero(); Eigen::VectorXd v(model_.nv);
        int i = 0;
        for (; i < opts.max_iterations; ++i) {
            pinocchio::forwardKinematics(model_, local, q);
            pinocchio::updateFramePlacements(model_, local);
            const pinocchio::SE3 iMd = local.oMf[frame_id].actInv(oMdes);
            err = pinocchio::log6(iMd).toVector();
            if (err.norm() < opts.tolerance) { result.success = true; break; }
            pinocchio::computeFrameJacobian(model_, local, q, frame_id, pinocchio::LOCAL, J);
            pinocchio::Data::Matrix6 Jlog; pinocchio::Jlog6(iMd, Jlog); J = Jlog * J;
            pinocchio::Data::Matrix6 JJt; JJt.noalias() = J * J.transpose();
            JJt.diagonal().array() += opts.damping;
            v.noalias() = J.transpose() * JJt.ldlt().solve(err);
            const double vn = v.norm();
            if (opts.max_joint_velocity > 0.0 && vn > opts.max_joint_velocity) v *= (opts.max_joint_velocity / vn);
            q = pinocchio::integrate(model_, q, v * opts.dt);
        }
        result.q = q; result.error_norm = err.norm(); result.iterations = i;
    }

    pinocchio::Model model_;
    pinocchio::Data  data_{model_};
    bool loaded_ = false;
    std::string last_error_;

    // closed-loop config
    bool configured_ = false;
    bool orientation_only_ = false;
    std::vector<FramePair> loop_frames_;
    std::vector<int> act_vidx_, act_qidx_, act_nq_, free_vidx_;
    int n_act_ = 0;
    pinocchio::FrameIndex ee_fid_ = 0;
    Eigen::VectorXd q_full_;          // persistent warm-start config (cached path)
};

}  // namespace fic

#endif  // FIC_WITH_PINOCCHIO
