#ifndef FIC_CONTROL_H
#define FIC_CONTROL_H

#include <cstddef>
#include <vector>
#include <memory>
#include <stdexcept>
#include <sstream>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <tinyxml2.h>
#include <cmath>
#include <limits>
#include <mutex>
#include <numeric>
#include <filesystem>
#include <string>
#include <algorithm>

// Library logic and parameter structs
#include "FIC.cpp" 
#include "fic_param_loader.hpp"

namespace fic {
    //===============================================//
    // Global state & defaults,mh
    //===============================================//
    inline double dt = 0.001;
    //===============================================//
    // compile‑time knobs
    //===============================================//
    inline size_t NumJoints = 1; // Number of joints in the robot make it so the user can change it
    inline bool   EnableAdmittance = true;
    std::vector<bool> jointIsContinuous_(NumJoints, false);
    std::vector<double> jointLower_;
    std::vector<double> jointUpper_;
    std::vector<double> error_max, force_max, saturation_ratios;
    std::vector<double> velocity;
    
    /**
     * @brief Top-level wrapper managing Joint, Cartesian, and Postural sub-controllers.
     */
    template <typename T>
    class ficControl {
    public:
        using FIC_t = FIC<T>;

        //=============================================================================
        // SUB-MODULE: JOINT CONTROLLER (JC)
        //=============================================================================
        struct JointController {
            ficControl* p; 

            // Clean syntax helper: JC.FIC(i, error)
            T FIC(size_t i, T error) { return p->fic_.control(i, error); }

            void impedance(size_t i, std::vector<double>& traj, std::vector<double>& torq, 
                           double q_target, double q_current, size_t n_j) {
                p->handleJointImpedanceControl(i, traj, torq, q_target, q_current, n_j);
            }

            void impedanceWithPlanner(size_t i, std::vector<double>& traj, std::vector<double>& torq, 
                                      double q_target, double q_current, size_t n_j) {
                p->handleJointImpedanceControlWithPlanner(i, traj, torq, q_target, q_current, n_j);
            }

            void admittance(size_t i, std::vector<double>& traj, std::vector<double>& torq,
                            double &q_target, double q_current, double des_eff,
                            double cur_eff, int mode, size_t n_j, bool use_planner = true) {
                p->handleJointAdmitanceControl(i, traj, torq, q_target, q_current, des_eff, cur_eff, mode, n_j, use_planner);
            }
        } JC;

        //=============================================================================
        // SUB-MODULE: ROBOT/CARTESIAN CONTROLLER (RC)
        //=============================================================================
        struct RobotController {
            ficControl* p;

            T FIC(size_t i, T error) { return p->fic_.control(i, error); }

            void impedance(std::vector<double>& traj, std::vector<double>& torq, 
                           Eigen::VectorXd& q_curr, const Eigen::VectorXd& grav, 
                           const std::vector<double>& X, const std::vector<double>& Xt, 
                           const Eigen::MatrixXd& J, int mode, const std::vector<double>& wrench,            
                           const std::vector<double>&  measured_joint_torques,
 
                           Eigen::VectorXd& q_t, size_t n_j) {
                p->cartesianimpedancecontrol(traj, torq, q_curr, grav, X, Xt, J, mode, wrench, measured_joint_torques, q_t, n_j);
            }

            void planner(std::vector<double>& traj, std::vector<double>& torq, 
                         Eigen::VectorXd& q_curr, const Eigen::VectorXd& grav, 
                         const std::vector<double>& X, const std::vector<double>& Xt, 
                         const Eigen::MatrixXd& J, int mode, const std::vector<double>& wrench, 
                         const std::vector<double>&  measured_joint_torques,
                         Eigen::VectorXd& q_t, size_t n_j) {
                p->cartesianPlanner(traj, torq, q_curr, grav, X, Xt, J, mode, wrench,measured_joint_torques, q_t, n_j);
            }
            void admittance(std::vector<double>& traj, std::vector<double>& torq,
                    Eigen::VectorXd& q_curr, const Eigen::VectorXd& grav,
                    const std::vector<double>& X, const std::vector<double>& Xt,
                    const Eigen::MatrixXd& J, int mode, const std::vector<double>& wrench,
                    const std::vector<double>&  measured_joint_torques,
                    Eigen::VectorXd& q_t,const Eigen::VectorXd& V, size_t n_j,
                    bool gravity_compensated = false) {
                p->cartesianAdmittanceControl(traj, torq, q_curr, grav, X, Xt, J, mode, wrench, measured_joint_torques, q_t, V, n_j, gravity_compensated);
            }

            void hapfic(std::vector<double>& traj, std::vector<double>& torq,
                        Eigen::VectorXd& q_curr, const Eigen::VectorXd& grav,
                        const std::vector<double>& X, const std::vector<double>& Xt,
                        const Eigen::MatrixXd& J, int mode, const std::vector<double>& wrench,
                        const std::vector<double>& measured_joint_torques,
                        Eigen::VectorXd& q_t, size_t n_j) {
                p->cartesianHapfic(traj, torq, q_curr, grav, X, Xt, J, mode, wrench, measured_joint_torques, q_t, n_j);
            }
        } RC;

        //=============================================================================
        // SUB-MODULE: POSTURAL CONTROLLER
        //=============================================================================
        struct PosturalController {
            ficControl* p;
            // Channels [0..5] are Cartesian axes, [6..8] are elbow. Postural lives
            // past those so it has its own FIC controllers in Cartesian (EE) modes.
            int controller_offset_ = 9;

            void setControllerOffset(int offset) { controller_offset_ = offset; }

            void init(size_t n_j, const fic::ScalarParams& s, const fic::ListParams& l) {
                for (size_t i = 0; i < n_j; ++i) {
                    p->initJointController(static_cast<int>(controller_offset_) + static_cast<int>(i), s, l);
                }
            }

            void apply(Eigen::VectorXd& q_curr, Eigen::VectorXd& q_t, bool mode, size_t n_j, std::vector<double>& tau) {
                p->applyJointPosturalCorrection(q_curr, q_t, mode, n_j, controller_offset_, tau);
            }
        } postural;

        //=============================================================================
        // CONSTRUCTOR & ACCESSORS
        //=============================================================================
        // Per-robot admittance deadbands (set via setDeadbands after construction)
        double deadband_joint_tau_  = 1.5;  // Nm — joint admittance torque deadband
        double deadband_cart_force_ = 4.0;  // N  — Cartesian translational force deadband
        double deadband_cart_torque_= 0.2;  // Nm — Cartesian rotational torque deadband

        void setDeadbands(double joint_tau, double cart_force, double cart_torque) {
            deadband_joint_tau_   = joint_tau;
            deadband_cart_force_  = cart_force;
            deadband_cart_torque_ = cart_torque;
        }

        // Desired wrench of the task-space admittance (modes 7/8, and HAP-FIC on host modes 7/8):
        // [Fx, Fy, Fz, Mx, My, Mz] in the frame and sign of the measured wrench. The admittance is
        // driven by desired - measured (after the deadbands), so it settles where the measured wrench
        // equals this one; zero (the start-up value) leaves the pure compliant behaviour. A message
        // with fewer than 6 or non-finite values is refused and the previous wrench kept.
        std::vector<double> desired_wrench_ = std::vector<double>(6, 0.0);

        bool setDesiredWrench(const std::vector<double>& w) {
            if (w.size() < 6) return false;
            for (size_t i = 0; i < 6; ++i) if (!std::isfinite(w[i])) return false;
            std::copy(w.begin(), w.begin() + 6, desired_wrench_.begin());
            return true;
        }
        const std::vector<double>& desiredWrench() const { return desired_wrench_; }

        // HAP-FIC's admittance drive, split (14 Oct 2026). The joint torque gap u = tau_des - tau_meas is
        // split into the part that produces an end-effector wrench, u_task = J^T w_gap with
        // w_gap = pinv(J^T) u, which passes a deadband in task space on w_gap (force and torque
        // magnitudes, with hysteresis: engaged at the deadband, released below kHapficDeadbandRelease
        // of it), and the rest, u_null = u - u_task, which passes the joint deadband per joint. Without
        // the deadbands the recombined drive is u, as before. With the joint deadband alone a held
        // contact kept up to b on every joint, which the arm's lever arms turn into several newtons at the
        // end-effector (Kinova, b = 2 N m: 1.7-5.2 N in simulation); with the split it ends within the
        // task release level of the task-layer law, whatever the lever arms. Host path only: with
        // hapfic_export_only_ (the inner-loop MCU runs the joint layer) the per-joint deadband is kept.
        bool   hapfic_split_drive_    = true;
        double hapfic_task_db_force_  = 1.0;    // N, engage (release 0.25 N)
        double hapfic_task_db_torque_ = 0.1;    // N m, engage (release 0.025 N m)
        bool   hapfic_task_engaged_f_ = false;
        bool   hapfic_task_engaged_t_ = false;

        void setHapficSplitDrive(bool on, double force_db, double torque_db) {
            hapfic_split_drive_    = on;
            hapfic_task_db_force_  = force_db;
            hapfic_task_db_torque_ = torque_db;
            hapfic_task_engaged_f_ = hapfic_task_engaged_t_ = false;
        }

        // Per-instance joint-admittance state, promoted out of function-local
        // statics in handleJointAdmitanceControl (a function-local static in a
        // member function is shared by every instance, which corrupted offsets
        // across robots in a multi-robot setup). Lazily resized to NumJoints the
        // same way the former statics were — see handleJointAdmitanceControl.
        std::vector<double> joint_adm_offset_;      // accumulated human push offset per joint (rad)
        std::vector<double> joint_adm_last_delta_;  // last raw admittance delta per joint (rad)
        std::vector<char>   joint_adm_engaged_;     // deadband hysteresis: yielding until the gap is below release

        // Accumulated per-joint "human push offset" (radians): how far the human
        // has pushed joint i away from the commanded target under joint admittance
        // (mode 1). Returns 0.0 for an out-of-range index.
        double humanOffset(size_t i) const {
            return (i < joint_adm_offset_.size()) ? joint_adm_offset_[i] : 0.0;
        }

        // Hapfic: Cartesian planner+impedance channels at a dedicated offset so they
        // don't collide with joint controllers at 0..n_j or postural at 9..9+n_j.
        int hapfic_cart_planner_offset_ = 16;

        // Inner-loop MCU: when set, cartesianHapfic stops after the Cartesian
        // layer and leaves tau_des = J^T*w + g in hapfic_tau_des_ instead of
        // running the joint admittance layer here — the MCU runs that layer.
        bool hapfic_export_only_ = false;
        Eigen::VectorXd hapfic_tau_des_;

        // Hapfic on host modes 7/8: the host's task-space admittance shifts the task layer's
        // goal. It runs on its own integrators (the admittance blocks of channels off+0..off+5,
        // off = hapfic_cart_planner_offset_; the joint layer uses 0..n_j-1) with its own state.
        // hapfic_q_shift_ is the first-order shift of the IK set-point, J# dx, that follows the
        // shifted goal; the MCU path adds it to the set-point it sends.
        std::vector<double> hapfic_cadm_offset_     = std::vector<double>(6, 0.0);
        std::vector<double> hapfic_cadm_last_delta_ = std::vector<double>(6, 0.0);
        int                 hapfic_cadm_last_mode_  = -1;
        Eigen::VectorXd     hapfic_q_shift_;
        static constexpr double kHapficIkDamping = 0.01;   // DLS lambda of J#, as the elbow helper

        void resetHapficTaskAdmittance(int mode) {
            std::fill(hapfic_cadm_offset_.begin(), hapfic_cadm_offset_.end(), 0.0);
            std::fill(hapfic_cadm_last_delta_.begin(), hapfic_cadm_last_delta_.end(), 0.0);
            for (int k = 0; k < 6; ++k) this->admCtrl(hapfic_cart_planner_offset_ + k).resetState();
            hapfic_cadm_last_mode_ = mode;
        }

        void initHapficCartesian(const fic::CartesianAllParams& cart_params,
                                 const std::vector<double>& current_pose) {
            int off = hapfic_cart_planner_offset_;
            for (int i = 0; i < 3; ++i) {
                initCartesianController(off + i, cart_params.pos_axes[i]);
                initCartesianPlanner(off + i, current_pose[i], cart_params.pos_axes[i]);
            }
            initCartesianController(off + 3, cart_params.rot_axis);
            // Elbow channels after the HAP-FIC Cartesian ones (off+4 .. off+6), clear
            // of the joint channels 0..n_j-1 (default 6..8 collides with joint 6).
            initElbowController(cart_params, off + 4);
            // Task-space admittance (host modes 7/8) on the admittance blocks of off..off+5,
            // with the Cartesian layer's admittance parameters; its bounds are set per tick.
            for (int i = 0; i < 3; ++i) initCartesianAdmittance(off + i, cart_params.pos_axes[i]);
            for (int i = 3; i < 6; ++i) initCartesianAdmittance(off + i, cart_params.rot_axis);
            resetHapficTaskAdmittance(-1);
            hapfic_task_engaged_f_ = hapfic_task_engaged_t_ = false;   // split drive's hysteresis
        }

        ficControl(T N_s, bool asymmetric = false, T N_negSat = T{}, size_t n_dof = 7, bool enable_adm = true)
            : fic_(N_s, asymmetric, N_negSat, n_dof, enable_adm),
              JC{this}, RC{this}, postural{this}
        {
            // Initialize metadata vectors to the requested DOF
            jointIsContinuous_.assign(n_dof, false);
            jointLower_.assign(n_dof, -M_PI);
            jointUpper_.assign(n_dof, M_PI);
        }

        // Consolidated Accessors
        auto& controller(size_t i) { return fic_.getController(i); }
        auto& planner(size_t i)    { return fic_.getPlanner(i); }
        auto& admCtrl(size_t i)    { return fic_.getAdmittanceController(i); }

        // HAP-FIC: a joint set-point no further than `bound` from where the joint is (wrapped for a
        // continuous joint). With the IK set-point inside the admittance bound, the offset can
        // always cancel it, so a blocked end-effector does not open the joint errors.
        // Smallest set-point window: a narrow free-space bound (+-0.02 rad) would otherwise hold the
        // set-point back in free motion (Kinova lemniscate, simulation: RMSE 26 mm against 9.7 mm).
        static constexpr double kSetpointWindowMin = 0.1;   // rad
        // HAP-FIC's deadband hysteresis: release at this fraction of the joint deadband.
        static constexpr double kHapficDeadbandRelease = 0.25;
        static double setpointWindow(double bound) { return std::max(std::fabs(bound), kSetpointWindowMin); }

        static double limitSetpoint(double q_target, double q, double bound, bool continuous) {
            double gap = q_target - q;
            if (continuous) gap = std::atan2(std::sin(gap), std::cos(gap));
            if (std::fabs(gap) <= bound) return q_target;
            const double lim = q + std::copysign(bound, gap);
            return continuous ? std::atan2(std::sin(lim), std::cos(lim)) : lim;
        }

        T control(size_t i, T err)       { return fic_.control(i, err); }
        T plan(size_t i, T p, T dt_val)  { return fic_.plan(i, p, dt_val); }
        T admittance(size_t i, T dt_val) { return fic_.admittance(i, dt_val); }

        // ==========================================
        // SINGULAR JOINT COMPONENT INITIALIZERS
        // ==========================================

        void initJointController(int i, const fic::ScalarParams& s, const fic::ListParams& l) {
            auto& c = this->controller(i);

            int n_s = l.n_saturations;
            int n_neg = l.n_neg_sat;
            if (!l.asymmetric && !l.error_max.empty()) {
                n_s = static_cast<int>(l.error_max.size()) * 2;
                n_neg = static_cast<int>(l.error_max.size());
            }
            c.reconfigure(n_s, l.asymmetric, n_neg);

            // FIC Nonlinear Mapping
            c.set_x_b(Eigen::Map<const Eigen::VectorXd>(l.error_max.data(), static_cast<Eigen::Index>(l.error_max.size())));
            c.setE_max(Eigen::Map<const Eigen::VectorXd>(l.force_max.data(), static_cast<Eigen::Index>(l.force_max.size())));

            // Saturation Logic
            if (!l.saturation_ratios.empty()) {
                c.setS_Start(Eigen::Map<const Eigen::VectorXd>(l.saturation_ratios.data(), static_cast<Eigen::Index>(l.saturation_ratios.size())));
                c.setNumSaturations(static_cast<int>(l.saturation_ratios.size()));
            } else {
                c.setS_Start(Eigen::VectorXd::Constant(1, 0.9));
                c.setNumSaturations(1);
            }
            c.setStiffness(Eigen::VectorXd::Constant(2, s.stiffness));
        }

        void initJointPlanner(int i, double current_pos, const fic::PlannerParams& p_vals) {
            // Continuity + limits are sourced from the robot config (setJointContinuous /
            // setJointLimits), not from the per-mode XML. Mode-switch paths that skip this
            // function (Cartesian modes) still get correct values.
            auto& p = this->planner(i);
            p.resetState(current_pos);
            p.setStiffness(p_vals.Plannerstiffness);
            p.setVelDesired(p_vals.vel_desired);
            p.setAccMax(p_vals.acc_max);
            p.setDampingRatio(p_vals.damping_ratio);
            p.setFrequency(p_vals.frequency);
            p.setWrapped(jointIsContinuous_[i] ? 1 : 0);
            p.setPosLimits(jointLower_[i], jointUpper_[i]);
        }

        // Robot-level metadata: call once per robot load, BEFORE any mode switch, so
        // Cartesian modes (which skip initJointPlanner) also see correct values.
        void setJointContinuous(const std::vector<bool>& v) {
            if (v.empty()) return;
            if (jointIsContinuous_.size() < v.size()) jointIsContinuous_.resize(v.size(), false);
            for (size_t i = 0; i < v.size(); ++i) jointIsContinuous_[i] = v[i];
        }

        void setJointLimits(const std::vector<double>& lower, const std::vector<double>& upper) {
            const size_t n = std::min(lower.size(), upper.size());
            if (jointLower_.size() < n) jointLower_.resize(n, -M_PI);
            if (jointUpper_.size() < n) jointUpper_.resize(n,  M_PI);
            for (size_t i = 0; i < n; ++i) {
                jointLower_[i] = lower[i];
                jointUpper_[i] = upper[i];
            }
        }

        void initJointAdmittance(int i, const fic::ListParams& l, const fic::PlannerParams& p_vals, const fic::AdmittanceParams& a_vals) {
            auto& a = this->admCtrl(i);
            auto& c = this->controller(i);

            a.setInvM_Kv(a_vals.InvM,a_vals.damping);
            a.setMinMax_dFlow(a_vals.dFlow[0], a_vals.dFlow[1]);
            a.setMinMax_Flow(a_vals.flow[0], a_vals.flow[1]);
            Eigen::VectorXd xb_vec = c.get_x_b();
            //a.setMinMax_Pose(a_vals.pose[0], a_vals.pose[1]);// should be xb from controller
            // Outer breakpoints of the mirrored x_b: [x_b(0), x_b(N_S-1)] = [-em_max, +em_max]
            // for a symmetric profile. (x_b(0), x_b(1)) were the two most negative
            // breakpoints, so the offset could never be 0.
            a.setMinMax_Pose(xb_vec(0), xb_vec(xb_vec.size() - 1));

            a.resetState();
            if (i < joint_adm_engaged_.size()) joint_adm_engaged_[i] = 0;
        }


        // ==========================================
        // CARTESIAN COMPONENT INITIALIZERS (Channels 0-5)
        // ==========================================

        /** @brief Setup math (FIC Controller) for a Cartesian axis (Translation or Rotation) */
        void initCartesianController(int idx, const fic::CartesianAxisParams& params) {
            auto& c = this->controller(idx);

            int n_s = params.n_saturations;
            int n_neg = params.n_neg_sat;
            if (!params.asymmetric && !params.error_max.empty()) {
                n_s = static_cast<int>(params.error_max.size()) * 2;
                n_neg = static_cast<int>(params.error_max.size());
            }
            c.reconfigure(n_s, params.asymmetric, n_neg);

            // FIC Nonlinear Mapping using Cartesian-specific params
            if (!params.error_max.empty() && !params.force_max.empty()) {
                c.set_x_b(Eigen::Map<const Eigen::VectorXd>(params.error_max.data(), static_cast<Eigen::Index>(params.error_max.size())));
                c.setE_max(Eigen::Map<const Eigen::VectorXd>(params.force_max.data(), static_cast<Eigen::Index>(params.force_max.size())));
            }

            // Saturation Logic
            if (!params.saturation_ratios.empty()) {
                c.setS_Start(Eigen::Map<const Eigen::VectorXd>(params.saturation_ratios.data(), static_cast<Eigen::Index>(params.saturation_ratios.size())));
                c.setNumSaturations(static_cast<int>(params.saturation_ratios.size()));
            }

            // Cartesian Stiffness (Translational Kp or Rotational Kq)
            c.setStiffness(Eigen::VectorXd::Constant(2, params.stiffness));
        }

        /** @brief Setup movement (Planner) for a Cartesian axis (usually Translation 0-2) */
        void initCartesianPlanner(int idx, double current_pos, const fic::CartesianAxisParams& params) {
            auto& p = this->planner(idx);
            
            p.resetState(current_pos);
            p.setStiffness(params.stiffness_P); 
            p.setVelDesired(params.vel_desired);
            p.setAccMax(params.acc_max);
            p.setDampingRatio(params.damping_ratio);
            p.setFrequency(params.frequency);
            
            // Cartesian space typically doesn't wrap and has wide safety limits
            p.setWrapped(false);
            p.setPosLimits(-10.0, 10.0); 
        }

        void initCartesianAdmittance(int idx, const fic::CartesianAxisParams& params) {
            auto& a = this->admCtrl(idx);
            auto& c = this->controller(idx);

            const auto& adm = params.admittance;

            // Apply exact same methods as the joint controller
            a.setInvM_Kv(adm.InvM, adm.damping);
            a.setMinMax_dFlow(adm.dFlow[0], adm.dFlow[1]); // Fix: Changed to dFlow
            a.setMinMax_Flow(adm.flow[0], adm.flow[1]);
            Eigen::VectorXd xb_vec = c.get_x_b();
            a.setMinMax_Pose(xb_vec(0), xb_vec(xb_vec.size() - 1)); // outer breakpoints (see initJointAdmittance)
            //a.setMinMax_Pose(adm.pose[0], adm.pose[1]);
            a.resetState();
        }
        // ==========================================
        // ELBOW COMPONENT INITIALIZERS (Channels 6-11)
        // ==========================================

        /** * @brief Setup the 6 channels for the Elbow Controller.
         * By default, it reuses the Cartesian params (Linear/Angular) 
         * but applies them to indices 6-11.
         */
        // First of the 3 elbow channels; pass it to applyElbowControl. 6 in the
        // Cartesian modes (after the EE channels 0-5). HAP-FIC runs joint
        // channels 0..n_j-1, which on a 7-DoF arm include 6, so it moves the
        // elbow past its own Cartesian channels (initHapficCartesian).
        int elbow_offset_ = 6;
        int elbowOffset() const { return elbow_offset_; }

        void initElbowController(const fic::CartesianAllParams& params, int start_idx = 6) {
            elbow_offset_ = start_idx;
            // 1. Linear Elbow (Channels start_idx .. start_idx+2) -> X, Y, Z
            for (int i = 0; i < 3; ++i) {
                // Reuse the generic logic, but pass the elbow index
                initCartesianController(start_idx + i, params.pos_axes[i]);
                }
        }
        void applyElbowControl(std::vector<double>& tau, 
                       const Eigen::VectorXd& q_curr, 
                       const Eigen::MatrixXd& J_prim,
                       const Eigen::MatrixXd& J_elb,
                       const std::vector<double>& X_curr,
                       const std::vector<double>& X_targ,
                       size_t n_j,
                       int ctrl_start_idx) 
        {
            // 1. Unpack Elbow Position only (Ignore X_curr[3..6])
            Eigen::Vector3d pc(X_curr[0], X_curr[1], X_curr[2]);
            Eigen::Vector3d pt(X_targ[0], X_targ[1], X_targ[2]);

            // 2. Compute 3D Linear Error Vector
            Eigen::Vector3d pos_err = pt - pc;

            // 3. Compute Linear Force using FIC Controller
            // We only use 3 channels starting from ctrl_start_idx (e.g., 6, 7, 8)
            Eigen::Vector3d f_cmd;
            for (int k = 0; k < 3; ++k) {
                f_cmd[k] = this->control(ctrl_start_idx + k, pos_err[k]);     
            }

            // 4. Map Force to Torques: tau_elb = J_linear^T * Force
            // We use .topRows(3) to get only the translational part of the Elbow Jacobian
            Eigen::VectorXd tau_elb = J_elb.topRows(3).transpose() * f_cmd;

            // 5. Compute Null-Space Projector for Primary Task (EE)
            // N = I - J#J
            double lambda = 0.01; // Damping for pseudo-inverse
            Eigen::MatrixXd JJT = J_prim * J_prim.transpose();
            JJT.diagonal().array() += lambda * lambda;
            Eigen::MatrixXd J_prim_pinv = J_prim.transpose() * JJT.inverse();
            
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n_j, n_j);
            Eigen::MatrixXd NullSpace = I - (J_prim_pinv * J_prim);

            // 6. Project Elbow effort into the Null Space
            // This ensures the elbow movement does not change the End-Effector position
            Eigen::VectorXd tau_null = NullSpace * tau_elb;

            // 7. Accumulate into the existing torque vector
            for (size_t i = 0; i < n_j; ++i) {
                tau[i] += tau_null[i];
            }
}
        
        inline double wrapToPi(double x) {
            return std::atan2(std::sin(x), std::cos(x));
        }

        //===============================================//
        // Single‑step handler
        //===============================================//

        void handleJointImpedanceControl(
            size_t i,
            std::vector<double>& traj_msg,
            std::vector<double>& torque_msg,
            double q_target,
            double q_current,
            size_t NumJoints) {


            if (jointIsContinuous_[i] == 1) {
                q_current = wrapToPi(q_current);  // only wrap encoder reading for continuous joints
            } else {
                q_target = std::clamp(q_target, jointLower_[i], jointUpper_[i]);// q target needs to be clamped at joint limits if not wrapped = 0
            }
            // error & control
            double error = q_target - q_current; 
            // only wrap for continuous (revolute) joints
            if (jointIsContinuous_[i] == 1) {
                error = wrapToPi(error); //THE WRAPP COULDBE CHANGED TO OTHER METHOD
            }

            // control
            double controlEffort = this->control(i, error);

            // resize outputs
            if (traj_msg.size() < NumJoints*8) {
            }
                traj_msg.resize(NumJoints*8, 0.0);
            if (torque_msg.size() < NumJoints){
                torque_msg.resize(NumJoints, 0.0);
            }
            // fill in
            traj_msg[i + 1*NumJoints] = q_current;
            traj_msg[i + 2*NumJoints] = error;
            traj_msg[i + 3*NumJoints] = q_target;
            traj_msg[i + 4*NumJoints] = controlEffort;
            torque_msg[i]             = controlEffort;
        }


        void handleJointImpedanceControlWithPlanner(
            size_t i,
            std::vector<double>& traj_msg,
            std::vector<double>& torque_msg,
            double q_target,
            double q_current,
            size_t NumJoints
            ) {

            // joint mode is impedance control
            // planner → target pos
            // q_current = wrapToPi(q_current);  // robot specific if encoder works 0 to 2pi
            double plannerTargetPos = this->plan(i, q_target, dt); // of not wrapped must be clamped at joint limit

            if (jointIsContinuous_[i] == 0) {
                plannerTargetPos = std::clamp(plannerTargetPos, jointLower_[i], jointUpper_[i]);// q target needs to be clamped at joint limits if not wrapped = 0
            }

            handleJointImpedanceControl(i, traj_msg, torque_msg, plannerTargetPos, q_current, NumJoints);

            // resize outputs
            if (traj_msg.size() < NumJoints*8) 
                traj_msg.resize(NumJoints*8, 0.0);

             traj_msg[i + 0*NumJoints] = plannerTargetPos;


        }

        void handleJointAdmitanceControl(
            size_t i,
            std::vector<double>& traj_msg,
            std::vector<double>& torque_msg,
            double &q_target,
            double q_current,
            double DesiredEffort,      // Passed as gravity_torques_[i]
            double current_effort_,     // Passed as joint_torques_[i] (Positive!)
            int AdmMode_,
            size_t NumJoints,
            bool use_planner = true,
            double release_tau = 0.0,  // > 0: deadband with hysteresis (HAP-FIC), see step 2
            double drive = std::numeric_limits<double>::quiet_NaN()   // set: the deadbanded drive,
            ) {                                                       // from the caller (HAP-FIC split)

            auto& a = this->admCtrl(i);

            // 1. Calculate external force
            double ext_tau = DesiredEffort - current_effort_;

            // 2. Deadband to ignore sensor noise + residual gravity-model error.
            //    Franka's tau_ext_hat_filtered carries 1-2 Nm of bias in heavy
            //    poses if a tool isn't registered via setLoad() — too low a
            //    deadband makes the admittance integrate that bias and drift.
            //    With release_tau > 0 (HAP-FIC) the deadband has hysteresis: the joint starts
            //    yielding when the gap reaches the deadband and keeps yielding until it is below
            //    release_tau, so a held contact ends within release_tau of the desired torque, not
            //    anywhere inside the deadband; at rest a bias below the deadband is still ignored.
            if (!std::isnan(drive)) {
                ext_tau = drive;   // already deadbanded by the caller (cartesianHapfic, split drive)
            } else if (release_tau > 0.0) {
                if (joint_adm_engaged_.size() < NumJoints) joint_adm_engaged_.resize(NumJoints, 0);
                char& on = joint_adm_engaged_[i];
                if (!on && std::abs(ext_tau) >= deadband_joint_tau_) on = 1;
                else if (on && std::abs(ext_tau) < release_tau) on = 0;
                if (!on) ext_tau = 0.0;
            } else if (std::abs(ext_tau) < deadband_joint_tau_) {
                ext_tau = 0.0;
            }

            // 3. Feed the isolated force into the admittance block. Both modes are driven
            //    by the interaction torque alone (desired minus measured, outside the
            //    deadband): with nothing pushing, the offset does not move. A desired
            //    effort in the drive as well would count it twice: the offset would creep
            //    with no push (gravity alone drives it) and, at a contact, rest where the
            //    joint applies twice the desired torque. Gravity is fed forward to the
            //    motors by the caller instead (modes 3/4 and HAP-FIC, both nodes).
            a.setDesiredEffort(0.0);
            a.setCurrentEffort(-ext_tau);
            // 4. Get the absolute offset from the admittance math
            double adm_delta = this->admittance(i, dt); 
            
            // 5. Track Delta securely
            // (per-instance member, not a function-local static — see joint_adm_last_delta_)
            auto& last_adm_delta = this->joint_adm_last_delta_;
            if (last_adm_delta.size() < NumJoints) {
                last_adm_delta.resize(NumJoints, 0.0);
            }
            double step_size = adm_delta - last_adm_delta[i];
            last_adm_delta[i] = adm_delta;

            // 6. Independent Human Offset Tracking
            // This allows the inner loop to stay stiff (Kp=15) while the arm still yields to your push
            // (per-instance member, not a function-local static — see joint_adm_offset_)
            auto& offest = this->joint_adm_offset_;
            if (offest.size() < NumJoints) {
                offest.resize(NumJoints, 0.0);
            }

            double q_targetAdm;
            double current_stable_q_target = wrapToPi(q_target); // The hub's original command

            if (AdmMode_ == 1) { 
                // We want the internal math to track the offset indefinitely without getting stuck.
                a.setMinMax_Pose(jointLower_[i], jointUpper_[i]);               

                // MODE 1: PURE ADMITTANCE (Push-and-Stay)
                // Accumulate the micro-step to our permanent human offset
                offest[i] += step_size; 
                
                // Final target = Hub Command + Human Push
                q_targetAdm = current_stable_q_target + offest[i]; 
            } 
            else {
                // MODE 2: HAPTIC ADMITTANCE (Spring-Return)
                offest[i] = 0.0; // Clear offset so it doesn't jump
                q_targetAdm = current_stable_q_target + adm_delta; 
            }

            // 7. Clamping and Anti-Windup (The TRUE physical wall)
            if (jointIsContinuous_[i] == 1) {
                q_targetAdm = wrapToPi(q_targetAdm); 
            } else {
                q_targetAdm = std::clamp(q_targetAdm, jointLower_[i], jointUpper_[i]);
                
                // If we hit the physical limit, stop adding to the human offset!
                // This prevents the math from winding up if you keep pushing against the wall
                if (q_targetAdm == jointUpper_[i] || q_targetAdm == jointLower_[i]) {
                    if (AdmMode_ == 1) {
                        offest[i] -= step_size; 
                    }
                }
            }
            
            // 8. Admittance target → Impedance (optionally through the planner)
            if (use_planner) {
                handleJointImpedanceControlWithPlanner(i, traj_msg, torque_msg, q_targetAdm, q_current, NumJoints);
            } else {
                handleJointImpedanceControl(i, traj_msg, torque_msg, q_targetAdm, q_current, NumJoints);
            }

            // 9. Resize and fill diagnostic arrays
            if (traj_msg.size() < NumJoints*8) traj_msg.resize(NumJoints*8, 0.0);
            traj_msg[i + 5*NumJoints] = q_targetAdm;
            traj_msg[i + 6*NumJoints] = DesiredEffort;
            traj_msg[i + 7*NumJoints] = current_effort_; 
        }

        // --- Quaternion orientation error helper ---
        struct SO3Error {
            Eigen::Vector3d axis;     // unit axis ê
            double angle;             // θ in [0, π]
            Eigen::Vector3d rotvec;   // θ * ê  (axis–angle vector)
        };

        // Returns the shortest-arc rotation from q_curr to q_des.
        // NOTE (13 Oct 2026): not called anywhere yet. It is the candidate replacement for the
        // orientation law of cartesianimpedancecontrol and cartesianHapfic (see the note there):
        // theta in [0, pi] about the axis of q_err flipped to q_s >= 0, so q_des and -q_des give
        // the same error. To be tried before any switch (C. Tiseo, 13 Oct 2026).
        inline SO3Error quatErrorShortestArc(const Eigen::Quaterniond& q_curr_raw,
                                                const Eigen::Quaterniond& q_des_raw)
        {
            Eigen::Quaterniond q_curr = q_curr_raw.normalized();
            Eigen::Quaterniond q_des  = q_des_raw.normalized();

            Eigen::Quaterniond q_err = q_des * q_curr.conjugate();

            if (q_err.w() < 0.0) q_err = Eigen::Quaterniond(-q_err.w(), -q_err.x(), -q_err.y(), -q_err.z());

            const double vnorm = q_err.vec().norm();
            const double w     = q_err.w();
            double angle       = 2.0 * std::atan2(vnorm, w);

            Eigen::Vector3d axis;
            if (vnorm > 1e-12) axis = q_err.vec() / vnorm;
            else               axis = Eigen::Vector3d::UnitX();

            return {axis, angle, axis * angle};
        }

        // Postural task: biases the redundant DOF toward q_target as a direct
        // secondary joint term. Applied in ALL Cartesian (EE) modes, including
        // the Cartesian admittance modes (7/8).
        void applyJointPosturalCorrection(Eigen::VectorXd& q_current,
                                        Eigen::VectorXd& q_target,
                                        bool CartAdmMode,
                                        size_t num_joints_,
                                        int controller_offset,
                                        std::vector<double>& tau)
        {
            (void)CartAdmMode; // postural now runs in all Cartesian modes (incl. admittance)

            for (size_t i = 0; i < num_joints_; ++i) {
                if (jointIsContinuous_[i] == 1) {
                    q_current[i] = wrapToPi(q_current[i]);
                } else {
                    q_target[i] = std::clamp(q_target[i], jointLower_[i], jointUpper_[i]);
                }

                double error = q_target[i] - q_current[i];
                if (jointIsContinuous_[i] == 1) {
                    error = wrapToPi(error);
                }

                tau[i] += this->control(controller_offset + static_cast<int>(i), error);
            }
        }

        void cartesianimpedancecontrol(
            std::vector<double>&        traj_msg,
            std::vector<double>&        torque_msg,
            Eigen::VectorXd&            q_current,        // n×1 joint positions
            const Eigen::VectorXd&      gravity_torques_, // n×1 gravity torques
            const std::vector<double>&  X,                // [x,y,z,qx,qy,qz,qw] current EE pose
            const std::vector<double>&  Xt,               // [x,y,z,qx,qy,qz,qw] target EE pose
            const Eigen::MatrixXd&      jacobian,         // 6×n EE Jacobian (Primary)
            int                         CartAdmMode,      // 0=no adm, 1=CA, 2=CA+I
            const std::vector<double>&  current_wrench_,  // [Fx,Fy,Fz,Tx,Ty,Tz]
            const std::vector<double>&  measured_joint_torques,
            Eigen::VectorXd&            q_target,
            size_t                      num_joints_
        ){
            // --- 1) Unpack current & desired Cartesian states ---
            Eigen::Vector3d curr_pos(X[0], X[1], X[2]);
            Eigen::Vector3d des_pos (Xt[0], Xt[1], Xt[2]);

            Eigen::Quaterniond q_curr(X[6], X[3], X[4], X[5]);
            Eigen::Quaterniond q_des (Xt[6], Xt[3], Xt[4], Xt[5]);
            Eigen::Vector3d pos_ref;
  
            q_curr.normalize(); q_des.normalize();

            // (Optional) internal impedance wrench for CA+I (Mode 2)
            Eigen::Vector3d wrench_imp = Eigen::Vector3d::Zero();
            
            // --- 5) Orientation control (Eq. 4 from FIC quaternion paper) ---
            //
            // NOTE (13 Oct 2026) -- ORIENTATION LAW UNDER REVIEW, NOT CHANGED.
            // C. Tiseo: "do not touch the quaternion for now" / "we will need to try".
            // Deployed law:
            //     theta = 2 atan(|q_v| / max(q_s, 1e-12))
            //     tau   = sign(q_s) * f_3(theta) * q_v / |q_v|
            // Issue: for q_s < 0 the max() clamps the denominator to 1e-12, so theta saturates at
            // about pi whatever the true error; sign(q_s) only fixes the direction. q and -q are the
            // same orientation (double cover), so a goal whose quaternion arrives with the other
            // sign (teleoperation, VR, scripts) gives q_s < 0 for a small error: e.g. 2 deg gives
            // the profile's cap (Kinova orientation profile of modes 6-8 and HAP-FIC: 50 N m/rad,
            // cap 10 N m) instead of about 1.7 N m, and 0 deg gives the cap about an axis set by
            // noise (|q_v| ~ 0). Every hardware run so far held the orientation read from the arm's own
            // pose (errors < 1 deg, q_s ~ 1), where both forms agree.
            // Proposed fix (C. Tiseo): put the sign in the angle and drop it from the torque,
            //     s     = (q_s >= 0 ? 1 : -1)                  (q_s == 0 must count as +1)
            //     theta = 2 atan(|q_v| / (s * max(|q_s|, 1e-12)))
            //     tau   = f_3(theta) * q_v / |q_v|             (no sign(q_s): it would flip twice)
            // i.e. the shortest-arc error: quatErrorShortestArc() above does the same (flip q_err
            // to q_s >= 0, theta = 2 atan2(|q_v|, q_s) in [0, pi]); equal for symmetric profiles,
            // which every Kinova profile is (an asymmetric one would use its negative side).
            // Try before switching: (1) host test: same torque for q_des and -q_des; a small error
            // gives f_3(error); errors near pi (the direction flip at exactly pi stays, as in any
            // shortest-path law); (2) sim A/B of modes 5-8 and HAP-FIC with the goal quaternion
            // sign flipped and with large orientation errors; (3) the robot. Change this law and
            // its copy in cartesianHapfic (channel 19) together.
            Eigen::Quaterniond q_err = q_des * q_curr.conjugate();
            Eigen::Vector3d q_v = q_err.vec();
            double q_s = q_err.w();
            double vnorm = q_v.norm();
            double theta = 2.0 * std::atan(vnorm / std::max(q_s, 1e-12));

            // FIC control on scalar angle θ (channel 3)
            double tau_cq = this->control(3, theta);

            // Project back: τ_c = sign(q̃_s) * τ_cq * q̃_v / ||q̃_v||
            vnorm = std::max(vnorm, 1e-12);
            Eigen::Vector3d ori_torque = (q_s >= 0.0 ? 1.0 : -1.0) * tau_cq * (q_v / vnorm);

            // --- 6) Cartesian error & wrench command ---
            Eigen::Matrix<double,6,1> wrench_cmd;
            for (int ax = 0; ax < 3; ++ax) {
                wrench_cmd[ax] = this->control(ax, des_pos[ax] - curr_pos[ax]);
            }
            wrench_cmd.tail<3>() = ori_torque;

            Eigen::VectorXd tau = jacobian.transpose() * wrench_cmd + gravity_torques_;
            
            torque_msg.assign(tau.data(), tau.data() + tau.size());
            
            // --- 8) Fill trajectory & torque messages (same layout) ---

            // for (size_t i = 0; i < num_joints_; ++i) {
            //     // We pass:
            //     // - tau_des[i] as DesiredEffort (what the Cartesian controller wants)
            //     // - measured_joint_torques[i] as current_effort_ (what the robot actually feels)
            //     // - q_target(i) as the persistent reference to be updated by admittance
                
            //     handleJointAdmitanceControl(
            //         i,
            //         traj_msg,
            //         torque_msg,
            //         q_target(i),               // Ref to persistent target
            //         q_current(i),
            //         tau(i),                // <--- Cartesian Torque as "Desired"
            //         measured_joint_torques[i], // <--- Measured Torque
            //         CartAdmMode,
            //         num_joints_
            //     );
            // }
            std::vector<double> controlEffortVec(num_joints_, 0.0);  // Store control efforts per joint
            Eigen::VectorXd controlEffortEigen = Eigen::Map<Eigen::VectorXd>(controlEffortVec.data(), controlEffortVec.size());
                tau = tau + controlEffortEigen;


            // --- 8) Fill trajectory & torque messages (same layout) ---
            traj_msg.assign(num_joints_ * 8, 0.0);
            torque_msg.assign(num_joints_, 0.0);

            for (size_t i = 0; i < num_joints_; ++i) {
                traj_msg[i*8 + 0] = q_current[i];
                traj_msg[i*8 + 1] = q_target[i];
                torque_msg[i]     = tau[i];
            }
            const size_t base = num_joints_ * 8;
            if (traj_msg.size() < base + 16) traj_msg.resize(base + 16, 0.0);

            traj_msg[base + 3]  = curr_pos.x(); // current
            traj_msg[base + 4]  = curr_pos.y();
            traj_msg[base + 5]  = curr_pos.z();

            traj_msg[base + 6]  = (des_pos - curr_pos).x(); // error
            traj_msg[base + 7]  = (des_pos - curr_pos).y();
            traj_msg[base + 8]  = (des_pos - curr_pos).z();

            traj_msg[base + 9]  = des_pos.x();   // raw desired
            traj_msg[base + 10] = des_pos.y();
            traj_msg[base + 11] = des_pos.z();

            traj_msg[base + 12] = ori_torque.x();
            traj_msg[base + 13] = ori_torque.y();
            traj_msg[base + 14] = ori_torque.z();
            traj_msg[base + 15] = theta;

        }

        void cartesianPlanner(
            std::vector<double>&        traj_msg,
            std::vector<double>&        torque_msg,
            Eigen::VectorXd&            q_current,
            const Eigen::VectorXd&      gravity_torques_,
            const std::vector<double>&  X,                
            const std::vector<double>&  Xt,               
            const Eigen::MatrixXd&      jacobian,
            int                         CartAdmMode,
            const std::vector<double>&  current_wrench_,
            const std::vector<double>&  measured_joint_torques,
            Eigen::VectorXd&            q_target,
            size_t                      num_joints_
        ){
            // --- 1) Unpack current & desired Cartesian states ---
            Eigen::Vector3d curr_pos(X[0], X[1], X[2]);
            Eigen::Vector3d des_pos (Xt[0], Xt[1], Xt[2]);

            // --- 2) Plan position trajectory ---
            Eigen::Vector3d pos_plan;
            for (int ax = 0; ax < 3; ++ax)
                pos_plan[ax] = this->plan(ax, des_pos[ax], dt);

            // FIX: Create a Planned Target vector (Xt_planned)
            // Instead of faking the Current X, we fake the Target Xt
            std::vector<double> Xt_planned = Xt; 
            
            Xt_planned[0] = pos_plan[0];
            Xt_planned[1] = pos_plan[1];
            Xt_planned[2] = pos_plan[2];
            
            // (Orientation in Xt_planned  the same as original Xt)

            // 3. Call controller with Real X and Planned Target
            cartesianimpedancecontrol(
                traj_msg, torque_msg, q_current, gravity_torques_,
                X, Xt_planned,  // Pass PLANNED target pose
                jacobian, CartAdmMode, current_wrench_,measured_joint_torques, q_target, num_joints_
            );

        }

        void cartesianAdmittanceControl(
            std::vector<double>&        traj_msg,
            std::vector<double>&        torque_msg,
            Eigen::VectorXd&            q_current,
            const Eigen::VectorXd&      gravity_torques_,
            const std::vector<double>&  X,
            const std::vector<double>&  Xt,
            const Eigen::MatrixXd&      jacobian,
            int                         CartAdmMode,      // 1 = pure admittance, 2 = haptic
            const std::vector<double>&  current_wrench_,
            const std::vector<double>&  measured_joint_torques,
            Eigen::VectorXd&            q_target,
            const Eigen::VectorXd&      ee_velocity_admittance_,
            size_t                      num_joints_,
            bool                        gravity_compensated = false
        ){
            const bool have_wrench = (current_wrench_.size() >= 3);
            const bool have_torque = (current_wrench_.size() >= 6);

            // Persistent state — 6 axes: 0..2 translation (m), 3..5 rotation (rad)
            static std::vector<double> last_adm_delta(6, 0.0);
            static std::vector<double> offset(6, 0.0);
            static int last_mode = -1;

            // Cartesian per-axis position bound on the AdmittanceController's
            // internal pose_. The same admCtrl(0..5) instances are shared with
            // joint admittance, where joint mode 1 sets these bounds to joint
            // angle limits (radians). Without resetting them here, Cartesian
            // mode 1 inherits radian-scale clamps applied to a meter-scale
            // position offset → effectively unbounded drift.
            constexpr double kPoseLimitMode1 =  0.30;  // ±30 cm accumulated push (workspace-safe, kinesthetically usable)
            constexpr double kPoseLimitMode2 =  0.05;  // ±5 cm haptic spring
            constexpr double kRotLimitMode1  =  M_PI / 6.0;   // ±30° accumulated rotation
            constexpr double kRotLimitMode2  =  M_PI / 36.0;  // ±5° haptic rotation

            // External anti-windup bound (applied to Xt directly, after offset add)
            constexpr double kWorkspaceLo = -10.0;
            constexpr double kWorkspaceHi =  10.0;

            // ---- Reference wrench: the desired wrench (setDesiredWrench, zero by default). The
            // measured wrench is the external wrench, gravity already removed (the Jacobian estimate
            // pinv(J^T)(-(tau_meas + g)) of both nodes, Franka's O_F_ext_hat_K, the Kortex tool
            // wrench), so at rest with no contact and a zero desired wrench the interaction is
            // -measured. A gravity force J# g as reference (as before for drivers without
            // gravity_compensated) was never matched by any of these sources: the offset ran with
            // the arm's weight (33 N up at the Kinova home pose).
            (void)gravity_compensated;
            const std::vector<double>& F_desired = desired_wrench_;

            // ---- Mode entry: clear push accumulator, delta memory, AND the
            // shared AdmittanceController state so leftover pose/velocity from
            // a prior joint-admittance session can't bias the integrator.
            const bool mode_changed = (CartAdmMode != last_mode);
            if (mode_changed) {
                std::fill(offset.begin(), offset.end(), 0.0);
                std::fill(last_adm_delta.begin(), last_adm_delta.end(), 0.0);
                for (int ax = 0; ax < 6; ++ax) this->admCtrl(ax).resetState();
                last_mode = CartAdmMode;
            }

            const double pose_limit =
                (CartAdmMode == 1) ? kPoseLimitMode1 : kPoseLimitMode2;
            const double rot_limit =
                (CartAdmMode == 1) ? kRotLimitMode1  : kRotLimitMode2;
            for (int ax = 0; ax < 3; ++ax) {
                this->admCtrl(ax    ).setMinMax_Pose(-pose_limit, pose_limit);
                this->admCtrl(ax + 3).setMinMax_Pose(-rot_limit,  rot_limit);
            }

            std::vector<double> Xt_adm = Xt;   // orientation axes 3..5 pass through
            
            for (int ax = 0; ax < 3; ++ax) {
                auto& a = this->admCtrl(ax);

                // 1. Isolate external force (joint analogue: DesiredEffort − current_effort_)
                const double desired_force  = F_desired[ax];
                const double measured_force = have_wrench ? current_wrench_[ax] : 0.0;
                double ext_force = desired_force - measured_force;

                // 2. Deadband — needs to ride out residual wrench-estimate bias.
                //    libfranka's O_F_ext_hat_K can carry several N of bias when
                //    a tool isn't registered via setLoad(); 1.5 N rode partway.
                if (std::abs(ext_force) < deadband_cart_force_) ext_force = 0.0;

                // 3. Feed the admittance block. Both modes are driven by the interaction force
                //    (desired - measured) alone, as the joint admittance: the desired force in the
                //    block's own desired effort as well would count it twice. M2 differs from M1
                //    in its bound only (beyond it the Cartesian law resists).
                a.setDesiredEffort(0.0);
                a.setCurrentEffort(-ext_force);

                // 4. Absolute delta
                double adm_delta = this->admittance(ax, dt);

                // 5. Track delta → step
                double step_size = adm_delta - last_adm_delta[ax];
                last_adm_delta[ax] = adm_delta;

                // 6. Apply with anti-windup
                if (CartAdmMode == 1) {
                    // MODE 1: Push-and-Stay — accumulate step into offset
                    offset[ax] += step_size;
                    double Xt_cart = Xt[ax] + offset[ax];

                    // 7. Clamp to workspace + anti-windup (mirrors joint-limit clamp)
                    const double before_clamp = Xt_cart;
                    Xt_cart = std::clamp(Xt_cart, kWorkspaceLo, kWorkspaceHi);
                    if (Xt_cart != before_clamp) {
                        offset[ax] -= step_size;                   // refuse windup against wall
                    }
                    Xt_adm[ax] = Xt_cart;
                } else {
                    // MODE 2: Haptic — live delta, no accumulation
                    offset[ax] = 0.0;
                    double Xt_cart = Xt[ax] + adm_delta;
                    Xt_cart = std::clamp(Xt_cart, kWorkspaceLo, kWorkspaceHi);
                    Xt_adm[ax] = Xt_cart;
                }
            }

            // ---- Orientation admittance (axes 3..5): torques about EE x/y/z ----
            // Mirrors the position loop but runs on the wrench torque triple.
            // Result is an axis-angle rotation in the base frame composed with
            // the desired quaternion (left multiply). Skipped silently when the
            // wrench message has no torque components (e.g. force-only sensors).
            Eigen::Vector3d orient_axis_angle = Eigen::Vector3d::Zero();
            if (have_torque) {
                for (int ax = 0; ax < 3; ++ax) {
                    const int idx = 3 + ax;
                    auto& a = this->admCtrl(idx);

                    // The desired moment is the user's (zero by default): gravity
                    // compensation is purely translational and adds none.
                    const double measured_torque = current_wrench_[idx];
                    double ext_torque = F_desired[idx] - measured_torque;

                    // Deadband — torque-estimate noise floor on Franka's
                    // tau_ext_hat_K is ~0.05–0.15 Nm; static bias from an
                    // unregistered tool's gravity adds mass × moment-arm
                    // (~0.25 Nm for a 0.5 kg gripper at 5 cm). 0.2 Nm rides
                    // the noise without dulling deliberate hand torque
                    // (typical wrist push is 0.1–0.3 Nm). Heavier tools need
                    // libfranka setLoad() to avoid drift; widen this if you
                    // can't / won't.
                    if (std::abs(ext_torque) < deadband_cart_torque_) ext_torque = 0.0;

                    a.setDesiredEffort(0.0);
                    a.setCurrentEffort(-ext_torque);

                    double adm_delta = this->admittance(idx, dt);
                    double step_size = adm_delta - last_adm_delta[idx];
                    last_adm_delta[idx] = adm_delta;

                    if (CartAdmMode == 1) {
                        offset[idx] += step_size;
                        // Anti-windup against the rad limit (mirrors translation
                        // workspace clamp). Clamp before reading so the offset
                        // value visible to the consumer is the bounded one.
                        const double before = offset[idx];
                        offset[idx] = std::clamp(offset[idx], -rot_limit, rot_limit);
                        if (offset[idx] != before) {
                            // Rolled back at the limit — drop this step's
                            // contribution to keep the integrator from winding.
                            offset[idx] = before - step_size;
                        }
                        orient_axis_angle[ax] = offset[idx];
                    } else {
                        offset[idx] = 0.0;
                        orient_axis_angle[ax] = adm_delta;
                    }
                }
            }

            // Compose the per-axis rotation into the desired quaternion.
            // Xt[3..6] = (qx, qy, qz, qw); apply the admittance rotation in
            // base frame so the response feels intuitive: pushing on the EE
            // rotates the target the same way the user pushed.
            Eigen::Quaterniond q_des(Xt[6], Xt[3], Xt[4], Xt[5]);
            q_des.normalize();
            const double rot_angle = orient_axis_angle.norm();
            Eigen::Quaterniond q_des_new = q_des;
            if (rot_angle > 1e-9) {
                Eigen::Quaterniond q_delta(
                    Eigen::AngleAxisd(rot_angle, orient_axis_angle / rot_angle));
                q_des_new = q_delta * q_des;
                q_des_new.normalize();
            }
            Xt_adm[3] = q_des_new.x();
            Xt_adm[4] = q_des_new.y();
            Xt_adm[5] = q_des_new.z();
            Xt_adm[6] = q_des_new.w();

            // 8. Diagnostic block (16 doubles at base = num_joints_ * 8)
            const size_t base = num_joints_ * 8;
            if (traj_msg.size() < base + 16) traj_msg.resize(base + 16, 0.0);
            traj_msg[base + 0] = Xt_adm[0];
            traj_msg[base + 1] = Xt_adm[1];
            traj_msg[base + 2] = Xt_adm[2];

            // 9. Admittance target → Planner → Impedance
            cartesianPlanner(traj_msg, torque_msg, q_current, gravity_torques_,
                            X, Xt_adm, jacobian, CartAdmMode,
                            current_wrench_, measured_joint_torques, q_target, num_joints_);
        }
        // Hapfic on host modes 7/8: the task-space admittance of cartesianAdmittanceControl,
        // step for step (drive = desired - measured wrench, deadbands, M1 push-and-stay
        // within ±0.30 m / ±pi/6, M2 live within ±0.05 m / ±pi/36), on Hapfic's own integrators
        // and state. Returns the goal Xt shifted by it.
        std::vector<double> hapficTaskAdmittance(const std::vector<double>& Xt,
                                                 const std::vector<double>& current_wrench_,
                                                 int CartAdmMode) {
            const int off = hapfic_cart_planner_offset_;
            if (CartAdmMode != hapfic_cadm_last_mode_) resetHapficTaskAdmittance(CartAdmMode);
            const double pose_limit = (CartAdmMode == 1) ? 0.30 : 0.05;
            const double rot_limit  = (CartAdmMode == 1) ? M_PI / 6.0 : M_PI / 36.0;
            for (int ax = 0; ax < 3; ++ax) {
                this->admCtrl(off + ax    ).setMinMax_Pose(-pose_limit, pose_limit);
                this->admCtrl(off + ax + 3).setMinMax_Pose(-rot_limit,  rot_limit);
            }
            const bool have_wrench = (current_wrench_.size() >= 3);
            const bool have_torque = (current_wrench_.size() >= 6);

            std::vector<double> Xt_adm = Xt;
            for (int ax = 0; ax < 3; ++ax) {
                auto& a = this->admCtrl(off + ax);
                double ext_force = desired_wrench_[ax] - (have_wrench ? current_wrench_[ax] : 0.0);
                if (std::abs(ext_force) < deadband_cart_force_) ext_force = 0.0;
                a.setDesiredEffort(0.0);
                a.setCurrentEffort(-ext_force);
                const double adm_delta = this->admittance(off + ax, dt);
                const double step_size = adm_delta - hapfic_cadm_last_delta_[ax];
                hapfic_cadm_last_delta_[ax] = adm_delta;
                if (CartAdmMode == 1) {
                    hapfic_cadm_offset_[ax] += step_size;
                    Xt_adm[ax] = Xt[ax] + hapfic_cadm_offset_[ax];
                } else {
                    hapfic_cadm_offset_[ax] = 0.0;
                    Xt_adm[ax] = Xt[ax] + adm_delta;
                }
            }
            Eigen::Vector3d orient_axis_angle = Eigen::Vector3d::Zero();
            if (have_torque) {
                for (int ax = 0; ax < 3; ++ax) {
                    const int idx = 3 + ax;
                    auto& a = this->admCtrl(off + idx);
                    double ext_torque = desired_wrench_[idx] - current_wrench_[idx];
                    if (std::abs(ext_torque) < deadband_cart_torque_) ext_torque = 0.0;
                    a.setDesiredEffort(0.0);
                    a.setCurrentEffort(-ext_torque);
                    const double adm_delta = this->admittance(off + idx, dt);
                    const double step_size = adm_delta - hapfic_cadm_last_delta_[idx];
                    hapfic_cadm_last_delta_[idx] = adm_delta;
                    if (CartAdmMode == 1) {
                        hapfic_cadm_offset_[idx] += step_size;
                        const double before = hapfic_cadm_offset_[idx];
                        hapfic_cadm_offset_[idx] = std::clamp(before, -rot_limit, rot_limit);
                        if (hapfic_cadm_offset_[idx] != before) hapfic_cadm_offset_[idx] = before - step_size;
                        orient_axis_angle[ax] = hapfic_cadm_offset_[idx];
                    } else {
                        hapfic_cadm_offset_[idx] = 0.0;
                        orient_axis_angle[ax] = adm_delta;
                    }
                }
            }
            Eigen::Quaterniond q_des(Xt[6], Xt[3], Xt[4], Xt[5]);
            q_des.normalize();
            const double rot_angle = orient_axis_angle.norm();
            if (rot_angle > 1e-9) {
                q_des = Eigen::Quaterniond(Eigen::AngleAxisd(rot_angle, orient_axis_angle / rot_angle)) * q_des;
                q_des.normalize();
            }
            Xt_adm[3] = q_des.x(); Xt_adm[4] = q_des.y(); Xt_adm[5] = q_des.z(); Xt_adm[6] = q_des.w();
            return Xt_adm;
        }

        void cartesianHapfic(
            std::vector<double>&        traj_msg,
            std::vector<double>&        torque_msg,
            Eigen::VectorXd&            q_current,
            const Eigen::VectorXd&      gravity_torques_,
            const std::vector<double>&  X,
            const std::vector<double>&  Xt,
            const Eigen::MatrixXd&      jacobian,
            int                         CartAdmMode,
            const std::vector<double>&  current_wrench_,
            const std::vector<double>&  measured_joint_torques,
            Eigen::VectorXd&            q_target,
            size_t                      num_joints_
        ) {
            // Host modes 7/8: the task-space admittance shifts the task layer's goal (mode 6: none).
            std::vector<double> Xt_task = Xt;
            if (CartAdmMode == 1 || CartAdmMode == 2) {
                Xt_task = hapficTaskAdmittance(Xt, current_wrench_, CartAdmMode);
            } else if (hapfic_cadm_last_mode_ != 0) {
                resetHapficTaskAdmittance(0);
            }

            Eigen::Vector3d curr_pos(X[0], X[1], X[2]);
            Eigen::Vector3d des_pos(Xt_task[0], Xt_task[1], Xt_task[2]);

            Eigen::Vector3d pos_plan;
            for (int ax = 0; ax < 3; ++ax)
                pos_plan[ax] = this->plan(hapfic_cart_planner_offset_ + ax, des_pos[ax], dt);

            Eigen::Matrix<double,6,1> wrench_cmd;
            for (int ax = 0; ax < 3; ++ax)
                wrench_cmd[ax] = this->control(hapfic_cart_planner_offset_ + ax, pos_plan[ax] - curr_pos[ax]);

            Eigen::Quaterniond q_curr(X[6], X[3], X[4], X[5]);
            Eigen::Quaterniond q_des(Xt_task[6], Xt_task[3], Xt_task[4], Xt_task[5]);
            q_curr.normalize(); q_des.normalize();

            // The IK set-point (the node's IK of the unshifted goal Xt) follows the shifted goal
            // to first order, q_shift = J# dx with J# = J^T (J J^T + lambda^2 I)^-1; zero in mode 6.
            const size_t nq = static_cast<size_t>(jacobian.cols());
            if (hapfic_q_shift_.size() != static_cast<Eigen::Index>(nq)) hapfic_q_shift_ = Eigen::VectorXd::Zero(nq);
            if (CartAdmMode == 1 || CartAdmMode == 2) {
                Eigen::Matrix<double,6,1> dx;
                dx.head<3>() = des_pos - Eigen::Vector3d(Xt[0], Xt[1], Xt[2]);
                Eigen::Quaterniond q_goal(Xt[6], Xt[3], Xt[4], Xt[5]);
                q_goal.normalize();
                const Eigen::AngleAxisd d_rot(q_des * q_goal.conjugate());
                dx.tail<3>() = d_rot.angle() * d_rot.axis();
                if (dx.norm() > 1e-12 && jacobian.rows() == 6) {
                    const Eigen::MatrixXd JJt = jacobian * jacobian.transpose()
                        + kHapficIkDamping * kHapficIkDamping * Eigen::MatrixXd::Identity(6, 6);
                    hapfic_q_shift_ = jacobian.transpose() * JJt.ldlt().solve(dx);
                } else {
                    hapfic_q_shift_.setZero();
                }
            } else {
                hapfic_q_shift_.setZero();
            }

            // NOTE (13 Oct 2026): same orientation law as cartesianimpedancecontrol, on channel 19;
            // under review there (q_s < 0 / double cover, proposed shortest-arc fix, test plan).
            // Not changed: change both copies together.
            Eigen::Quaterniond q_err = q_des * q_curr.conjugate();
            Eigen::Vector3d q_v = q_err.vec();
            double q_s = q_err.w();
            double vnorm = q_v.norm();
            double theta = 2.0 * std::atan(vnorm / std::max(q_s, 1e-12));

            double tau_cq = this->control(hapfic_cart_planner_offset_ + 3, theta);
            vnorm = std::max(vnorm, 1e-12);
            Eigen::Vector3d ori_torque = (q_s >= 0.0 ? 1.0 : -1.0) * tau_cq * (q_v / vnorm);
            wrench_cmd.tail<3>() = ori_torque;

            Eigen::VectorXd tau_des = jacobian.transpose() * wrench_cmd + gravity_torques_;

            if (hapfic_export_only_) hapfic_tau_des_ = tau_des;
            // measured_joint_torques arrives in the Cartesian-mode convention
            // (sim: -effort, hardware: joint_torques_), the negative of what
            // handleJointAdmitanceControl takes (joint admittance passes
            // sim +effort / hardware -joint_torques_), so flip it here.
            // The IK set-point of each joint is limited to the admittance bound (the joint law's
            // outer breakpoint) about the reached joint position, never narrower than
            // kSetpointWindowMin, on a copy: q_target (the node's IK solution of the goal) is kept.
            // Admittance drive (see hapfic_split_drive_): empty = each joint deadbands its own gap.
            Eigen::VectorXd drive;
            if (hapfic_split_drive_ && !hapfic_export_only_ && jacobian.rows() == 6 && nq == num_joints_) {
                Eigen::VectorXd u(num_joints_);
                for (size_t i = 0; i < num_joints_; ++i) u[i] = tau_des[i] + measured_joint_torques[i];
                const Eigen::MatrixXd JJt = jacobian * jacobian.transpose()
                    + kHapficIkDamping * kHapficIkDamping * Eigen::MatrixXd::Identity(6, 6);
                const Eigen::Matrix<double,6,1> w_gap = JJt.ldlt().solve(jacobian * u);   // pinv(J^T) u
                const Eigen::VectorXd u_null = u - jacobian.transpose() * w_gap;
                const double rel = kHapficDeadbandRelease;
                const double fn = w_gap.head<3>().norm(), tn = w_gap.tail<3>().norm();
                if (!hapfic_task_engaged_f_ && fn >= hapfic_task_db_force_) hapfic_task_engaged_f_ = true;
                else if (hapfic_task_engaged_f_ && fn < rel * hapfic_task_db_force_) hapfic_task_engaged_f_ = false;
                if (!hapfic_task_engaged_t_ && tn >= hapfic_task_db_torque_) hapfic_task_engaged_t_ = true;
                else if (hapfic_task_engaged_t_ && tn < rel * hapfic_task_db_torque_) hapfic_task_engaged_t_ = false;
                Eigen::Matrix<double,6,1> w_db = Eigen::Matrix<double,6,1>::Zero();
                if (hapfic_task_engaged_f_) w_db.head<3>() = w_gap.head<3>();
                if (hapfic_task_engaged_t_) w_db.tail<3>() = w_gap.tail<3>();
                drive = jacobian.transpose() * w_db;
                if (joint_adm_engaged_.size() < num_joints_) joint_adm_engaged_.resize(num_joints_, 0);
                for (size_t i = 0; i < num_joints_; ++i) {   // the joint deadband on the rest
                    char& on = joint_adm_engaged_[i];
                    if (!on && std::abs(u_null[i]) >= deadband_joint_tau_) on = 1;
                    else if (on && std::abs(u_null[i]) < rel * deadband_joint_tau_) on = 0;
                    if (on) drive[i] += u_null[i];
                }
            }
            for (size_t i = 0; i < num_joints_ && !hapfic_export_only_; ++i) {
                const Eigen::VectorXd xb = this->controller(i).get_x_b();
                const bool cont = i < jointIsContinuous_.size() && jointIsContinuous_[i];
                const double q_ik_i = q_target[i] + (i < nq ? hapfic_q_shift_[i] : 0.0);
                double q_t_i = limitSetpoint(q_ik_i, q_current[i], setpointWindow(xb(xb.size() - 1)), cont);
                handleJointAdmitanceControl(
                    i, traj_msg, torque_msg, q_t_i, q_current[i],
                    tau_des[i], -measured_joint_torques[i], 2, num_joints_, true,
                    kHapficDeadbandRelease * deadband_joint_tau_,
                    drive.size() ? drive[i] : std::numeric_limits<double>::quiet_NaN()
                );
            }

            const size_t base = num_joints_ * 8;
            if (traj_msg.size() < base + 16) traj_msg.resize(base + 16, 0.0);
            traj_msg[base + 0]  = des_pos.x();   // task goal after the task-space admittance (= Xt in mode 6)
            traj_msg[base + 1]  = des_pos.y();
            traj_msg[base + 2]  = des_pos.z();
            traj_msg[base + 3]  = curr_pos.x();
            traj_msg[base + 4]  = curr_pos.y();
            traj_msg[base + 5]  = curr_pos.z();
            traj_msg[base + 6]  = (pos_plan - curr_pos).x();
            traj_msg[base + 7]  = (pos_plan - curr_pos).y();
            traj_msg[base + 8]  = (pos_plan - curr_pos).z();
            traj_msg[base + 9]  = pos_plan.x();
            traj_msg[base + 10] = pos_plan.y();
            traj_msg[base + 11] = pos_plan.z();
            traj_msg[base + 12] = ori_torque.x();
            traj_msg[base + 13] = ori_torque.y();
            traj_msg[base + 14] = ori_torque.z();
            traj_msg[base + 15] = theta;
        }

    private:
      FIC_t fic_;
    };
} //namespace fic
#endif // FIC_CONTROL_H
