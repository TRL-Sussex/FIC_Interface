#ifndef FIC_CONTROL_H
#define FIC_CONTROL_H
// FIC controllers for a whole robot, on top of the per-channel blocks of FIC.cpp:
//
//   JC        joint space: impedance (mode 1), impedance with planner (2), admittance push-and-stay
//             (3) and haptic (4)
//   RC        task space: Cartesian impedance (5), with planner (6), Cartesian admittance (7, 8),
//             and H-PMP (a task layer on top of the joint admittance)
//   postural  a joint-space bias for the redundant joints in the task-space modes
//
// Channels: 0-5 the end-effector axes in the Cartesian modes (0-2 translation, 3 orientation),
// 6-8 the elbow, 9 onward the postural joints; joints use 0..n-1 in the joint modes; H-PMP's own
// task channels start at hpmp_cart_planner_offset_ (16).

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
    // The control period (s) every block steps with.
    inline double dt = 0.001;

    // One robot's FIC: the joint, task-space and postural controllers over one set of channels.
    // Joint limits, continuity and every piece of controller state belong to the instance, so
    // several robots in one process do not share them.
    template <typename T>
    class ficControl {
    public:
        using FIC_t = FIC<T>;

        //==================================================================================
        // Joint controller (JC): the joint modes, one joint per call
        //==================================================================================
        struct JointController {
            ficControl* p;

            // The bare FIC law on channel i: JC.FIC(i, error).
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

        //==================================================================================
        // Robot controller (RC): the task-space modes, the whole arm per call
        //==================================================================================
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

            void hpmp(std::vector<double>& traj, std::vector<double>& torq,
                        Eigen::VectorXd& q_curr, const Eigen::VectorXd& grav,
                        const std::vector<double>& X, const std::vector<double>& Xt,
                        const Eigen::MatrixXd& J, int mode, const std::vector<double>& wrench,
                        const std::vector<double>& measured_joint_torques,
                        Eigen::VectorXd& q_t, size_t n_j) {
                p->cartesianHpmp(traj, torq, q_curr, grav, X, Xt, J, mode, wrench, measured_joint_torques, q_t, n_j);
            }
        } RC;

        //==================================================================================
        // Postural controller: pulls the redundant joints toward a posture in the task modes
        //==================================================================================
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

        //==================================================================================
        // Robot settings and per-instance state
        //==================================================================================

        // Joint metadata from the robot config (setJointLimits, setJointContinuous), sized to the
        // robot's channels at construction (+-pi, not continuous, until set).
        std::vector<bool>   jointIsContinuous_;
        std::vector<double> jointLower_;
        std::vector<double> jointUpper_;

        // Admittance deadbands per robot (setDeadbands): efforts smaller than these are treated
        // as sensor noise and do not move the admittance.
        double deadband_joint_tau_  = 1.5;  // Nm — joint admittance torque deadband
        double deadband_cart_force_ = 4.0;  // N  — Cartesian translational force deadband
        double deadband_cart_torque_= 0.2;  // Nm — Cartesian rotational torque deadband

        void setDeadbands(double joint_tau, double cart_force, double cart_torque) {
            deadband_joint_tau_   = joint_tau;
            deadband_cart_force_  = cart_force;
            deadband_cart_torque_ = cart_torque;
        }

        // Desired wrench of the task-space admittance (modes 7/8, and H-PMP on host modes 7/8):
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

        // How far a push may move the task-space goal under Cartesian admittance (modes 7 and 8,
        // and H-PMP on host modes 7 and 8), per robot (robots_config.yaml `admittance_limits`):
        // push-and-stay (mode 7) and haptic (mode 8), translation per axis (m) and rotation
        // about each axis (rad). In H-PMP's push-and-stay the rotation bound is the profile's
        // orientation admittance pose bound when it has one (initHpmpCartesian).
        double cart_adm_push_m_     = 0.30;          // 30 cm: room to place the tool by hand
        double cart_adm_push_rad_   = M_PI / 6.0;    // 30 deg
        double cart_adm_haptic_m_   = 0.05;          // 5 cm of spring
        double cart_adm_haptic_rad_ = M_PI / 36.0;   // 5 deg

        // Sets the four bounds; a value that is not a positive finite number keeps the one before.
        void setCartesianAdmittanceLimits(double push_m, double push_rad,
                                          double haptic_m, double haptic_rad) {
            auto take = [](double v, double& into) { if (std::isfinite(v) && v > 0.0) into = v; };
            take(push_m, cart_adm_push_m_);
            take(push_rad, cart_adm_push_rad_);
            take(haptic_m, cart_adm_haptic_m_);
            take(haptic_rad, cart_adm_haptic_rad_);
        }

        // The robot's FIC law options (robots_config.yaml `fic_law`), on every channel: restart
        // the convergence when the error stalls (FIC.cpp Controller), and the planners' cap on
        // their natural frequency (Hz) and speed margin. A planner value that is not a positive
        // finite number keeps the one before. The channels keep these through a mode change.
        void setLawOptions(bool restart_on_stall, double planner_max_hz, double planner_speed_margin) {
            restart_on_stall_ = restart_on_stall;
            for (size_t i = 0; i < fic_.channels(); ++i)
                fic_.getPlanner(i).setShape(T(planner_max_hz), T(planner_speed_margin));
            applyRestart();
        }

        // The restart is for tracking a goal (modes 1, 2, 5, 6). Where an admittance moves the goal
        // it stays off: the admittance is driven by the measured torque, which carries the law's own
        // effort, so a restart's step would read as a push (in the simulator, a 20 N push in mode 3
        // then moved the arm 0.06 rad instead of 0.6). The admittance paths below turn it off while
        // they run, and the nodes turn it off for the whole tick in those modes (setRestartAllowed),
        // which covers the elbow and postural terms run alongside.
        bool restart_on_stall_ = true;
        bool restart_allowed_ = true;
        static bool restartAllowedInMode(int mode, bool hpmp_active) {
            return !hpmp_active && mode != 3 && mode != 4 && mode != 7 && mode != 8;
        }
        void setRestartAllowed(bool allowed) {
            if (allowed == restart_allowed_) return;
            restart_allowed_ = allowed;
            applyRestart();
        }
        void applyRestart() {
            for (size_t i = 0; i < fic_.channels(); ++i)
                fic_.getController(i).setRestartOnStall(restart_on_stall_ && restart_allowed_);
        }
        // Off for one admittance path, then back as it was.
        struct NoRestart {
            ficControl* p; bool was;
            explicit NoRestart(ficControl* q) : p(q), was(q->restart_allowed_) { p->setRestartAllowed(false); }
            ~NoRestart() { p->setRestartAllowed(was); }
            NoRestart(const NoRestart&) = delete;
            NoRestart& operator=(const NoRestart&) = delete;
        };

        // The robot's max joint speed (rad/s; 0: none), the joint planners' speed cap: a planned
        // joint move never goes faster than the velocity safety allows. Channels are shared with
        // the Cartesian planners (metres), so the cap follows initJointPlanner, and
        // initCartesianPlanner clears it. A change applies at once to the joint planners running.
        double joint_speed_limit_ = 0.0;
        std::vector<char> joint_planner_ = {};   // channel i's planner was last set up for a joint

        void setJointSpeedLimit(double v) {
            joint_speed_limit_ = (std::isfinite(v) && v > 0.0) ? v : 0.0;
            for (size_t i = 0; i < joint_planner_.size(); ++i)
                if (joint_planner_[i]) this->planner(i).setSpeedLimit(T(joint_speed_limit_));
        }

        // Cartesian admittance state (cartesianAdmittanceControl), 6 axes: 0-2 translation (m),
        // 3-5 rotation (rad). It was function-local static state before 18 Oct 2026, shared by
        // every instance (two robots in one process pushed each other's goals).
        std::vector<double> cart_adm_offset_     = std::vector<double>(6, 0.0);
        std::vector<double> cart_adm_last_delta_ = std::vector<double>(6, 0.0);
        int                 cart_adm_last_mode_  = -1;

        // H-PMP's admittance drive, split (14 Oct 2026). The joint torque gap u = tau_des - tau_meas is
        // split into the part that produces an end-effector wrench, u_task = J^T w_gap with
        // w_gap = pinv(J^T) u, which passes a deadband in task space on w_gap (force and torque
        // magnitudes, with hysteresis: engaged at the deadband, released below kHpmpDeadbandRelease
        // of it), and the rest, u_null = u - u_task, which passes the joint deadband per joint. Without
        // the deadbands the recombined drive is u, as before. With the joint deadband alone a held
        // contact kept up to b on every joint, which the arm's lever arms turn into several newtons at the
        // end-effector (Kinova, b = 2 N m: 1.7-5.2 N in simulation); with the split it ends within the
        // task release level of the task-layer law, whatever the lever arms. Host path only: with
        // hpmp_export_only_ (the inner-loop MCU runs the joint layer) the per-joint deadband is kept.
        bool   hpmp_split_drive_    = true;
        double hpmp_task_db_force_  = 1.0;    // N, engage (release 0.25 N)
        double hpmp_task_db_torque_ = 0.1;    // N m, engage (release 0.025 N m)
        bool   hpmp_task_engaged_f_ = false;
        bool   hpmp_task_engaged_t_ = false;

        void setHpmpSplitDrive(bool on, double force_db, double torque_db) {
            hpmp_split_drive_    = on;
            hpmp_task_db_force_  = force_db;
            hpmp_task_db_torque_ = torque_db;
            hpmp_task_engaged_f_ = hpmp_task_engaged_t_ = false;
        }

        // Joint admittance state, per instance (it was function-local static state, shared by
        // every instance, which mixed up offsets between robots in one process). Grown to the
        // robot's joint count on first use, in handleJointAdmitanceControl.
        std::vector<double> joint_adm_offset_;      // accumulated human push offset per joint (rad)
        std::vector<double> joint_adm_last_delta_;  // last raw admittance delta per joint (rad)
        std::vector<char>   joint_adm_engaged_;     // deadband hysteresis: yielding until the gap is below release

        // Accumulated per-joint "human push offset" (radians): how far the human
        // has pushed joint i away from the commanded target under joint admittance
        // (mode 1). Returns 0.0 for an out-of-range index.
        double humanOffset(size_t i) const {
            return (i < joint_adm_offset_.size()) ? joint_adm_offset_[i] : 0.0;
        }

        // Hpmp: Cartesian planner+impedance channels at a dedicated offset so they
        // don't collide with joint controllers at 0..n_j or postural at 9..9+n_j.
        int hpmp_cart_planner_offset_ = 16;

        // Inner-loop MCU: when set, cartesianHpmp stops after the Cartesian
        // layer and leaves tau_des = J^T*w + g in hpmp_tau_des_ instead of
        // running the joint admittance layer here — the MCU runs that layer.
        bool hpmp_export_only_ = false;
        Eigen::VectorXd hpmp_tau_des_;

        // Hpmp on host modes 7/8: the host's task-space admittance shifts the task layer's
        // goal. It runs on its own integrators (the admittance blocks of channels off+0..off+5,
        // off = hpmp_cart_planner_offset_; the joint layer uses 0..n_j-1) with its own state.
        // hpmp_q_shift_ is the first-order shift of the IK set-point, J# dx, that follows the
        // shifted goal; the MCU path adds it to the set-point it sends.
        std::vector<double> hpmp_cadm_offset_     = std::vector<double>(6, 0.0);
        std::vector<double> hpmp_cadm_last_delta_ = std::vector<double>(6, 0.0);
        int                 hpmp_cadm_last_mode_  = -1;
        // H-PMP's task-space admittance in push-and-stay (host mode 7, teaching): the rotation bound of its
        // goal, the profile's orientation admittance pose bound (initHpmpCartesian); 0 until a profile
        // gives one, and then the robot's push-and-stay rotation (cart_adm_push_rad_) applies.
        double              hpmp_teach_rot_limit_ = 0.0;
        Eigen::VectorXd     hpmp_q_shift_;
        static constexpr double kHpmpIkDamping = 0.01;   // DLS lambda of J#, as the elbow helper

        void resetHpmpTaskAdmittance(int mode) {
            std::fill(hpmp_cadm_offset_.begin(), hpmp_cadm_offset_.end(), 0.0);
            std::fill(hpmp_cadm_last_delta_.begin(), hpmp_cadm_last_delta_.end(), 0.0);
            for (int k = 0; k < 6; ++k) this->admCtrl(hpmp_cart_planner_offset_ + k).resetState();
            hpmp_cadm_last_mode_ = mode;
        }

        void initHpmpCartesian(const fic::CartesianAllParams& cart_params,
                                 const std::vector<double>& current_pose) {
            int off = hpmp_cart_planner_offset_;
            for (int i = 0; i < 3; ++i) {
                initCartesianController(off + i, cart_params.pos_axes[i]);
                initCartesianPlanner(off + i, current_pose[i], cart_params.pos_axes[i]);
            }
            initCartesianController(off + 3, cart_params.rot_axis);
            // Elbow channels after the H-PMP Cartesian ones (off+4 .. off+6), clear
            // of the joint channels 0..n_j-1 (default 6..8 collides with joint 6).
            initElbowController(cart_params, off + 4);
            // Task-space admittance (host modes 7/8) on the admittance blocks of off..off+5,
            // with the Cartesian layer's admittance parameters; its bounds are set per tick.
            for (int i = 0; i < 3; ++i) initCartesianAdmittance(off + i, cart_params.pos_axes[i]);
            for (int i = 3; i < 6; ++i) initCartesianAdmittance(off + i, cart_params.rot_axis);
            const auto& rp = cart_params.rot_axis.admittance.pose;
            if (!rp.empty()) {
                double lim = 0.0;
                for (double v : rp) lim = std::max(lim, std::abs(v));
                if (lim > 0.0) hpmp_teach_rot_limit_ = lim;
            }
            resetHpmpTaskAdmittance(-1);
            hpmp_task_engaged_f_ = hpmp_task_engaged_t_ = false;   // split drive's hysteresis
        }

        // N_s saturation points per channel (N_negSat of them negative when asymmetric), n_dof
        // channels, each with an admittance block when enable_adm.
        ficControl(T N_s, bool asymmetric = false, T N_negSat = T{}, size_t n_dof = 7, bool enable_adm = true)
            : JC{this}, RC{this}, postural{this},
              fic_(N_s, asymmetric, N_negSat, n_dof, enable_adm)
        {
            jointIsContinuous_.assign(n_dof, false);
            jointLower_.assign(n_dof, -M_PI);
            jointUpper_.assign(n_dof, M_PI);
        }

        // The blocks of channel i.
        auto& controller(size_t i) { return fic_.getController(i); }
        auto& planner(size_t i)    { return fic_.getPlanner(i); }
        auto& admCtrl(size_t i)    { return fic_.getAdmittanceController(i); }

        // H-PMP: a joint set-point no further than `bound` from where the joint is (wrapped for a
        // continuous joint). With the IK set-point inside the admittance bound, the offset can
        // always cancel it, so a blocked end-effector does not open the joint errors.
        // Smallest set-point window: a narrow free-space bound (+-0.02 rad) would otherwise hold the
        // set-point back in free motion (Kinova lemniscate, simulation: RMSE 26 mm against 9.7 mm).
        static constexpr double kSetpointWindowMin = 0.1;   // rad
        // H-PMP's deadband hysteresis: release at this fraction of the joint deadband.
        static constexpr double kHpmpDeadbandRelease = 0.25;
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

        //==================================================================================
        // Setting up a joint
        //==================================================================================

        // The FIC profile of joint i: breakpoints, saturation forces, band starts (0.9 if none
        // given) and stiffness. A symmetric profile lists the positive side only.

        void initJointController(int i, const fic::ScalarParams& s, const fic::ListParams& l) {
            auto& c = this->controller(i);

            int n_s = l.n_saturations;
            int n_neg = l.n_neg_sat;
            if (!l.asymmetric && !l.error_max.empty()) {
                n_s = static_cast<int>(l.error_max.size()) * 2;
                n_neg = static_cast<int>(l.error_max.size());
            }
            c.reconfigure(n_s, l.asymmetric, n_neg);

            c.set_x_b(Eigen::Map<const Eigen::VectorXd>(l.error_max.data(), static_cast<Eigen::Index>(l.error_max.size())));
            c.setE_max(Eigen::Map<const Eigen::VectorXd>(l.force_max.data(), static_cast<Eigen::Index>(l.force_max.size())));

            if (!l.saturation_ratios.empty()) {
                c.setS_Start(Eigen::Map<const Eigen::VectorXd>(l.saturation_ratios.data(), static_cast<Eigen::Index>(l.saturation_ratios.size())));
                c.setNumSaturations(static_cast<int>(l.saturation_ratios.size()));
            } else {
                c.setS_Start(Eigen::VectorXd::Constant(1, 0.9));
                c.setNumSaturations(1);
            }
            c.setStiffness(Eigen::VectorXd::Constant(2, s.stiffness));
        }

        // The planner of joint i, starting at rest at current_pos. Continuity and limits come from
        // the robot config (setJointContinuous / setJointLimits), not the per-mode XML.
        void initJointPlanner(int i, double current_pos, const fic::PlannerParams& p_vals) {
            auto& p = this->planner(i);
            p.resetState(current_pos);
            p.setStiffness(p_vals.Plannerstiffness);
            p.setVelDesired(p_vals.vel_desired);
            p.setAccMax(p_vals.acc_max);
            p.setDampingRatio(p_vals.damping_ratio);
            p.setFrequency(p_vals.frequency);
            p.setWrapped(jointIsContinuous_[i] ? 1 : 0);
            p.setPosLimits(jointLower_[i], jointUpper_[i]);
            p.setSpeedLimit(T(joint_speed_limit_));
            markPlanner(i, true);
        }

        void markPlanner(int i, bool joint) {
            if (i < 0) return;
            if (joint_planner_.size() <= static_cast<size_t>(i)) joint_planner_.resize(i + 1, 0);
            joint_planner_[i] = joint ? 1 : 0;
        }

        // Robot-level metadata: set once per robot load, before any mode switch, so the Cartesian
        // modes (which skip initJointPlanner) see it too.
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

        // The admittance of joint i (after initJointController: its bounds come from the profile).
        // The offset is bounded by the profile's outer breakpoints, [x_b(0), x_b(N_S-1)] =
        // [-em_max, +em_max] for a symmetric profile; the XML's own admittance pose bound is not
        // used. Mode 3 widens the bound to the joint's span when it runs (see
        // handleJointAdmitanceControl).
        void initJointAdmittance(int i, [[maybe_unused]] const fic::ListParams& l,
                                 [[maybe_unused]] const fic::PlannerParams& p_vals,
                                 const fic::AdmittanceParams& a_vals) {
            auto& a = this->admCtrl(i);
            auto& c = this->controller(i);

            a.setInvM_Kv(a_vals.InvM,a_vals.damping);
            a.setMinMax_dFlow(a_vals.dFlow[0], a_vals.dFlow[1]);
            a.setMinMax_Flow(a_vals.flow[0], a_vals.flow[1]);
            Eigen::VectorXd xb_vec = c.get_x_b();
            a.setMinMax_Pose(xb_vec(0), xb_vec(xb_vec.size() - 1));

            a.resetState();
            if (static_cast<size_t>(i) < joint_adm_engaged_.size()) joint_adm_engaged_[i] = 0;
        }


        //==================================================================================
        // Setting up a Cartesian axis (channels 0-5 in the Cartesian modes)
        //==================================================================================

        // The FIC profile of a translation or rotation channel.
        void initCartesianController(int idx, const fic::CartesianAxisParams& params) {
            auto& c = this->controller(idx);

            int n_s = params.n_saturations;
            int n_neg = params.n_neg_sat;
            if (!params.asymmetric && !params.error_max.empty()) {
                n_s = static_cast<int>(params.error_max.size()) * 2;
                n_neg = static_cast<int>(params.error_max.size());
            }
            c.reconfigure(n_s, params.asymmetric, n_neg);

            if (!params.error_max.empty() && !params.force_max.empty()) {
                c.set_x_b(Eigen::Map<const Eigen::VectorXd>(params.error_max.data(), static_cast<Eigen::Index>(params.error_max.size())));
                c.setE_max(Eigen::Map<const Eigen::VectorXd>(params.force_max.data(), static_cast<Eigen::Index>(params.force_max.size())));
            }

            if (!params.saturation_ratios.empty()) {
                c.setS_Start(Eigen::Map<const Eigen::VectorXd>(params.saturation_ratios.data(), static_cast<Eigen::Index>(params.saturation_ratios.size())));
                c.setNumSaturations(static_cast<int>(params.saturation_ratios.size()));
            }

            // Translational (N/m) or rotational (N m/rad) stiffness.
            c.setStiffness(Eigen::VectorXd::Constant(2, params.stiffness));
        }

        // The planner of a translation channel, starting at rest at current_pos. No wrap; the
        // +-10 m limits are wider than any workspace (the bound is the robot's reach).
        void initCartesianPlanner(int idx, double current_pos, const fic::CartesianAxisParams& params) {
            auto& p = this->planner(idx);
            p.resetState(current_pos);
            p.setStiffness(params.stiffness_P);
            p.setVelDesired(params.vel_desired);
            p.setAccMax(params.acc_max);
            p.setDampingRatio(params.damping_ratio);
            p.setFrequency(params.frequency);
            p.setWrapped(false);
            p.setPosLimits(-10.0, 10.0);
            p.setSpeedLimit(T(0));      // metres: the joint speed cap does not apply
            markPlanner(idx, false);
        }

        // The admittance of a Cartesian channel, as initJointAdmittance (after its controller).
        // The Cartesian admittance sets its own bounds each step (cart_adm_*).
        void initCartesianAdmittance(int idx, const fic::CartesianAxisParams& params) {
            auto& a = this->admCtrl(idx);
            auto& c = this->controller(idx);
            const auto& adm = params.admittance;
            a.setInvM_Kv(adm.InvM, adm.damping);
            a.setMinMax_dFlow(adm.dFlow[0], adm.dFlow[1]);
            a.setMinMax_Flow(adm.flow[0], adm.flow[1]);
            Eigen::VectorXd xb_vec = c.get_x_b();
            a.setMinMax_Pose(xb_vec(0), xb_vec(xb_vec.size() - 1));
            a.resetState();
        }
        //==================================================================================
        // Elbow: a position task for the elbow, kept out of the end-effector's way
        //==================================================================================

        // First of the 3 elbow channels (x, y, z); pass it to applyElbowControl. 6 in the Cartesian
        // modes (after the end-effector channels 0-5). H-PMP runs joint channels 0..n_j-1, which on
        // a 7-DoF arm include 6, so it moves the elbow past its own channels (initHpmpCartesian).
        int elbow_offset_ = 6;
        int elbowOffset() const { return elbow_offset_; }

        // The elbow's three channels use the end-effector's translation profiles.
        void initElbowController(const fic::CartesianAllParams& params, int start_idx = 6) {
            elbow_offset_ = start_idx;
            for (int i = 0; i < 3; ++i) {
                initCartesianController(start_idx + i, params.pos_axes[i]);
            }
        }

        // Damping of the damped least-squares pseudo-inverse J# = J^T (J J^T + lambda^2 I)^-1
        // (the elbow's null-space projector; H-PMP's IK shift uses kHpmpIkDamping, the same value).
        static constexpr double kElbowDlsDamping = 0.01;

        // Adds the elbow task to tau: an FIC force on the elbow's position error (X_curr and
        // X_targ give the elbow position first; orientation is ignored), mapped to joint torques
        // by the elbow Jacobian's translation rows, then projected into the null space of the
        // end-effector Jacobian J_prim, N = I - J# J, so it does not move the end-effector.
        void applyElbowControl(std::vector<double>& tau,
                       [[maybe_unused]] const Eigen::VectorXd& q_curr,
                       const Eigen::MatrixXd& J_prim,
                       const Eigen::MatrixXd& J_elb,
                       const std::vector<double>& X_curr,
                       const std::vector<double>& X_targ,
                       size_t n_j,
                       int ctrl_start_idx)
        {
            Eigen::Vector3d pc(X_curr[0], X_curr[1], X_curr[2]);
            Eigen::Vector3d pt(X_targ[0], X_targ[1], X_targ[2]);
            Eigen::Vector3d pos_err = pt - pc;

            Eigen::Vector3d f_cmd;
            for (int k = 0; k < 3; ++k) {
                f_cmd[k] = this->control(ctrl_start_idx + k, pos_err[k]);
            }
            Eigen::VectorXd tau_elb = J_elb.topRows(3).transpose() * f_cmd;

            double lambda = kElbowDlsDamping;
            Eigen::MatrixXd JJT = J_prim * J_prim.transpose();
            JJT.diagonal().array() += lambda * lambda;
            Eigen::MatrixXd J_prim_pinv = J_prim.transpose() * JJT.inverse();
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n_j, n_j);
            Eigen::MatrixXd NullSpace = I - (J_prim_pinv * J_prim);
            Eigen::VectorXd tau_null = NullSpace * tau_elb;

            for (size_t i = 0; i < n_j; ++i) {
                tau[i] += tau_null[i];
            }
        }

        inline double wrapToPi(double x) {
            return std::atan2(std::sin(x), std::cos(x));
        }

        //==================================================================================
        // Joint modes: one joint, one control step
        //==================================================================================
        // Every handler fills the joint's torque and its 8 trajectory_data blocks
        // (traj_msg[i + k*NumJoints]): 0 planner set-point, 1 position, 2 error, 3 target,
        // 4 effort, 5 admittance target, 6 desired effort, 7 measured effort.

        // Mode 1: the FIC law on the joint error. A continuous joint's position and error are
        // wrapped to [-pi, pi]; any other joint's target is held inside its limits.
        void handleJointImpedanceControl(
            size_t i,
            std::vector<double>& traj_msg,
            std::vector<double>& torque_msg,
            double q_target,
            double q_current,
            size_t NumJoints) {

            if (jointIsContinuous_[i] == 1) {
                q_current = wrapToPi(q_current);
            } else {
                q_target = std::clamp(q_target, jointLower_[i], jointUpper_[i]);
            }
            double error = q_target - q_current;
            if (jointIsContinuous_[i] == 1) {
                error = wrapToPi(error);
            }

            double controlEffort = this->control(i, error);

            // traj_msg is always exactly 8 blocks of NumJoints here (a longer one is cut back: the
            // Cartesian callers write their own block after this).
            traj_msg.resize(NumJoints*8, 0.0);
            if (torque_msg.size() < NumJoints){
                torque_msg.resize(NumJoints, 0.0);
            }
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
            // Mode 2: the planner moves the set-point toward q_target, then mode 1 on it.
            double plannerTargetPos = this->plan(i, q_target, dt);
            if (jointIsContinuous_[i] == 0) {
                plannerTargetPos = std::clamp(plannerTargetPos, jointLower_[i], jointUpper_[i]);
            }

            handleJointImpedanceControl(i, traj_msg, torque_msg, plannerTargetPos, q_current, NumJoints);

            if (traj_msg.size() < NumJoints*8)
                traj_msg.resize(NumJoints*8, 0.0);
            traj_msg[i + 0*NumJoints] = plannerTargetPos;
        }

        // Modes 3 and 4 (and H-PMP's joint layer): the joint yields to the torque a person or a
        // contact applies, and the impedance law tracks the shifted target.
        //   AdmMode_ 1: push-and-stay (mode 3): the offset accumulates and stays.
        //   AdmMode_ 2: haptic (mode 4): the offset springs back, bounded by the profile.
        // The drive is DesiredEffort - current_effort_ (the callers pass the measured torque in
        // the sign that makes this the interaction torque), outside the deadband.
        void handleJointAdmitanceControl(
            size_t i,
            std::vector<double>& traj_msg,
            std::vector<double>& torque_msg,
            double &q_target,
            double q_current,
            double DesiredEffort,
            double current_effort_,
            int AdmMode_,
            size_t NumJoints,
            bool use_planner = true,
            double release_tau = 0.0,  // > 0: deadband with hysteresis (H-PMP), see step 2
            double drive = std::numeric_limits<double>::quiet_NaN()   // set: the deadbanded drive,
            ) {                                                       // from the caller (H-PMP split)
            NoRestart no_restart(this);

            auto& a = this->admCtrl(i);

            // 1. The interaction torque.
            double ext_tau = DesiredEffort - current_effort_;

            // 2. Deadband to ignore sensor noise + residual gravity-model error.
            //    Franka's tau_ext_hat_filtered carries 1-2 Nm of bias in heavy
            //    poses if a tool isn't registered via setLoad() — too low a
            //    deadband makes the admittance integrate that bias and drift.
            //    With release_tau > 0 (H-PMP) the deadband has hysteresis: the joint starts
            //    yielding when the gap reaches the deadband and keeps yielding until it is below
            //    release_tau, so a held contact ends within release_tau of the desired torque, not
            //    anywhere inside the deadband; at rest a bias below the deadband is still ignored.
            if (!std::isnan(drive)) {
                ext_tau = drive;   // already deadbanded by the caller (cartesianHpmp, split drive)
            } else if (release_tau > 0.0) {
                if (joint_adm_engaged_.size() < NumJoints) joint_adm_engaged_.resize(NumJoints, 0);
                char& on = joint_adm_engaged_[i];
                if (!on && std::abs(ext_tau) >= deadband_joint_tau_) on = 1;
                else if (on && std::abs(ext_tau) < release_tau) on = 0;
                if (!on) ext_tau = 0.0;
            } else if (std::abs(ext_tau) < deadband_joint_tau_) {
                ext_tau = 0.0;
            }

            // 3. Into the admittance block. Both modes are driven
            //    by the interaction torque alone (desired minus measured, outside the
            //    deadband): with nothing pushing, the offset does not move. A desired
            //    effort in the drive as well would count it twice: the offset would creep
            //    with no push (gravity alone drives it) and, at a contact, rest where the
            //    joint applies twice the desired torque. Gravity is fed forward to the
            //    motors by the caller instead (modes 3/4 and H-PMP, both nodes).
            a.setDesiredEffort(0.0);
            a.setCurrentEffort(-ext_tau);
            // 4. The admittance offset after this step.
            double adm_delta = this->admittance(i, dt);

            // 5. This step's change of it.
            auto& last_adm_delta = this->joint_adm_last_delta_;
            if (last_adm_delta.size() < NumJoints) {
                last_adm_delta.resize(NumJoints, 0.0);
            }
            double step_size = adm_delta - last_adm_delta[i];
            last_adm_delta[i] = adm_delta;

            // 6. The person's offset, kept apart from the commanded target, so the impedance law
            //    stays stiff around a target that has moved where the joint was pushed.
            auto& offset = this->joint_adm_offset_;
            if (offset.size() < NumJoints) {
                offset.resize(NumJoints, 0.0);
            }

            double q_targetAdm;
            double current_stable_q_target = wrapToPi(q_target);   // the commanded target

            if (AdmMode_ == 1) {
                // Push-and-stay: the steps add up and stay. The admittance integrates a change of
                // angle from where the push began, so its bound is the joint's span (+-(upper -
                // lower)), not the joint's limits: a range that leaves out zero (Franka joints 4 and
                // 6) clamped the integrator's start to the nearest limit, and that jump became
                // an offset (0.15 and 0.42 rad of motion as mode 3 started; fixed 18 Oct 2026).
                // The limits themselves are applied to the target in step 7.
                const double span = jointUpper_[i] - jointLower_[i];
                a.setMinMax_Pose(-span, span);
                offset[i] += step_size;
                q_targetAdm = current_stable_q_target + offset[i];
            }
            else {
                // Haptic: the live offset, bounded by the profile; nothing accumulates.
                offset[i] = 0.0;
                q_targetAdm = current_stable_q_target + adm_delta;
            }

            // 7. The joint limits. At a limit, push-and-stay stops adding to the offset, so
            //    pushing on against the limit does not wind it up.
            if (jointIsContinuous_[i] == 1) {
                q_targetAdm = wrapToPi(q_targetAdm);
            } else {
                q_targetAdm = std::clamp(q_targetAdm, jointLower_[i], jointUpper_[i]);
                if (q_targetAdm == jointUpper_[i] || q_targetAdm == jointLower_[i]) {
                    if (AdmMode_ == 1) {
                        offset[i] -= step_size;
                    }
                }
            }

            // 8. The impedance law on the shifted target (through the planner if asked).
            if (use_planner) {
                handleJointImpedanceControlWithPlanner(i, traj_msg, torque_msg, q_targetAdm, q_current, NumJoints);
            } else {
                handleJointImpedanceControl(i, traj_msg, torque_msg, q_targetAdm, q_current, NumJoints);
            }

            if (traj_msg.size() < NumJoints*8) traj_msg.resize(NumJoints*8, 0.0);
            traj_msg[i + 5*NumJoints] = q_targetAdm;
            traj_msg[i + 6*NumJoints] = DesiredEffort;
            traj_msg[i + 7*NumJoints] = current_effort_;
        }

        //==================================================================================
        // Task-space modes
        //==================================================================================

        // Orientation error as axis, angle and rotation vector (quatErrorShortestArc).
        struct SO3Error {
            Eigen::Vector3d axis;     // unit axis ê
            double angle;             // θ in [0, π]
            Eigen::Vector3d rotvec;   // θ * ê  (axis–angle vector)
        };

        // Returns the shortest-arc rotation from q_curr to q_des.
        // NOTE (13 Oct 2026): not called anywhere yet. It is the candidate replacement for the
        // orientation law of cartesianimpedancecontrol and cartesianHpmp (see the note there):
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

        // Postural task: adds an FIC torque per joint toward q_target (on the postural channels),
        // biasing the redundant joints. Runs in every task-space mode, the admittance ones too.
        void applyJointPosturalCorrection(Eigen::VectorXd& q_current,
                                        Eigen::VectorXd& q_target,
                                        [[maybe_unused]] bool CartAdmMode,
                                        size_t num_joints_,
                                        int controller_offset,
                                        std::vector<double>& tau)
        {
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

        // Mode 5: the FIC law on the end-effector's position error (channels 0-2) and orientation
        // error (channel 3), as a wrench mapped to joint torques by J^T, plus gravity.
        void cartesianimpedancecontrol(
            std::vector<double>&        traj_msg,
            std::vector<double>&        torque_msg,
            Eigen::VectorXd&            q_current,        // joint positions
            const Eigen::VectorXd&      gravity_torques_, // gravity torques
            const std::vector<double>&  X,                // [x,y,z,qx,qy,qz,qw] end-effector pose
            const std::vector<double>&  Xt,               // [x,y,z,qx,qy,qz,qw] goal pose
            const Eigen::MatrixXd&      jacobian,         // 6 x n end-effector Jacobian
            [[maybe_unused]] int                         CartAdmMode,
            [[maybe_unused]] const std::vector<double>&  current_wrench_,
            [[maybe_unused]] const std::vector<double>&  measured_joint_torques,
            Eigen::VectorXd&            q_target,
            size_t                      num_joints_
        ){
            Eigen::Vector3d curr_pos(X[0], X[1], X[2]);
            Eigen::Vector3d des_pos (Xt[0], Xt[1], Xt[2]);

            Eigen::Quaterniond q_curr(X[6], X[3], X[4], X[5]);
            Eigen::Quaterniond q_des (Xt[6], Xt[3], Xt[4], Xt[5]);
            q_curr.normalize(); q_des.normalize();

            // --- Orientation (Eq. 4 of the FIC quaternion paper) ---
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
            // the profile's cap (Kinova orientation profile of modes 6-8 and H-PMP: 50 N m/rad,
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
            // shortest-path law); (2) sim A/B of modes 5-8 and H-PMP with the goal quaternion
            // sign flipped and with large orientation errors; (3) the robot. Change this law and
            // its copy in cartesianHpmp (channel 19) together.
            Eigen::Quaterniond q_err = q_des * q_curr.conjugate();
            Eigen::Vector3d q_v = q_err.vec();
            double q_s = q_err.w();
            double vnorm = q_v.norm();
            double theta = 2.0 * std::atan(vnorm / std::max(q_s, 1e-12));

            // The FIC law on the angle (channel 3), then about the error's axis:
            // tau = sign(q_s) * f(theta) * q_v / |q_v|.
            double tau_cq = this->control(3, theta);
            vnorm = std::max(vnorm, 1e-12);
            Eigen::Vector3d ori_torque = (q_s >= 0.0 ? 1.0 : -1.0) * tau_cq * (q_v / vnorm);

            // --- Position, and the wrench to joint torques ---
            Eigen::Matrix<double,6,1> wrench_cmd;
            for (int ax = 0; ax < 3; ++ax) {
                wrench_cmd[ax] = this->control(ax, des_pos[ax] - curr_pos[ax]);
            }
            wrench_cmd.tail<3>() = ori_torque;

            Eigen::VectorXd tau = jacobian.transpose() * wrench_cmd + gravity_torques_;

            // --- Outputs. In the task-space modes trajectory_data holds, per joint, 8 values
            //     (position, target, then zeros) and after them a 16-value task block:
            //     [0-2 goal after the admittance (mode 7/8), 3-5 position, 6-8 error, 9-11 goal,
            //      12-14 orientation torque, 15 orientation error angle]. ---
            traj_msg.assign(num_joints_ * 8, 0.0);
            torque_msg.assign(num_joints_, 0.0);

            for (size_t i = 0; i < num_joints_; ++i) {
                traj_msg[i*8 + 0] = q_current[i];
                traj_msg[i*8 + 1] = q_target[i];
                torque_msg[i]     = tau[i];
            }
            const size_t base = num_joints_ * 8;
            if (traj_msg.size() < base + 16) traj_msg.resize(base + 16, 0.0);

            traj_msg[base + 3]  = curr_pos.x();
            traj_msg[base + 4]  = curr_pos.y();
            traj_msg[base + 5]  = curr_pos.z();
            traj_msg[base + 6]  = (des_pos - curr_pos).x();
            traj_msg[base + 7]  = (des_pos - curr_pos).y();
            traj_msg[base + 8]  = (des_pos - curr_pos).z();
            traj_msg[base + 9]  = des_pos.x();
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
            // Mode 6: the planner moves the goal's position (channels 0-2); the orientation goes
            // straight to the law. Then mode 5 on the planned goal.
            Eigen::Vector3d des_pos (Xt[0], Xt[1], Xt[2]);
            Eigen::Vector3d pos_plan;
            for (int ax = 0; ax < 3; ++ax)
                pos_plan[ax] = this->plan(ax, des_pos[ax], dt);

            std::vector<double> Xt_planned = Xt;
            Xt_planned[0] = pos_plan[0];
            Xt_planned[1] = pos_plan[1];
            Xt_planned[2] = pos_plan[2];

            cartesianimpedancecontrol(
                traj_msg, torque_msg, q_current, gravity_torques_,
                X, Xt_planned,
                jacobian, CartAdmMode, current_wrench_, measured_joint_torques, q_target, num_joints_
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
            [[maybe_unused]] const Eigen::VectorXd&      ee_velocity_admittance_,
            size_t                      num_joints_,
            bool                        gravity_compensated = false
        ){
            // Modes 7 (CartAdmMode 1, push-and-stay) and 8 (2, haptic): the end-effector yields to
            // the wrench a person applies, by moving the goal; then mode 6 on the moved goal.
            NoRestart no_restart(this);
            const bool have_wrench = (current_wrench_.size() >= 3);
            const bool have_torque = (current_wrench_.size() >= 6);

            auto& last_adm_delta = cart_adm_last_delta_;
            auto& offset = cart_adm_offset_;
            int& last_mode = cart_adm_last_mode_;

            // The bounds of the admittance blocks 0-5, set here every step: the same blocks serve
            // the joint admittance, whose bounds are in radians of joint angle.
            const double kPoseLimitMode1 = cart_adm_push_m_;
            const double kPoseLimitMode2 = cart_adm_haptic_m_;
            const double kRotLimitMode1  = cart_adm_push_rad_;
            const double kRotLimitMode2  = cart_adm_haptic_rad_;

            // Anti-windup bound on the moved goal's position (wider than any workspace).
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

            // On entering a mode: clear the offsets, the step memory and the admittance blocks, so
            // nothing left from a joint admittance session biases the integrators.
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

            std::vector<double> Xt_adm = Xt;

            // ---- Translation (axes 0-2) ----
            for (int ax = 0; ax < 3; ++ax) {
                auto& a = this->admCtrl(ax);

                // 1. The interaction force (the joint admittance's DesiredEffort - current_effort_).
                const double desired_force  = F_desired[ax];
                const double measured_force = have_wrench ? current_wrench_[ax] : 0.0;
                double ext_force = desired_force - measured_force;

                // 2. Deadband: rides out the wrench estimate's bias (libfranka's O_F_ext_hat_K
                //    carries several N when a tool is not registered with setLoad(); 1.5 N was
                //    not enough).
                if (std::abs(ext_force) < deadband_cart_force_) ext_force = 0.0;

                // 3. Feed the admittance block. Both modes are driven by the interaction force
                //    (desired - measured) alone, as the joint admittance: the desired force in the
                //    block's own desired effort as well would count it twice. M2 differs from M1
                //    in its bound only (beyond it the Cartesian law resists).
                a.setDesiredEffort(0.0);
                a.setCurrentEffort(-ext_force);

                // 4. The offset after this step, and the step itself.
                double adm_delta = this->admittance(ax, dt);
                double step_size = adm_delta - last_adm_delta[ax];
                last_adm_delta[ax] = adm_delta;

                if (CartAdmMode == 1) {
                    // Push-and-stay: the steps add up and stay; at the workspace bound the step is
                    // taken back (no wind-up against it).
                    offset[ax] += step_size;
                    double Xt_cart = Xt[ax] + offset[ax];
                    const double before_clamp = Xt_cart;
                    Xt_cart = std::clamp(Xt_cart, kWorkspaceLo, kWorkspaceHi);
                    if (Xt_cart != before_clamp) {
                        offset[ax] -= step_size;
                    }
                    Xt_adm[ax] = Xt_cart;
                } else {
                    // Haptic: the live offset, nothing accumulates.
                    offset[ax] = 0.0;
                    double Xt_cart = Xt[ax] + adm_delta;
                    Xt_cart = std::clamp(Xt_cart, kWorkspaceLo, kWorkspaceHi);
                    Xt_adm[ax] = Xt_cart;
                }
            }

            // ---- Rotation (axes 3-5): the same on the moments about x, y and z. The result is a
            // rotation vector in the base frame, applied to the goal orientation (left multiply).
            // Skipped when the wrench has no moments (a force-only sensor). ----
            Eigen::Vector3d orient_axis_angle = Eigen::Vector3d::Zero();
            if (have_torque) {
                for (int ax = 0; ax < 3; ++ax) {
                    const int idx = 3 + ax;
                    auto& a = this->admCtrl(idx);

                    // The desired moment is the user's (zero by default).
                    const double measured_torque = current_wrench_[idx];
                    double ext_torque = F_desired[idx] - measured_torque;

                    // Deadband: Franka's tau_ext_hat_K has a noise floor of about 0.05-0.15 N m, and an
                    // unregistered tool adds its weight times its lever (about 0.25 N m for a 0.5 kg
                    // gripper at 5 cm). 0.2 N m rides the noise without dulling a deliberate wrist
                    // push (0.1-0.3 N m). Heavier tools need setLoad(), or a wider deadband.
                    if (std::abs(ext_torque) < deadband_cart_torque_) ext_torque = 0.0;

                    a.setDesiredEffort(0.0);
                    a.setCurrentEffort(-ext_torque);

                    double adm_delta = this->admittance(idx, dt);
                    double step_size = adm_delta - last_adm_delta[idx];
                    last_adm_delta[idx] = adm_delta;

                    if (CartAdmMode == 1) {
                        offset[idx] += step_size;
                        // At the rotation bound, this step is taken back (no wind-up).
                        const double before = offset[idx];
                        offset[idx] = std::clamp(offset[idx], -rot_limit, rot_limit);
                        if (offset[idx] != before) {
                            offset[idx] = before - step_size;
                        }
                        orient_axis_angle[ax] = offset[idx];
                    } else {
                        offset[idx] = 0.0;
                        orient_axis_angle[ax] = adm_delta;
                    }
                }
            }

            // The rotation applied to the goal in the base frame, so the goal turns the way the
            // end-effector was pushed. Xt[3..6] = (qx, qy, qz, qw).
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

            cartesianPlanner(traj_msg, torque_msg, q_current, gravity_torques_,
                            X, Xt_adm, jacobian, CartAdmMode,
                            current_wrench_, measured_joint_torques, q_target, num_joints_);

            // The moved goal, in the task block of trajectory_data (see cartesianimpedancecontrol).
            // Written after the law, which rebuilds trajectory_data: written before it (until 18 Oct
            // 2026) it was always sent as zeros.
            const size_t base = num_joints_ * 8;
            if (traj_msg.size() < base + 16) traj_msg.resize(base + 16, 0.0);
            traj_msg[base + 0] = Xt_adm[0];
            traj_msg[base + 1] = Xt_adm[1];
            traj_msg[base + 2] = Xt_adm[2];
        }
        // H-PMP on host modes 7/8: the task-space admittance of cartesianAdmittanceControl, step
        // for step (drive = desired - measured wrench, deadbands, push-and-stay within
        // cart_adm_push_m_ and the profile's rotation bound, haptic within cart_adm_haptic_m_ and
        // cart_adm_haptic_rad_), on H-PMP's own integrators and state. Returns the moved goal.
        std::vector<double> hpmpTaskAdmittance(const std::vector<double>& Xt,
                                                 const std::vector<double>& current_wrench_,
                                                 int CartAdmMode) {
            const int off = hpmp_cart_planner_offset_;
            if (CartAdmMode != hpmp_cadm_last_mode_) resetHpmpTaskAdmittance(CartAdmMode);
            const double pose_limit = (CartAdmMode == 1) ? cart_adm_push_m_ : cart_adm_haptic_m_;
            const double rot_limit  = (CartAdmMode == 1)
                ? (hpmp_teach_rot_limit_ > 0.0 ? hpmp_teach_rot_limit_ : cart_adm_push_rad_)
                : cart_adm_haptic_rad_;
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
                const double step_size = adm_delta - hpmp_cadm_last_delta_[ax];
                hpmp_cadm_last_delta_[ax] = adm_delta;
                if (CartAdmMode == 1) {
                    hpmp_cadm_offset_[ax] += step_size;
                    Xt_adm[ax] = Xt[ax] + hpmp_cadm_offset_[ax];
                } else {
                    hpmp_cadm_offset_[ax] = 0.0;
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
                    const double step_size = adm_delta - hpmp_cadm_last_delta_[idx];
                    hpmp_cadm_last_delta_[idx] = adm_delta;
                    if (CartAdmMode == 1) {
                        hpmp_cadm_offset_[idx] += step_size;
                        const double before = hpmp_cadm_offset_[idx];
                        hpmp_cadm_offset_[idx] = std::clamp(before, -rot_limit, rot_limit);
                        if (hpmp_cadm_offset_[idx] != before) hpmp_cadm_offset_[idx] = before - step_size;
                        orient_axis_angle[ax] = hpmp_cadm_offset_[idx];
                    } else {
                        hpmp_cadm_offset_[idx] = 0.0;
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

        // CartAdmMode value of cartesianHpmp for host mode 5 (Cartesian impedance): no planner.
        static constexpr int kHpmpHostImpedance = -1;

        void cartesianHpmp(
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
            NoRestart no_restart(this);
            // Host modes 7/8: the task-space admittance shifts the task layer's goal (mode 6: none).
            // Host mode 5 (CartAdmMode = kHpmpHostImpedance): no admittance and no planner, the
            // task layer's law acts on the goal itself, as Cartesian impedance does.
            std::vector<double> Xt_task = Xt;
            if (CartAdmMode == 1 || CartAdmMode == 2) {
                Xt_task = hpmpTaskAdmittance(Xt, current_wrench_, CartAdmMode);
            } else if (hpmp_cadm_last_mode_ != 0) {
                resetHpmpTaskAdmittance(0);
            }

            Eigen::Vector3d curr_pos(X[0], X[1], X[2]);
            Eigen::Vector3d des_pos(Xt_task[0], Xt_task[1], Xt_task[2]);

            Eigen::Vector3d pos_plan;
            for (int ax = 0; ax < 3; ++ax)
                pos_plan[ax] = (CartAdmMode == kHpmpHostImpedance)
                    ? des_pos[ax]
                    : this->plan(hpmp_cart_planner_offset_ + ax, des_pos[ax], dt);

            Eigen::Matrix<double,6,1> wrench_cmd;
            for (int ax = 0; ax < 3; ++ax)
                wrench_cmd[ax] = this->control(hpmp_cart_planner_offset_ + ax, pos_plan[ax] - curr_pos[ax]);

            Eigen::Quaterniond q_curr(X[6], X[3], X[4], X[5]);
            Eigen::Quaterniond q_des(Xt_task[6], Xt_task[3], Xt_task[4], Xt_task[5]);
            q_curr.normalize(); q_des.normalize();

            // The IK set-point (the node's IK of the unshifted goal Xt) follows the shifted goal
            // to first order, q_shift = J# dx with J# = J^T (J J^T + lambda^2 I)^-1; zero in mode 6.
            const size_t nq = static_cast<size_t>(jacobian.cols());
            if (hpmp_q_shift_.size() != static_cast<Eigen::Index>(nq)) hpmp_q_shift_ = Eigen::VectorXd::Zero(nq);
            if (CartAdmMode == 1 || CartAdmMode == 2) {
                Eigen::Matrix<double,6,1> dx;
                dx.head<3>() = des_pos - Eigen::Vector3d(Xt[0], Xt[1], Xt[2]);
                Eigen::Quaterniond q_goal(Xt[6], Xt[3], Xt[4], Xt[5]);
                q_goal.normalize();
                const Eigen::AngleAxisd d_rot(q_des * q_goal.conjugate());
                dx.tail<3>() = d_rot.angle() * d_rot.axis();
                if (dx.norm() > 1e-12 && jacobian.rows() == 6) {
                    const Eigen::MatrixXd JJt = jacobian * jacobian.transpose()
                        + kHpmpIkDamping * kHpmpIkDamping * Eigen::MatrixXd::Identity(6, 6);
                    hpmp_q_shift_ = jacobian.transpose() * JJt.ldlt().solve(dx);
                } else {
                    hpmp_q_shift_.setZero();
                }
            } else {
                hpmp_q_shift_.setZero();
            }

            // NOTE (13 Oct 2026): same orientation law as cartesianimpedancecontrol, on channel 19;
            // under review there (q_s < 0 / double cover, proposed shortest-arc fix, test plan).
            // Not changed: change both copies together.
            Eigen::Quaterniond q_err = q_des * q_curr.conjugate();
            Eigen::Vector3d q_v = q_err.vec();
            double q_s = q_err.w();
            double vnorm = q_v.norm();
            double theta = 2.0 * std::atan(vnorm / std::max(q_s, 1e-12));

            double tau_cq = this->control(hpmp_cart_planner_offset_ + 3, theta);
            vnorm = std::max(vnorm, 1e-12);
            Eigen::Vector3d ori_torque = (q_s >= 0.0 ? 1.0 : -1.0) * tau_cq * (q_v / vnorm);
            wrench_cmd.tail<3>() = ori_torque;

            Eigen::VectorXd tau_des = jacobian.transpose() * wrench_cmd + gravity_torques_;

            if (hpmp_export_only_) hpmp_tau_des_ = tau_des;
            // measured_joint_torques arrives in the Cartesian-mode convention
            // (sim: -effort, hardware: joint_torques_), the negative of what
            // handleJointAdmitanceControl takes (joint admittance passes
            // sim +effort / hardware -joint_torques_), so flip it here.
            // The IK set-point of each joint is limited to the admittance bound (the joint law's
            // outer breakpoint) about the reached joint position, never narrower than
            // kSetpointWindowMin, on a copy: q_target (the node's IK solution of the goal) is kept.
            // Admittance drive (see hpmp_split_drive_): empty = each joint deadbands its own gap.
            Eigen::VectorXd drive;
            if (hpmp_split_drive_ && !hpmp_export_only_ && jacobian.rows() == 6 && nq == num_joints_) {
                Eigen::VectorXd u(num_joints_);
                for (size_t i = 0; i < num_joints_; ++i) u[i] = tau_des[i] + measured_joint_torques[i];
                const Eigen::MatrixXd JJt = jacobian * jacobian.transpose()
                    + kHpmpIkDamping * kHpmpIkDamping * Eigen::MatrixXd::Identity(6, 6);
                const Eigen::Matrix<double,6,1> w_gap = JJt.ldlt().solve(jacobian * u);   // pinv(J^T) u
                const Eigen::VectorXd u_null = u - jacobian.transpose() * w_gap;
                const double rel = kHpmpDeadbandRelease;
                const double fn = w_gap.head<3>().norm(), tn = w_gap.tail<3>().norm();
                if (!hpmp_task_engaged_f_ && fn >= hpmp_task_db_force_) hpmp_task_engaged_f_ = true;
                else if (hpmp_task_engaged_f_ && fn < rel * hpmp_task_db_force_) hpmp_task_engaged_f_ = false;
                if (!hpmp_task_engaged_t_ && tn >= hpmp_task_db_torque_) hpmp_task_engaged_t_ = true;
                else if (hpmp_task_engaged_t_ && tn < rel * hpmp_task_db_torque_) hpmp_task_engaged_t_ = false;
                Eigen::Matrix<double,6,1> w_db = Eigen::Matrix<double,6,1>::Zero();
                if (hpmp_task_engaged_f_) w_db.head<3>() = w_gap.head<3>();
                if (hpmp_task_engaged_t_) w_db.tail<3>() = w_gap.tail<3>();
                drive = jacobian.transpose() * w_db;
                if (joint_adm_engaged_.size() < num_joints_) joint_adm_engaged_.resize(num_joints_, 0);
                for (size_t i = 0; i < num_joints_; ++i) {   // the joint deadband on the rest
                    char& on = joint_adm_engaged_[i];
                    if (!on && std::abs(u_null[i]) >= deadband_joint_tau_) on = 1;
                    else if (on && std::abs(u_null[i]) < rel * deadband_joint_tau_) on = 0;
                    if (on) drive[i] += u_null[i];
                }
            }
            for (size_t i = 0; i < num_joints_ && !hpmp_export_only_; ++i) {
                const Eigen::VectorXd xb = this->controller(i).get_x_b();
                const bool cont = i < jointIsContinuous_.size() && jointIsContinuous_[i];
                const double q_ik_i = q_target[i] + (i < nq ? hpmp_q_shift_[i] : 0.0);
                double q_t_i = limitSetpoint(q_ik_i, q_current[i], setpointWindow(xb(xb.size() - 1)), cont);
                handleJointAdmitanceControl(
                    i, traj_msg, torque_msg, q_t_i, q_current[i],
                    tau_des[i], -measured_joint_torques[i], 2, num_joints_, true,
                    kHpmpDeadbandRelease * deadband_joint_tau_,
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
