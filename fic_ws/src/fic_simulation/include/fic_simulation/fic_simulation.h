// fic_simulation.h — FIC Simulation Node declaration
#pragma once

#include "fic/joint_target_slew.h"
#include "fic/safety_zones.h"
#include <vector>
#include <cmath>
#include <limits>
#include <mutex>
#include <algorithm>
#include <string>
#include <iostream>
#include <stdexcept>
#include <filesystem>
#include <thread>
#include <future>

// Linear Algebra & Physics
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tinyxml2.h>

// ROS 2 Core
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float32.hpp>

// ROS 2 Messages
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/wrench.hpp"

// Kinematics solver (pluggable; pinocchio is the default backend)
#include <fic/kinematics_solver.hpp>
#include <fic/solvers/solver_factory.hpp>

// Custom Controller Headers
#include "communication/msg/jacobian.hpp"
#include "communication/msg/actuator_cmds.hpp"

#include <fic/fic_control.h>
#include <fic/butterworth_filter.hpp>
#include <fic/fic_param_loader.hpp>
#include <fic/robot_config_loader.hpp>

// ═══════════════════════════════════════════════════════════════════════════════
//  CONTROL MODE ENUM
// ═══════════════════════════════════════════════════════════════════════════════

enum class ControlMode {
    JOINT_IMPEDANCE = 1,
    JOINT_IMPEDANCE_PLANNER = 2,
    JOINT_ADMITTANCE_M1 = 3,
    JOINT_ADMITTANCE_M2 = 4,
    CARTESIAN_IMPEDANCE = 5,
    CARTESIAN_PLANNER = 6,
    CARTESIAN_ADMITTANCE_M1 = 7,
    CARTESIAN_ADMITTANCE_M2 = 8,
    CARTESIAN_IK_JOINT_IMPEDANCE = 9
};

// ═══════════════════════════════════════════════════════════════════════════════
//  FIC SIMULATION NODE
// ═══════════════════════════════════════════════════════════════════════════════

template <typename T>
class FicSimulationNode : public rclcpp::Node {
public:
    explicit FicSimulationNode(int num_joints);

private:
    // ── Core State ──
    size_t num_joints_;
    bool Adm_enable = true;
    int current_control_mode_;
    std::string selected_robot_;
    std::string active_profile_ = "default";
    T dt = static_cast<T>(0.001);
    std::mutex data_mutex_;

    // ── Robot Config ──
    fic::RobotConfigLoader robot_config_loader_;
    fic::RobotConfig current_robot_config_;
    std::string ee_frame_name_ = "ee_link";
    std::string elbow_frame_name_ = "ForeArm_Link";
    std::string joint_name_prefix_;
    std::vector<std::string> prefixed_joint_names_;
    std::string source_prefix_ = "simulation";

    // ── Kinematics Solver (null in degraded mode: joint-space only) ──
    std::unique_ptr<fic::KinematicsSolver> solver_;
    std::vector<std::string> relevant_joints;

    // ── Geometry / Eigen State ──
    geometry_msgs::msg::Pose ee_target, ee_current;
    geometry_msgs::msg::Pose elbow_current_pose_, elbow_target_pose_;
    Eigen::MatrixXd jacobian;
    Eigen::MatrixXd J_elbow_6D;
    Eigen::VectorXd q_current_, q_target, gravity_torques_, q_target_ik_, ee_velocity_admittance_;
    // As fic_hardware: the postural task pulls toward the IK solution, but
    // reached at no more than kPosturalMaxRate per joint -- near a
    // singularity the IK can jump round the null space, and pulling
    // straight at it swung the Kinova's joints 1 and 3 at 80-90 deg/s. Only
    // the control loop touches q_postural_; a mode switch or postural
    // enable asks for it to start again from the arm (postural_reset_).
    Eigen::VectorXd q_postural_;
    bool postural_reset_ = true;   // guarded by data_mutex_
    static constexpr double kPosturalMaxRate = 0.52;   // rad/s, ~30 deg/s
    std::chrono::steady_clock::time_point postural_at_{};   // last step
    Eigen::Vector3d target_pos, current_pos;
    std::vector<T> current_effort, currentVelocities_;
    // Unfiltered joint velocity, exactly as received on joint_states.
    //
    // `currentVelocities_` is the 10 Hz-filtered estimate and stays that way:
    // it feeds the Cartesian velocity used for state estimation and display,
    // where smoothness matters and a few tens of milliseconds of lag does not.
    // The *output damping* term in controlLoop() is a different consumer with
    // the opposite requirement. It realises kd*qdot, and a damper only removes
    // energy while its velocity signal is roughly in phase with the true
    // velocity. A 2nd-order Butterworth at 1.6x its corner already contributes
    // ~113 deg of lag and at 3.9x ~158 deg, so above the corner the term stops
    // damping and starts pumping. Spot's hip-abduction joints resonate at
    // ~15.9 Hz and its unloaded knees at ~38.5 Hz - both above the 10 Hz corner
    // - which is why they were the joints that always diverged first. Damping
    // needs phase fidelity far more than it needs smoothness, so it is fed from
    // here instead. Position and torque filtering are unchanged.
    std::vector<T> rawVelocities_;
    // The velocity the damping term actually uses: rawVelocities_ through a
    // dedicated FIRST-ORDER lag (see kDampingCutoffHz).
    std::vector<T> dampingVelocities_;

    // ── Filters ──
    // Shared by the constructor and by the first-sample priming in jointStateCallback.
    //
    // The state/torque filters are stepped once per incoming joint_states
    // message, so their sample rate is the *feedback publish rate*, not the
    // control-loop rate. A Butterworth designed for a sample rate it is not
    // actually stepped at has its cutoff scaled by (real rate / declared rate):
    // declaring 1000 Hz while being fed at ~137 Hz turns a 10 Hz cutoff into an
    // effective 1.4 Hz one, i.e. ~160 ms of phase lag on the position the
    // impedance law reacts to. The real rate is not a constant either: it is
    // whatever the publisher achieves and this node manages to consume — the
    // 200 Hz nominal of mujoco_native measures 137-175 Hz here depending on
    // joint count, and a hardware driver picks its own. So the rate is measured
    // from the stream rather than hard-coded to any number.
    static constexpr double kTorqueCutoffHz = 10.0;
    // Measured-torque filter cutoff: the robot's hardware_model torque_filter_hz when it has one
    // (fic_hardware's, 200 Hz on the Kinova), else kTorqueCutoffHz.
    double torqueCutoffHz() const {
        const auto& hm = current_robot_config_.hardware_model;
        return (hm.enabled && hm.torque_filter_hz > 0.0) ? hm.torque_filter_hz : kTorqueCutoffHz;
    }
    double torque_cutoff_applied_hz_ = kTorqueCutoffHz;
    static constexpr double kStateCutoffHz  = 10.0;
    // Fallback used until the rate has been measured, and permanently if it
    // cannot be established (implausible or pathologically jittery stream).
    // 1000 Hz is the nominal control-loop rate this node was written against,
    // so the fallback reproduces the historical behaviour exactly.
    static constexpr double kFilterRateFallbackHz = 1000.0;
    // Plausible-range guard. Below 4x the 10 Hz cutoff the filter is not
    // meaningfully realisable; above 20 kHz no joint_states stream exists.
    static constexpr double kFilterRateMinHz = 40.0;
    static constexpr double kFilterRateMaxHz = 20000.0;
    // Warm-up window: inter-arrival intervals collected before committing.
    static constexpr size_t kFilterRateWarmupSamples = 50;
    // ...but never wait longer than this, nor commit on fewer than this many
    // intervals (a median of a handful of samples is not a measurement).
    static constexpr double kFilterRateWarmupMaxSec = 2.0;
    static constexpr size_t kFilterRateMinSamples = 8;
    // Fraction of intervals that must sit within +-50% of the median for the
    // measurement to be trusted.
    static constexpr double kFilterRateConsistency = 0.6;

    // ── Damping-path velocity filter ──
    //
    // The output damping term (controlLoop) realises kd*qdot with
    // kd = per_joint_torque_limit / max_joint_velocity. It has two hard
    // requirements that the state filter cannot meet at the same time, and
    // both were being violated:
    //
    //  1. It must never invert. A damper removes energy only while its
    //     velocity signal leads the true velocity by less than 90 deg. A
    //     2nd-order Butterworth asymptotes to 180 deg of lag, so above its
    //     corner it turns the damper into an *amplifier*: at the 10 Hz state
    //     cutoff the phase is 124 deg at 15.9 Hz (Spot's hip-abduction
    //     resonance) and 159 deg at 38.5 Hz (its unloaded knee), i.e. the term
    //     was contributing -0.56 and -0.93 of its nominal damping there. A
    //     FIRST-order lag can never exceed 90 deg, so it can only ever
    //     attenuate a damper, never reverse it. The order matters more than
    //     the cutoff, which is why this is a separate filter and not simply a
    //     faster kStateCutoffHz.
    //
    //  2. It must be stable as a sampled-data loop. The damping value is only
    //     refreshed when a joint_states message arrives (measured 137 Hz here)
    //     and is then held for the whole feedback period T by the 1 kHz control
    //     loop. For a joint of inertia I that is a ZOH damper, stable only
    //     while kd*T/I < 2. Spot's knee (I = 0.0154 kg m^2, kd = 12.25) gives
    //     kd*T/I = 5.8, so the *unfiltered* velocity limit-cycles: one
    //     saturated 100 Nm period changes the knee velocity by 47 rad/s, which
    //     overshoots and reverses it, and the joint chatters at the Nyquist
    //     frequency with a mean torque of ~0 - measured, and the reason raw
    //     velocity alone does not stand the robot up.
    //
    // With a first-order lag of pole a = exp(-2*pi*fc/fs), requirement 2
    // becomes kd*T/I < 2(1+a)/(1-a). Using the free-joint knee inertia that
    // predicts fc < 15.6 Hz; the measured edge is higher (~35-40 Hz) because
    // the knee in stance carries the reflected trunk and because the FIC
    // impedance is already saturated there. Both edges were measured directly,
    // sweeping the cutoff on Spot's hold-your-own-pose test (trunk z, mean knee
    // tracking error, peak joint velocity over the last 3 s of a 12 s hold):
    //
    //   fc (Hz)   0     5     8    10    12    15    20    25    30    40    60
    //   z (m)   .227  .102  .435  .434  .435  .438  .439  .439  .439  .419  .104
    //   err(rad).807  .869  .051  .050  .049  .039  .034  .034  .034  .132  .200
    //   |v|max  164   0.3   28    20    8.8   3.2   0.5   0.6   0.9   42    62
    //   fault    1     1     0     0     0     0     0     0     0     0     0
    //
    // i.e. a plateau from ~8 to ~30 Hz, anti-damping below it (the incumbent
    // failure) and sampled-data limit-cycling above it (the raw-velocity
    // failure). 20 Hz is the middle of the plateau on a log scale, with a 2x
    // margin to each measured edge; it is also above the 15.9 Hz hip resonance,
    // so the term keeps +0.78 of its true damping there and +0.46 at the
    // 38.5 Hz knee resonance - against -0.56 and -0.93 for the incumbent
    // 2nd-order 10 Hz path. Set to 0 to use the raw velocity directly.
    static constexpr double kDampingCutoffHz = 20.0;
    double damping_alpha_ = 0.0;   // recomputed whenever filter_rate_hz_ changes

    double filter_rate_hz_ = kFilterRateFallbackHz;
    bool   filter_rate_established_ = false;   // latched: filters rebuilt (or fallback kept) exactly once
    double last_feedback_stamp_s_ = -1.0;
    double first_feedback_stamp_s_ = -1.0;
    double filter_rate_deadline_s_ = -1.0;     // wall-clock backstop for the control-loop gate
    std::vector<double> feedback_intervals_;
    std::vector<ButterworthFilter> torque_filters_;
    std::vector<ButterworthFilter> position_filters_;
    // Continuous joints only: the raw angle carried across +-pi so the
    // position filter works on a continuous signal, whose output is then
    // wrapped to (-pi, pi]. MuJoCo reports hinge angles unwrapped, so here
    // this is chiefly for parity with fic_hardware (the Kinova driver wraps
    // every joint): the rest of the controller assumes a continuous joint
    // lies in [-pi, pi] -- its limits are set to +-pi and IK answers are
    // clamped there -- and an unwrapped angle broke that. A base turned
    // past 360 deg (Gaze's full rotation) latched the joint-limit hard
    // fault in sim only; the CSPM, limited to +-180 deg, faulted at half a
    // turn. Being an unwrap, it also stays right for a source that wraps.
    std::vector<double> raw_unwrapped_;
    std::vector<double> prev_raw_;
    std::vector<bool>   joint_continuous_;

    // True for a joint the URDF declares continuous and whose unwrap
    // state exists; false on any size mismatch (plain filtering).
    bool isContinuousJoint(size_t i) const {
        return i < joint_continuous_.size() && joint_continuous_[i] &&
               i < raw_unwrapped_.size() && i < prev_raw_.size();
    }
    std::vector<ButterworthFilter> velocity_filters_;

    // Measures the joint_states inter-arrival period and, once the warm-up
    // window is full, rebuilds the filter banks from the observed rate.
    // Called from jointStateCallback with data_mutex_ held.
    void calibrateFilterRate(double stamp_s);

    // Recomputes damping_alpha_ from kDampingCutoffHz and filter_rate_hz_.
    // Called at construction and again once the feedback rate is measured.
    void updateDampingAlpha();

    // ── Per-joint state sizing ──
    // Sole owner of num_joints_ and of every per-joint buffer's length. Two
    // places change the joint count — the launch parameter and the GUI's
    // number_of_joints topic — and both go through here, so they cannot size
    // different subsets of the state and leave the rest indexed out of bounds.
    void resizeJointState(size_t n);

    // Defensive backstop: the number of joints it is actually safe to index,
    // i.e. min(num_joints_, every per-joint buffer's size). Equal to
    // num_joints_ whenever resizeJointState is the only writer; if it is not,
    // this clamps and logs loudly rather than letting the loop run off the end.
    size_t checkedJointCount(const char* where);

    // ── FIC Library ──
    std::vector<fic::ScalarParams> joint_scalars_;
    std::vector<fic::ListParams> joint_lists_;
    std::vector<fic::PlannerParams> joint_planners_;
    std::vector<fic::AdmittanceParams> joint_admittance_;
    // Null-space postural task parameters (robot-level <postural> block), loaded at every
    // Cartesian/H-PMP init so postural.apply() runs on initialised channels (as fic_hardware).
    fic::ScalarParams postural_scalars_;
    fic::ListParams postural_lists_;
    fic::PlannerParams postural_planner_;
    fic::AdmittanceParams postural_admittance_;
    fic::CartesianAllParams cart_params_;
    fic::ficControl<T> fic_ctrl_;

    // ── Tuning (see fic::TuningSet) ──
    // config.xml is read off the control thread by refreshTuning(). The loop
    // keeps running on the set it has until the new one is ready, then swaps it
    // in; re-seeding the controllers for a new target only copies from memory.
    fic::TuningSet tuning_;
    bool have_tuning_ = false;
    std::future<fic::TuningSet> tuning_load_;
    bool tuning_reload_ = false;      // the tuning on disk changed: read it again (guarded by data_mutex_)
    bool tuning_failed_ = false;      // the last read for tuning_failed_key_ threw: don't retry it every tick
    fic::TuningKey tuning_failed_key_;

    // ── Pre-allocated Loop Variables ──
    std::vector<double> loop_traj_data_;
    std::vector<double> loop_torque_data_;
    std::vector<double> loop_current_pose_;
    std::vector<double> loop_target_pose_;
    std::vector<double> loop_wrench_data_;
    std::vector<double> loop_elbow_current_;
    std::vector<double> loop_elbow_target_;

    // ── Flags ──
    bool number_of_joints_received_ = false;
    bool robot_selection_received_ = false;
    bool mujoco_gravity_received_ = false;
    bool joint_state_received_ = false;   // latched once real joint feedback has populated q_current_
    bool initControllerPLanner = false;
    bool postural_enabled_ = false;
    bool elbow_enabled_ = false;
    bool hpmp_enabled_ = false;
    // Desired wrench of the task-space admittance [Fx Fy Fz Mx My Mz], robotN_desired_wrench; zero at start-up.
    std::vector<double> desired_wrench_ = std::vector<double>(6, 0.0);
    bool admittance_use_planner_ = true;

    // ── Safety Limits ──
    struct CartesianLimits {
        double x_min = -1.0, x_max = 1.0;
        double y_min = -1.0, y_max = 1.0;
        double z_min = 0.0, z_max = 1.5;
        bool x_enabled = true, y_enabled = true, z_enabled = true;
    } cart_safety_limits_;

    // Whole-arm safety zones from robotN_safety/zones (own lock: read in the joint-state and target callbacks).
    // zone_tau_ is the zones' wall torque, worked out with the kinematics in the joint-state callback and
    // added by the control loop in every mode; zone_worst_ the arm's worst clearance, reported to the Studio.
    fic::safety::ZoneSet zones_;
    std::mutex zones_mutex_;
    bool zones_active_ = false;
    Eigen::VectorXd zone_tau_;
    fic::safety::ZoneSet::BodyHit zone_worst_;
    Eigen::Vector3d zone_worst_point_ = Eigen::Vector3d::Zero();
    bool zone_target_safe_ = false;
    Eigen::Vector3d zone_safe_target_ = Eigen::Vector3d::Zero();
    double zone_state_last_s_ = 0.0;

    std::vector<double> joint_safety_lower_;
    std::vector<double> joint_safety_upper_;
    bool joint_safety_enabled_ = true;
    double max_joint_velocity_ = 1.0;
    // Joint target speed of the joint modes (fic/joint_target_slew.h), rad/s; 0 (off) until the Studio sets it.
    fic::JointTargetSlew joint_slew_;
    double max_cart_velocity_ = 0.5;
    bool velocity_limits_enabled_ = true;
    double output_torque_limit_ = 150.0;   // fallback: clamp |tau| when no per-joint limit
    std::vector<double> per_joint_torque_limit_;  // from robot config; empty = use output_torque_limit_
    bool hard_fault_ = false;                     // latched on config-limit violation; forces gravity-comp only

    // ── ROS Timer ──
    rclcpp::TimerBase::SharedPtr control_timer_;

    // ── ROS Publishers ──
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr trajectory_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_command_publisher_;
    rclcpp::Publisher<communication::msg::ActuatorCmds>::SharedPtr control_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr actuator_torque_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr admittance_offset_publisher_;
    rclcpp::Publisher<communication::msg::Jacobian>::SharedPtr jacobian_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr ee_pose_publisher_;

    // ── ROS Subscribers ──
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_position_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr elbow_target_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr force_torque_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gravity_torque_subscriber_;
    // Joint torques of the environment on the arm (MuJoCo constraint + applied forces), from
    // which H-PMP's task-space admittance estimates the external wrench in simulation.
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr contact_torque_subscriber_;
    std::vector<double> contact_torques_;
    bool contact_torques_received_ = false;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr control_mode_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr num_joints_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_selection_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr active_profile_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr param_update_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr desired_joint_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_target_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gripper_width_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cart_safety_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_safety_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr vel_safety_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr zones_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr zone_state_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr postural_enable_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr elbow_enable_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hpmp_enable_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hpmp_enable_legacy_sub_;   // {rid}_hapfic_enable
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr desired_wrench_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr admittance_planner_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gravity_comp_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr wrench_gravity_comp_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr wrench_from_api_sub_;
    void hpmpEnableCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void desiredWrenchCallback(const geometry_msgs::msg::Wrench::SharedPtr msg);
    void admittancePlannerCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void gravityCompCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void wrenchGravityCompCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void wrenchFromApiCallback(const std_msgs::msg::Bool::SharedPtr msg);
    geometry_msgs::msg::Wrench current_wrench_;

    // ── Robot Model ──
    void loadRobotModel(const std::string& robot_name);

    // ── Kinematics ──
    inline Eigen::Vector3d rotVecFromQuatError(const Eigen::Quaterniond& q_curr, const Eigen::Quaterniond& q_des);
    Eigen::VectorXd solveFullIterativeIK(const Eigen::VectorXd& q_start, const geometry_msgs::msg::Pose& ee_target_des);
    void computeAndPublishJacobian(const Eigen::VectorXd& q);

    // ── ROS Callbacks ──
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void targetPositionCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void forceTorqueCallback(const geometry_msgs::msg::Wrench::SharedPtr msg);
    void gravityTorqueCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void contactTorqueCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void controlModeCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void numberOfJointsCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void robotSelectionCallback(const std_msgs::msg::String::SharedPtr msg);
    void activeProfileCallback(const std_msgs::msg::String::SharedPtr msg);
    void paramUpdateCallback(const std_msgs::msg::String::SharedPtr msg);
    void desiredJointCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void targetJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void gripperWidthCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void posturalEnableCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void elbowEnableCallback(const std_msgs::msg::Bool::SharedPtr msg);

    // ── Safety Callbacks ──
    void cartesianSafetyCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void jointSafetyCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void velocitySafetyCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void zonesCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    // Whole-arm safety zones (fic/safety_zones.h)
    std::vector<std::string> zoneBodyFrames(int from_joint) const;
    Eigen::VectorXd zoneConfig(const Eigen::VectorXd& q_joints) const;
    void updateZoneWall(const Eigen::VectorXd& q);
    bool bodyTargetClear(const Eigen::VectorXd& q_joints);

    // ── Safety Clamping ──
    double clamp(double value, double min_val, double max_val);
    void applyCartesianSafetyLimits(geometry_msgs::msg::Pose& target_pose);
    void applyJointSafetyLimits(Eigen::VectorXd& joint_target);
    void applyVelocityLimits(Eigen::VectorXd& joint_velocities, double dt);
    bool isPositionInSafetyZone(double x, double y, double z);
    bool eeTargetInsideZone(const Eigen::VectorXd& q);

    // ── Controller Init ──
    void initJointControllers(size_t n_j, const Eigen::VectorXd& q_curr,
        const std::vector<fic::ScalarParams>& scalars, const std::vector<fic::ListParams>& lists,
        const std::vector<fic::PlannerParams>& planners, const std::vector<fic::AdmittanceParams>& admittance);
    void initCartesianControllers(const std::vector<double>& current_pose, const fic::CartesianAllParams& cart_params);
    // Starts/collects the background config.xml read. True once the tuning for
    // `want` is in memory (in joint_scalars_ … cart_params_).
    bool refreshTuning(const fic::TuningKey& want, const std::string& profile);

    // ── Main Loop ──
    void controlLoop();
};