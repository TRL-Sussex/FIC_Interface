// fic_control.h — FIC Hardware Node declaration
#pragma once

#include "communication/msg/actuator_cmds.hpp"
#include "communication/msg/jacobian.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "fic/joint_target_slew.h"
#include "fic/safety_zones.h"
#include <cmath>

#include <filesystem>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <future>
#include <chrono>
#include <vector>
#include <string>
#include <cstdlib>
#include <atomic>
#include <mutex>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/wrench.hpp"

// Kinematics solver (pluggable; pinocchio is the default backend)
#include <fic/kinematics_solver.hpp>
#include <fic/solvers/solver_factory.hpp>

#include <fic/fic_control.h>
#include <fic/shared_params.h>
#include <fic/butterworth_filter.hpp>
#include <fic/fic_param_loader.hpp>
#include <fic/robot_config_loader.hpp>

#include "fic_control/mcu_backend.hpp"

// ═══════════════════════════════════════════════════════════════════════════════
//  FIC HARDWARE NODE
// ═══════════════════════════════════════════════════════════════════════════════

template <typename T>
class FicControlNode : public rclcpp::Node {
public:
    explicit FicControlNode(int default_num_joints);
    ~FicControlNode() override;

    // Clean shutdown: tell an inner-loop MCU to ramp out of torque mode.
    void shutdownMcu();

private:
    // ── Core State ──
    int current_control_mode_;
    size_t num_joints_;
    bool Adm_enable = true;
    int AdmMode = 0;
    T dt = static_cast<T>(0.001);
    std::mutex data_mutex_;
    std::string selected_robot_;
    std::string active_profile_ = "default";
    bool is_robot_supported_ = false;

    // ── Robot Config ──
    fic::RobotConfigLoader robot_config_loader_;
    fic::RobotConfig current_robot_config_;
    std::string ee_frame_name_;
    std::string elbow_frame_name_;

    // ── Kinematics Solver (null in degraded mode: joint-space only) ──
    std::unique_ptr<fic::KinematicsSolver> solver_;

    // ── Geometry / Eigen State ──
    geometry_msgs::msg::Pose ee_current;
    geometry_msgs::msg::Pose ee_target;
    geometry_msgs::msg::Pose elbow_current_pose_;
    geometry_msgs::msg::Pose elbow_target_pose_;
    Eigen::MatrixXd jacobian;
    Eigen::VectorXd q_current_;
    Eigen::VectorXd q_target_;
    Eigen::VectorXd q_home;
    Eigen::VectorXd q_target_ik_;
    // What the postural task pulls toward in the Cartesian modes: the IK
    // solution q_target_ik_, but reached at no more than kPosturalMaxRate
    // per joint (stepPosturalTarget). Near a singularity -- the Kinova's
    // joint 2 at zero lines joints 1 and 3 up -- the IK can come back with a
    // solution far round the null space, and a postural task pulling
    // straight at it swung those two joints at 80-90 deg/s and faulted the
    // arm (Gaze sessions on 14 Sep 2026).
    Eigen::VectorXd q_postural_;
    std::chrono::steady_clock::time_point postural_at_{};   // last step
    static constexpr double kPosturalMaxRate = 0.52;   // rad/s, ~30 deg/s
    void stepPosturalTarget();
    Eigen::VectorXd gravity_torques_;
    Eigen::VectorXd ee_velocity_admittance_;
    std::vector<double> joint_torques_;
    std::vector<double> joint_positions_;
    std::vector<double> prev_joint_positions_;
    std::vector<T> currentVelocities_;
    // Raw (unfiltered) joint positions exactly as the driver wrote them into
    // shared memory, and the previous cycle's copy. Only the damping path uses
    // these; joint_positions_ stays the 10 Hz-filtered signal it always was.
    std::vector<double> raw_joint_positions_;
    std::vector<double> prev_raw_joint_positions_;
    // Continuous joints only: the raw angle carried across +-pi, so the
    // position filter never sees the driver's 2*pi wrap step (the driver
    // wraps every joint with atan2(sin, cos)). Filtering the wrapped angle
    // turned each crossing into a sweep through the whole circle, and the
    // controller used that sweep: on the Kinova, whose home pose parks
    // Actuator3 at -179.5 deg, 0.5 deg of motion put a garbage FK and
    // Jacobian into the Cartesian law and a 6283 rad/s velocity into the
    // damping path. See readJointFeedback's filter pass.
    std::vector<double> raw_unwrapped_;
    std::vector<double> prev_raw_wrapped_;
    std::vector<bool>   joint_continuous_;

    // True for a joint the URDF declares continuous and whose unwrap
    // state exists. False on any size mismatch, which falls back to the
    // plain filtering every joint used before.
    bool isContinuousJoint(size_t i) const {
        return i < joint_continuous_.size() && joint_continuous_[i] &&
               i < raw_unwrapped_.size() && i < prev_raw_wrapped_.size();
    }

    // Difference of two joint readings, taken the short way round for a
    // continuous joint so a +-pi crossing is not a 2*pi jump.
    double jointDelta(size_t i, double now, double before) const {
        const double d = now - before;
        return isContinuousJoint(i) ? std::atan2(std::sin(d), std::cos(d)) : d;
    }
    std::vector<double> raw_torques_scratch_;   // this cycle's raw torque samples
    // Unfiltered joint velocity (finite difference of raw_joint_positions_).
    std::vector<T> rawVelocities_;
    // The velocity the OUTPUT DAMPING term uses: rawVelocities_ through a
    // dedicated FIRST-ORDER lag (see kDampingCutoffHz).
    std::vector<T> dampingVelocities_;

    // ── Filters ──
    //
    // Sample-rate note. These banks are stepped from readSharedFeedback(),
    // which is called from exactly one place — controlLoop(), driven by the
    // 1 ms wall timer. So the rate at which they are stepped IS the control
    // loop rate, and declaring 1000 Hz is correct. (fic_simulation steps its
    // banks once per incoming joint_states message, a rate structurally
    // unrelated to its declared one, which is why it has to measure the
    // feedback rate at runtime and rebuild. Hardware has no such mismatch:
    // shared memory is polled, not subscribed.)
    // On the arm: torque 200 Hz, filters stepped at the 1 kHz control rate. In simulation (target
    // simulation) they are the old fic_simulation's: torque 10 Hz (the hardware model's torque_filter_hz when
    // it is on), stepped once per simulator step (only on a new STATE), at the simulator's rate.
    double torque_cutoff_hz_ = 200.0;
    static constexpr double kStateCutoffHz      = 10.0;
    double filter_rate_hz_ = 1000.0;
    double samplePeriod() const { return 1.0 / filter_rate_hz_; }

    // ── Damping-path velocity filter ──
    //
    // The output damping term in controlLoop realises kd*qdot with
    // kd = per_joint_torque_limit / max_joint_velocity. It used to be fed
    // currentVelocities_, i.e. the derivative of the 10 Hz SECOND-order
    // Butterworth position. A damper only removes energy while its velocity
    // signal lags the true velocity by less than 90 deg; a 2nd-order filter
    // asymptotes to 180 deg, so above its corner the term stops damping and
    // starts pumping (127 deg at 20 Hz, 150 deg at 35 Hz for a 10 Hz corner).
    // A FIRST-order lag can never exceed 90 deg, so it can only ever attenuate
    // a damper, never reverse it — the filter ORDER is what matters here, not
    // the cutoff, which is why this is a separate filter and not a faster
    // kStateCutoffHz. 20 Hz is the value fic_simulation measured (plateau
    // 8-30 Hz on Spot's hold-pose test); at hardware's 1 kHz refresh the
    // sampled-data (ZOH) stability bound kd*T/I < 2(1+a)/(1-a) is ~7x slacker
    // than it is at simulation's ~137 Hz feedback rate, so 20 Hz sits well
    // inside the safe region here too. Set to 0 to use the raw velocity.
    static constexpr double kDampingCutoffHz = 20.0;
    double damping_alpha_ = 0.0;   // exp(-2*pi*fc/fs); set by updateDampingAlpha()

    std::vector<ButterworthFilter> torque_filters_;
    std::vector<ButterworthFilter> position_filters_;
    std::vector<ButterworthFilter> velocity_filters_;

    // ── FIC Library ──
    std::vector<fic::ScalarParams> joint_scalars_;
    std::vector<fic::ListParams> joint_lists_;
    std::vector<fic::PlannerParams> joint_planners_;
    std::vector<fic::AdmittanceParams> joint_admittance_;
    fic::CartesianAllParams cart_params_;
    fic::ScalarParams postural_scalars_;
    fic::ListParams postural_lists_;
    fic::PlannerParams postural_planner_;
    fic::AdmittanceParams postural_admittance_;
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

    // ── Flags ──
    bool initControllerPLanner = false;
    bool robot_selection_received_ = false;
    bool number_of_joints_received_ = false;
    // Latched once the vendor driver has actually written joint feedback into
    // shared memory. Until then q_current_ / joint_positions_ hold the zeros
    // that initSharedMemory() wrote, which is NOT a robot state: the joint-limit
    // hard-fault check must not be evaluated against it (Franka_FR3's joint 3
    // range is [-176, -4] deg, so 0 deg latches a fault on the first tick), and
    // no torque may be commanded from it either. Re-armed by resizeVectors()
    // whenever the per-joint buffers are re-zeroed for a new robot.
    bool feedback_received_ = false;
    // Latched once the driver has written a measured gripper state into
    // shared memory (kGripperFeedbackSlot is not the bit-exact zero pair).
    // Re-armed by resizeVectors() and whenever `alive` drops (the driver ran
    // cleanupSharedMemory(), zeroing the slot). See publishGripperState().
    bool gripper_feedback_seen_ = false;
    // Control cycles since {rid}_gripper_state was last published.
    unsigned gripper_state_tick_ = 0;
    bool postural_enabled_ = false;
    bool elbow_enabled_ = false;
    bool hpmp_enabled_ = false;
    // Desired wrench of the task-space admittance [Fx Fy Fz Mx My Mz], robotN_desired_wrench; zero at start-up.
    std::vector<double> desired_wrench_ = std::vector<double>(6, 0.0);
    bool admittance_use_planner_ = true;

    // ── Torque ramp-down on stop ──
    std::vector<double> ramp_down_torques_;
    bool is_ramping_down_ = false;

    // ── Safety Limits ──
    struct CartesianLimits {
        double x_min = -1.0, x_max = 1.0;
        double y_min = -1.0, y_max = 1.0;
        double z_min = 0.0, z_max = 1.5;
        bool x_enabled = true, y_enabled = true, z_enabled = true;
    } cart_safety_limits_;

    // Whole-arm safety zones from robotN_safety/zones (own lock: read in the target callbacks, which do not all
    // take data_mutex_). zone_tau_ is their wall torque, worked out with the kinematics in the control loop and
    // added in every mode; zone_worst_ the arm's worst clearance, reported to the Studio.
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
    // Joint target speed of the joint modes (fic/joint_target_slew.h), rad/s; 0: off.
    fic::JointTargetSlew joint_slew_;
    double max_cart_velocity_ = 0.5;
    bool velocity_limits_enabled_ = true;
    double output_torque_limit_ = 150.0;   // fallback: clamp |tau| when no per-joint limit
    std::vector<double> per_joint_torque_limit_;  // from robot config; empty = use output_torque_limit_
    bool hard_fault_ = false;                     // latched on config-limit violation; forces gravity-comp only

    // ── ROS Timer ──
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr home_startup_timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr home_startup_pub_;

    // ── ROS Publishers ──
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr trajectory_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr q_current_publisher_;
    // Unfiltered joint positions, straight off the shared-memory feedback.
    // Added when position_filters_ low-passed the WRAPPED angle and every
    // +-pi crossing became a ~35 ms sweep across the whole circle in
    // q_current (measured 275 deg in 35 ms = 7873 deg/s). That sweep is now
    // gone -- continuous joints are unwrapped before filtering -- but this
    // stays for consumers that want the sample with no filter lag at all.
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr q_current_raw_publisher_;
    // The joint target the impedance controller is actually tracking.
    // Not the same as the {rid}_desired_joint_angles command: in the
    // Cartesian modes q_target_ comes from this node's own IK, so a
    // consumer that wants the controller's tracking error cannot
    // reconstruct it from the command stream. fic_contact's deflection
    // channel needs it: K*(q_current - q_target) is a second, encoder-
    // based estimate of the external joint torque.
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr q_target_publisher_;
    // The joint target the controller is tracking THIS CYCLE. Not the same
    // as q_target_: the Cartesian modes track q_target_ik_, derived from
    // this node's own IK, so a consumer cannot know which is live without
    // duplicating the mode logic. Publishing the effective one keeps that
    // knowledge here, where it belongs. fic_contact compares q_current
    // against it to decide the arm is settled and untouched.
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr q_target_active_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr ee_pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr wrench_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr admittance_offset_publisher_;
    // Measured gripper state [width_m, motor_current] from the driver's
    // shared-memory gripper slot. Only published once the driver has written
    // one, so "no messages" means no gripper feedback (sim, older driver).
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_state_publisher_;

    // ── ROS Subscribers ──
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_position_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_mode_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr desired_joint_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_selection_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr active_profile_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr param_update_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr num_joints_subscriber_;
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
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr wrench_external_sub_;
    // Simulation inputs (target simulation): MuJoCo's exact gravity, the joint torques the environment
    // applies (H-PMP's wrench on the ideal simulator) and the wrist F/T sensor (wrench_from_api).
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sim_gravity_sub_, sim_contact_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr sim_ft_sub_;
    std::vector<double> sim_gravity_, sim_contact_;
    geometry_msgs::msg::Wrench sim_ft_;
    // Simulation outputs, from the inner loop's STATE: what fic_simulation published (the experiments record
    // them): the law's torque (fic_torque_commands), the applied torque (actuators_cmds, actuator_torques),
    // the Jacobian.
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr sim_torque_cmd_pub_, sim_actuator_torque_pub_;
    rclcpp::Publisher<communication::msg::ActuatorCmds>::SharedPtr sim_actuators_pub_;
    rclcpp::Publisher<communication::msg::Jacobian>::SharedPtr sim_jacobian_pub_;
    void publishSimOutputs();
    // plant topics (the Newton backend: no lockstep): this node is the plant's side of
    // the inner loop — SimStep from the latest joint_states every tick, the SimTorque as actuators_cmds.
    bool topics_plant_ = false;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sim_joint_sub_;
    // Simulation only: the old fic_simulation's mode and joint-target topics (the experiment suite and the
    // extensions publish them), passed to the same callbacks as the hardware topics.
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sim_mode_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sim_target_joint_sub_;
    std::vector<double> plant_q_, plant_tau_;
    std::chrono::steady_clock::time_point plant_stamp_{};
    void topicPlantTick();
    bool sim_ft_ok_ = false;

    // ── Reflected wrench, for bilateral teleoperation ─────────────────────
    // A teleoperation leader renders what its follower is touching by
    // publishing on {rid}_wrench_external. The value is ADDED to the measured
    // wrench before it reaches the Cartesian admittance and Hpmp laws, so
    // the arm responds to remote contact exactly as it responds to its own.
    //
    // It is deliberately NOT added to what {rid}_force_torque publishes. That
    // topic stays the arm's own measurement: if the reflected force appeared
    // there, a leader and follower reflecting to each other would close a
    // positive feedback loop through the network.
    //
    // Everything that makes this safe lives here rather than in the GUI,
    // because the GUI is on the other side of a wifi link that fails.
    //
    // The per-axis limits and deadbands are per robot, from
    // robots_config.yaml `teleop_wrench_limits` via RobotConfig. The fade
    // below is not: how fast a reflected force must vanish when the link dies
    // is a property of the controller, the same for every arm, and not
    // something a config file should be able to weaken.
    //: Age at which a reflected wrench stops being trusted, and the time it
    //: then takes to fade linearly to zero. A fade rather than a cut, because
    //: a step to zero is itself a disturbance. Together: gone within 300 ms of
    //: the partner going quiet, which is inside the session layer's own
    //: 0.3 s hold threshold.
    static constexpr double kExtWrenchStaleS = 0.1;
    static constexpr double kExtWrenchFadeS  = 0.2;

    std::vector<double> external_wrench_ = std::vector<double>(6, 0.0);
    rclcpp::Time external_wrench_stamp_;
    bool external_wrench_active_ = false;   // false until the first message

    // ── Streamed-target watchdog ──────────────────────────────────────────
    // {rid}_desired_joint_angles is used two ways: a one-shot target the arm
    // then holds for minutes (Send on the Robot Control page), and a live
    // stream (teleoperation, the keyboard, the trajectory player, the
    // experiment runner). Only the second kind can be abandoned halfway, and
    // until now nothing noticed: if the Studio died mid-stream the arm went on
    // pulling at full stiffness toward a target nobody was updating any more,
    // indefinitely.
    //
    // So: recognise a stream by its arrival rate, and if a recognised stream
    // stops, stop chasing its last target and hold where the arm actually is.
    // A one-shot target never arms this, so holding a commanded pose for an
    // hour still works exactly as before.
    //
    // The threshold is deliberately looser than the teleop session layer's own
    // 0.3 s hold and 2.0 s end (fic_studio/teleop/session.py). This is the
    // backstop for that logic being gone, not a second copy of it.
    static constexpr double kTargetStreamGapS = 0.2;  // closer than this reads as a stream
    static constexpr int    kTargetStreamMin  = 5;    // arrivals before we believe it
    static constexpr double kTargetStaleS     = 2.0;  // stream gone this long
    //: Written in the subscription callback, read in the control loop, and
    //: desiredJointCallback does not take data_mutex_. Atomics rather than
    //: plain members so there is no torn read across the two threads.
    std::atomic<double> last_target_s_{0.0};
    std::atomic<bool>   target_stream_active_{false};
    double prev_target_s_ = 0.0;      // callback thread only
    int    target_stream_count_ = 0;  // callback thread only
    void hpmpEnableCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void desiredWrenchCallback(const geometry_msgs::msg::Wrench::SharedPtr msg);
    void gravityCompCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void wrenchGravityCompCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void wrenchFromApiCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void wrenchExternalCallback(const geometry_msgs::msg::Wrench::SharedPtr msg);

    // ── Internal Helpers ──
    void resizeVectors(size_t n);
    // Recomputes damping_alpha_ from kDampingCutoffHz and kFilterSampleRateHz.
    void updateDampingAlpha();
    void loadRobotModel(const std::string& robot_name);
    Eigen::VectorXd solveFullIterativeIK(const Eigen::VectorXd& q_start, const geometry_msgs::msg::Pose& ee_target_des);

    // IK off the control thread. targetPositionCallback only stores the newest target; this
    // worker solves the newest one pending (a target that arrives during a solve replaces any
    // older pending one) and publishes q_target_ik_ / the elbow target under data_mutex_. A
    // solve (up to 1000 iterations, ~56 ms on the Kinova) used to run inside the single-threaded
    // executor, so every streamed target froze the 1 kHz control loop for its duration.
    void ikWorker();
    std::thread ik_thread_;
    std::mutex ik_mutex_;                  // ik_target_, ik_pending_, ik_stop_
    std::condition_variable ik_cv_;
    geometry_msgs::msg::Pose ik_target_;
    bool ik_pending_ = false;
    bool ik_stop_ = false;
    std::mutex solver_mutex_;              // held while the worker solves, and while solver_ is replaced
    void readSharedFeedback();
    // Which inner loop drives the arm, for the log: the MCU, the PC's fic_inner_loop, or the simulation's.
    const char* innerLoopWhere() const {
        return sim_ ? "fic_inner_loop, simulation" : (mcu_.local() ? "fic_inner_loop on this PC" : "the MCU");
    }
    void publishGripperState();
    void initJointControllers(size_t n_j, const Eigen::VectorXd& q_curr,
        const std::vector<fic::ScalarParams>& scalars, const std::vector<fic::ListParams>& lists,
        const std::vector<fic::PlannerParams>& planners, const std::vector<fic::AdmittanceParams>& admittance);
    void initCartesianControllers(const std::vector<double>& current_pose, const fic::CartesianAllParams& cart_params);
    // Starts/collects the background config.xml read; called with data_mutex_
    // held. True once the tuning for `want` is in memory (joint_scalars_ …).
    bool refreshTuning(const fic::TuningKey& want, const std::string& profile);

    // ── ROS Callbacks ──
    void gripperWidthCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void controlModeCallback(const std_msgs::msg::String::SharedPtr msg);
    void targetPositionCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void desiredJointCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void robotSelectionCallback(const std_msgs::msg::String::SharedPtr msg);
    void activeProfileCallback(const std_msgs::msg::String::SharedPtr msg);
    void paramUpdateCallback(const std_msgs::msg::String::SharedPtr msg);
    void numberOfJointsCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void posturalEnableCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void elbowEnableCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void admittancePlannerCallback(const std_msgs::msg::Bool::SharedPtr msg);

    // ── Safety Callbacks ──
    void cartesianSafetyCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void jointSafetyCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void velocitySafetyCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void zonesCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    // Whole-arm safety zones (fic/safety_zones.h)
    std::vector<std::string> zoneBodyFrames(int from_joint) const;
    void updateZoneWall(const Eigen::VectorXd& q);
    bool bodyTargetClear(const Eigen::VectorXd& q_joints);

    // ── Safety Clamping ──
    double clamp(double value, double min_val, double max_val);
    void applyCartesianSafetyLimits(geometry_msgs::msg::Pose& target_pose);
    void applyJointSafetyLimits(Eigen::VectorXd& joint_target);
    bool isPositionInSafetyZone(double x, double y, double z);
    bool eeTargetInsideZone(const Eigen::VectorXd& q);

    // ── Inner-loop MCU (mcu_backend.hpp) ──
    // Decided per robot, at load and again at "Start": if the robot has an
    // `mcu:` adapter in robots_config.yaml and an MCU at mcu_address reports
    // that adapter, the joint layer runs on the MCU and this node sends it the
    // outer-loop result each tick. Otherwise the standard vendor-driver path.
    fic_hw::McuBackend mcu_;
    fic_hw::McuBackend::RobotLimits mcu_limits_;
    // target simulation: this node drives a simulated arm (the simulator in lockstep with
    // fic_inner_loop --adapter sim on 127.0.0.1:simInnerLoopPort(station)); no driver, no Start needed.
    bool sim_ = false;
    std::string source_prefix_ = "simulation";   // the simulator's topics: /<source_prefix>/...
    std::string joint_name_prefix_;               // the simulator's joint and actuator names: <prefix>...
    double sim_dt_ = 0.002;                       // the simulator's physics step
    uint32_t last_state_tick_ = 0;                // the last STATE whose feedback was filtered (simulation)
    uint32_t last_published_tick_ = 0;            // the last STATE published as the simulation outputs
    bool use_mcu_ = false;          // an inner loop drives this robot: the MCU board, or fic_inner_loop (mcu_.local())
    unsigned inner_loop_retry_ = 0;   // status ticks without an inner loop (a retry every 4th)
    bool mcu_armed_ = false;        // "Start" done: torque may be enabled
    bool mcu_stop_sent_ = false;    // StopTorque sent; disarm once the MCU is out
    bool pending_target_snap_ = false;  // MCU path: set targets to the current pose once feedback is real
    std::string mcu_detail_;        // last detection result, for the status topic
    bool standard_driver_launched_ = false;  // vendor driver started: never re-probe
    std::atomic<bool> mcu_setup_running_{false};
    std::atomic<bool> mcu_setup_done_{false};
    std::atomic<bool> mcu_setup_failed_{false};
    std::thread mcu_setup_thread_;
    std::vector<double> wall_torque_;   // virtual-wall term, kept apart for tau_ff
    static constexpr double kGravJacStep = 1e-4;   // rad, finite-difference dg/dq
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr inner_loop_status_pub_;
    rclcpp::TimerBase::SharedPtr inner_loop_status_timer_;
    void selectInnerLoop(const char* when);
    void startMcu();
    void sendMcuCommand(const std::vector<double>& torque_data, bool solver_ok);
    std::string robotLibraryCommand(std::string cmd) const;

    // ── Main Loop ──
    void controlLoop();
};