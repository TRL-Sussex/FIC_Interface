// fic_control.cpp — FIC Hardware Node implementation
#include <fic/state_diag.h>
#include <fic/plant_sample.h>
#include "fic_control/fic_control.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>

// ── geometry_msgs::Pose ↔ Eigen::Isometry3d (solver interface boundary) ──
// Wrap an angle to (-pi, pi]. Same convention as the Kinova driver's
// wrapToPi, so a wrapped filter output matches what it would have reported.
static inline double wrapAngle(double x) {
    return std::atan2(std::sin(x), std::cos(x));
}

static Eigen::Isometry3d poseMsgToIso(const geometry_msgs::msg::Pose& p) {
    Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
    out.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
    out.linear() = Eigen::Quaterniond(p.orientation.w, p.orientation.x,
                                      p.orientation.y, p.orientation.z).toRotationMatrix();
    return out;
}

static geometry_msgs::msg::Pose isoToPoseMsg(const Eigen::Isometry3d& iso) {
    geometry_msgs::msg::Pose p;
    p.position.x = iso.translation().x();
    p.position.y = iso.translation().y();
    p.position.z = iso.translation().z();
    Eigen::Quaterniond q(iso.rotation());
    q.normalize();
    p.orientation.w = q.w();
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    return p;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  CONSTRUCTOR
// ═══════════════════════════════════════════════════════════════════════════════

// H-PMP's IK seed (5.7.0): the previous IK solution while H-PMP runs, so the arm's redundant
// (self-motion) posture is held by the joint layer and a push moves it only through the
// admittance offsets; the measured posture otherwise, or when the previous solution is more than
// kHpmpIkSeedMaxRad from it on any joint (after a mode switch or a large push). Seeding from the
// measured posture under H-PMP let the joint set-point follow a push in the null space and drift
// on from there.
static constexpr double kHpmpIkSeedMaxRad = 0.5;
static inline Eigen::VectorXd hpmpIkSeed(bool hpmp, const Eigen::VectorXd& q_meas,
                                         const Eigen::VectorXd& q_prev) {
    if (!hpmp || q_prev.size() != q_meas.size() || q_meas.size() == 0) return q_meas;
    for (Eigen::Index i = 0; i < q_meas.size(); ++i)
        if (!std::isfinite(q_prev[i]) || std::abs(q_prev[i] - q_meas[i]) > kHpmpIkSeedMaxRad)
            return q_meas;
    return q_prev;
}

// The arm's feedback as this node sees it: filled from the inner loop's STATE (McuBackend::mirrorFeedback), on
// both paths. This node never maps the vendor driver's shared memory any more: the inner loop (the MCU, or
// fic_inner_loop on this PC) owns the arm; initSharedMemory() here would also reset the driver's fields.
static SharedFeedback g_feedback_block;

template <typename T>
FicControlNode<T>::FicControlNode(int default_num_joints)
    : Node("fic_hardware_node"),
      num_joints_(default_num_joints),
      fic_ctrl_(4.0, true, 2.0, 23, Adm_enable)
{
    RCLCPP_INFO(this->get_logger(), "Robot Control Node has been started.");
    shared_feedback = &g_feedback_block;
    shared_feedback->commanded_gripper_width.store(0.08f, std::memory_order_relaxed);
    shared_feedback->alive.store(1, std::memory_order_release);

    if (!robot_config_loader_.loadFromDefault()) {
        RCLCPP_WARN(this->get_logger(), "Could not load robots_config.yaml, using built-in defaults");
    } else {
        RCLCPP_INFO(this->get_logger(), "Loaded %zu robot configuration(s) from YAML", robot_config_loader_.size());
    }

    q_home = Eigen::VectorXd::Zero(num_joints_);
    q_current_ = Eigen::VectorXd::Zero(num_joints_);
    updateDampingAlpha();
    resizeVectors(num_joints_);
    current_control_mode_ = 1;

    // Station ID parameter — determines which robotN topics to use (1-4)
    this->declare_parameter<int>("station_id", 1);
    int station_id = this->get_parameter("station_id").as_int();
    std::string rid = "robot" + std::to_string(station_id);
    RCLCPP_INFO(this->get_logger(), "Station ID: %d — subscribing to %s topics", station_id, rid.c_str());

    // Subscribers (topic names derived from station_id)
    control_mode_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        rid + "_mode_select", 10,
        std::bind(&FicControlNode::controlModeCallback, this, std::placeholders::_1));

    desired_joint_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        rid + "_desired_joint_angles", 10,
        std::bind(&FicControlNode::desiredJointCallback, this, std::placeholders::_1));

    target_position_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "command_ee_position_change_" + rid, 10,
        std::bind(&FicControlNode::targetPositionCallback, this, std::placeholders::_1));

    robot_selection_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        rid + "_selected_robot", 10,
        std::bind(&FicControlNode::robotSelectionCallback, this, std::placeholders::_1));

    // Reliable + transient-local, as FIC Studio and the experiment runner publish it: a profile chosen before
    // this node started still reaches it.
    active_profile_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        rid + "_active_profile", rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&FicControlNode::activeProfileCallback, this, std::placeholders::_1));

    // Explicit "the tuning on disk changed, re-read it" signal. desiredJointCallback
    // used to reload config.xml on every streamed joint target, which is why it is
    // no longer done for Joint Impedance — but that was the only runtime reload
    // trigger for that mode, so stopping it would silently kill live re-tuning.
    // This restores the reload without tying it to the target stream. The GUI
    // publishes "<rid>:<param_type>:<value>" here on every successful SAVE TO XML.
    param_update_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        rid + "_param_update", 10,
        std::bind(&FicControlNode::paramUpdateCallback, this, std::placeholders::_1));

    num_joints_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
        rid + "_number_of_joints", 10,
        std::bind(&FicControlNode::numberOfJointsCallback, this, std::placeholders::_1));

    gripper_width_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
        rid + "_gripper_width", 10,
        std::bind(&FicControlNode::gripperWidthCallback, this, std::placeholders::_1));

    // Safety subscribers (station-specific)
    cart_safety_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        rid + "_safety/cartesian_limits", 10,
        std::bind(&FicControlNode::cartesianSafetyCallback, this, std::placeholders::_1));

    joint_safety_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        rid + "_safety/joint_limits", 10,
        std::bind(&FicControlNode::jointSafetyCallback, this, std::placeholders::_1));

    vel_safety_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        rid + "_safety/velocity_limits", 10,
        std::bind(&FicControlNode::velocitySafetyCallback, this, std::placeholders::_1));

    // Whole-arm safety zones (the Studio latches them, so a node started later still gets them)
    zones_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        rid + "_safety/zones", rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&FicControlNode::zonesCallback, this, std::placeholders::_1));
    zone_state_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(rid + "_safety/zone_state", 10);

    postural_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        rid + "_postural_enable", 10,
        std::bind(&FicControlNode::posturalEnableCallback, this, std::placeholders::_1));

    elbow_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        rid + "_elbow_enable", 10,
        std::bind(&FicControlNode::elbowEnableCallback, this, std::placeholders::_1));

    // the old topic name, still accepted
    hpmp_enable_legacy_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        rid + "_hapfic_enable", 10,
        std::bind(&FicControlNode::hpmpEnableCallback, this, std::placeholders::_1));
    // H-PMP's topic (since 5.7.0).
    hpmp_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        rid + "_hpmp_enable", 10,
        std::bind(&FicControlNode::hpmpEnableCallback, this, std::placeholders::_1));

    desired_wrench_sub_ = this->create_subscription<geometry_msgs::msg::Wrench>(
        rid + "_desired_wrench", 10,
        std::bind(&FicControlNode::desiredWrenchCallback, this, std::placeholders::_1));

    admittance_planner_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        rid + "_admittance_planner_enable", 10,
        std::bind(&FicControlNode::admittancePlannerCallback, this, std::placeholders::_1));

    gravity_comp_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        rid + "_gravity_compensated", rclcpp::QoS(1).transient_local(),
        std::bind(&FicControlNode::gravityCompCallback, this, std::placeholders::_1));

    wrench_gravity_comp_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        rid + "_wrench_gravity_compensated", rclcpp::QoS(1).transient_local(),
        std::bind(&FicControlNode::wrenchGravityCompCallback, this, std::placeholders::_1));

    wrench_from_api_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        rid + "_wrench_from_api", rclcpp::QoS(1).transient_local(),
        std::bind(&FicControlNode::wrenchFromApiCallback, this, std::placeholders::_1));

    // Reflected contact from a teleoperation partner. SensorDataQoS: this is a
    // stream where the newest sample is the only one worth having, and over
    // wifi a reliable queue would stall on a lost fragment.
    wrench_external_sub_ = this->create_subscription<geometry_msgs::msg::Wrench>(
        rid + "_wrench_external", rclcpp::SensorDataQoS(),
        std::bind(&FicControlNode::wrenchExternalCallback, this, std::placeholders::_1));

    // Publishers (station-specific state feedback)
    trajectory_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(rid + "_trajectory_data", 10);
    q_current_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(rid + "_q_current", 10);
    q_current_raw_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(rid + "_q_current_raw", rclcpp::SensorDataQoS());
    q_target_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(rid + "_q_target", rclcpp::SensorDataQoS());
    q_target_active_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(rid + "_q_target_active", rclcpp::SensorDataQoS());
    ee_pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>(rid + "_ee_current_pose", 10);
    wrench_publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>(rid + "_force_torque", rclcpp::SensorDataQoS());
    velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(rid + "_joint_velocities", 10);

    // Measured joint torque. Already sign-corrected (torque_sign) and
    // Butterworth-filtered by readSharedFeedback(). This is the ONLY topic
    // carrying the arm's measured torque — trajectory_data block 7 is written
    // only in joint-admittance modes 3/4, and block 4 is the commanded effort.
    torque_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        rid + "_joint_torques", rclcpp::SensorDataQoS());

    // Accumulated per-joint "human push offset" (radians) from joint admittance
    // — the controller's own integrated yield, NOT planner lag (contrast
    // trajectory_data blocks 3/5). Published ONLY while joint admittance is
    // active (modes 3/4); see the publish call in controlLoop().
    admittance_offset_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        rid + "_admittance_offset", rclcpp::SensorDataQoS());

    // Measured gripper state, data = [width_m, motor_current], ~50 Hz. Read
    // from shared_params.h's kGripperFeedbackSlot; see publishGripperState().
    gripper_state_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        rid + "_gripper_state", rclcpp::SensorDataQoS());

    // target: hardware (the arm, through the MCU or fic_inner_loop on its driver's shared memory) or simulation
    // (a simulated arm, through fic_inner_loop --adapter sim in lockstep with the simulator). One node for both:
    // sim_control_launch.py runs it with target:=simulation.
    {
        this->declare_parameter<std::string>("target", "hardware");
        this->declare_parameter<std::string>("source_prefix", "simulation");
        this->declare_parameter<std::string>("joint_name_prefix", "");
        this->declare_parameter<double>("sim_dt", 0.002);
        this->declare_parameter<std::string>("plant", "auto");
        sim_ = this->get_parameter("target").as_string() == "simulation";
        source_prefix_ = this->get_parameter("source_prefix").as_string();
        joint_name_prefix_ = this->get_parameter("joint_name_prefix").as_string();
        sim_dt_ = std::max(1e-4, this->get_parameter("sim_dt").as_double());
        if (sim_) {
            mcu_.setSimAddress("127.0.0.1:" + std::to_string(fic::link::simInnerLoopPort(station_id)));
            filter_rate_hz_ = 1.0 / sim_dt_;
            updateDampingAlpha();
            const auto sens = rclcpp::SensorDataQoS().keep_last(1);
            sim_gravity_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/" + source_prefix_ + "/gravity_torques", sens,
                [this](const std_msgs::msg::Float64MultiArray::SharedPtr m) {
                    std::lock_guard<std::mutex> l(data_mutex_);
                    sim_gravity_.assign(m->data.begin(), m->data.end());
                });
            sim_contact_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/" + source_prefix_ + "/contact_torques", sens,
                [this](const std_msgs::msg::Float64MultiArray::SharedPtr m) {
                    std::lock_guard<std::mutex> l(data_mutex_);
                    sim_contact_.assign(m->data.begin(), m->data.end());
                });
            sim_ft_sub_ = this->create_subscription<geometry_msgs::msg::Wrench>(
                "/" + source_prefix_ + "/force_torque", sens,
                [this](const geometry_msgs::msg::Wrench::SharedPtr m) {
                    std::lock_guard<std::mutex> l(data_mutex_);
                    sim_ft_ = *m;
                    sim_ft_ok_ = true;
                });
            const auto sens1 = rclcpp::SensorDataQoS().keep_last(1);
            sim_torque_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                "/" + rid + "_fic_torque_commands", sens1);
            sim_actuator_torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                "/" + source_prefix_ + "/actuator_torques", sens1);
            sim_actuators_pub_ = this->create_publisher<communication::msg::ActuatorCmds>(
                "/" + source_prefix_ + "/actuators_cmds", sens1);
            sim_jacobian_pub_ = this->create_publisher<communication::msg::Jacobian>(rid + "_jacobian", sens1);
            // The old fic_simulation's topics: the mode as a number (0 Stop, 1-8) and the joint target as a
            // JointState. Simulation only, so a simulation tool never drives a real arm.
            sim_mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
                rid + "_control_mode", rclcpp::SensorDataQoS(),
                [this](const std_msgs::msg::Int32::SharedPtr m) {
                    static const char* kNames[] = {
                        "Stop Controller", "Joint Impedance", "Joint Impedance With Planner", "Joint Admittance",
                        "Joint Admittance Haptic", "Cartesian Impedance", "Cartesian Planner",
                        "Cartesian Admittance", "Cartesian Admittance Haptic"};
                    if (m->data < 0 || m->data > 8) {
                        RCLCPP_ERROR(this->get_logger(), "Unknown control mode %d (0 stops, 1-8; mode 9 is mode 5 "
                                     "with postural control on)", m->data);
                        return;
                    }
                    if (m->data == current_control_mode_) return;
                    auto s = std::make_shared<std_msgs::msg::String>();
                    s->data = kNames[m->data];
                    controlModeCallback(s);
                });
            sim_target_joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                rid + "_target_joint_angles", rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::JointState::SharedPtr m) {
                    if (m->position.size() != num_joints_) return;
                    auto a = std::make_shared<std_msgs::msg::Float32MultiArray>();
                    a->data.assign(m->position.begin(), m->position.end());
                    desiredJointCallback(a);
                });
            const std::string plant = this->get_parameter("plant").as_string();
            topics_plant_ = plant == "topics" || (plant == "auto" && source_prefix_.rfind("vr", 0) == 0);
            if (topics_plant_) {               // the inner loop ticks once per control tick (1 ms), not per step
                sim_dt_ = 0.001;
                filter_rate_hz_ = 1.0 / sim_dt_;
                updateDampingAlpha();
            }
            if (topics_plant_) {
                sim_joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                    "/" + source_prefix_ + "/joint_states", rclcpp::SensorDataQoS().keep_last(1),
                    [this](const sensor_msgs::msg::JointState::SharedPtr m) {
                        std::lock_guard<std::mutex> l(data_mutex_);
                        if (fic::pickJoints(current_robot_config_.joint_names, joint_name_prefix_, m->name,
                                            m->position, m->effort, plant_q_, plant_tau_))
                            plant_stamp_ = std::chrono::steady_clock::now();
                    });
            }
            RCLCPP_INFO(this->get_logger(), "Target: simulation (/%s, inner loop 127.0.0.1:%d, step %.4f s)",
                        source_prefix_.c_str(), fic::link::simInnerLoopPort(station_id), sim_dt_);
        }
    }
    // Inner-loop MCU: where to look for one (robots with an `mcu:` adapter). FIC_MCU_ADDRESS overrides the
    // default; an empty mcu_address disables it. Every other case uses the PC inner loop, fic_inner_loop
    // (127.0.0.1:7360, FIC_INNER_LOOP_ADDRESS), which robot_control_launch.py starts.
    {
        std::string mcu_default = "192.168.1.20";
        if (const char* env = std::getenv("FIC_MCU_ADDRESS")) mcu_default = env;
        this->declare_parameter<std::string>("mcu_address", mcu_default);
        mcu_.setAddress(this->get_parameter("mcu_address").as_string());
    }
    // Which inner loop drives this robot, ~2 Hz: "mcu ...", "inner loop ..." or "no inner loop: ...". While
    // none answers, look again every 2 s (the executor is single-threaded: the probe, at most 0.3 s, never
    // runs beside the control loop, which drives nothing meanwhile).
    inner_loop_status_pub_ = this->create_publisher<std_msgs::msg::String>(
        rid + "_inner_loop", rclcpp::QoS(1).transient_local());
    inner_loop_status_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), [this]() {
            std_msgs::msg::String m;
            if (!use_mcu_ && is_robot_supported_ && ++inner_loop_retry_ % 4 == 0) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                selectInnerLoop("retry");
            }
            if (!use_mcu_) {
                m.data = mcu_detail_.rfind("no inner loop", 0) == 0 ? mcu_detail_ : "no inner loop: " + mcu_detail_;
            } else if (!mcu_.configured()) {
                m.data = "mcu available, waiting for Start: " + mcu_detail_;
                if (mcu_setup_running_) m.data = "mcu: running setup_command";
                if (mcu_setup_failed_) m.data = "mcu: setup_command FAILED — press Start to retry";
            } else {
                m.data = mcu_.status();
            }
            inner_loop_status_pub_->publish(m);
        });

    // Parameter
    this->declare_parameter<std::string>("selected_robot", "Default_Robot");
    std::string robot_name_from_param = this->get_parameter("selected_robot").as_string();
    loadRobotModel(robot_name_from_param);

    // Publish home position so the robot moves there on startup (the arm only: a simulation starts where its
    // scene puts the arm, as the old fic_simulation did).
    if (!sim_ && !current_robot_config_.q_home_rad.empty()) {
        auto home_msg = std_msgs::msg::Float32MultiArray();
        for (size_t i = 0; i < current_robot_config_.q_home_rad.size(); ++i) {
            home_msg.data.push_back(static_cast<float>(current_robot_config_.q_home_rad[i]));
        }
        // Use a one-shot timer so the message is published after the node is fully initialized
        auto home_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            rid + "_desired_joint_angles", 10);
        home_startup_pub_ = home_pub;
        home_startup_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            [this, home_msg, rid]() {
                home_startup_timer_->cancel();       // one shot, whatever happens next
                // Somebody is already commanding this arm. A controller that
                // restarts mid-session (teleoperation, a trajectory, the
                // experiment runner) must not yank the arm to home underneath
                // whoever is driving it: by now a 100 Hz stream has sent
                // around fifty targets, and home could be a long way off.
                if (last_target_s_.load(std::memory_order_acquire) > 0.0) {
                    RCLCPP_INFO(this->get_logger(),
                        "Not publishing the home pose: %s_desired_joint_angles is already "
                        "being commanded, and interrupting that would move the arm.",
                        rid.c_str());
                    return;
                }
                home_startup_pub_->publish(home_msg);
                RCLCPP_INFO(this->get_logger(), "Published home position to %s_desired_joint_angles (%zu joints)",
                    rid.c_str(), home_msg.data.size());
            });
    }

    // Timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&FicControlNode::controlLoop, this));

    ik_thread_ = std::thread(&FicControlNode::ikWorker, this);

    RCLCPP_INFO(this->get_logger(), "FicControlNode initialization complete.");
}

template <typename T>
FicControlNode<T>::~FicControlNode() {
    {
        std::lock_guard<std::mutex> lk(ik_mutex_);
        ik_stop_ = true;
    }
    ik_cv_.notify_all();
    if (ik_thread_.joinable()) ik_thread_.join();
    if (mcu_setup_thread_.joinable()) mcu_setup_thread_.join();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  INTERNAL HELPERS
// ═══════════════════════════════════════════════════════════════════════════════

template <typename T>
void FicControlNode<T>::resizeVectors(size_t n) {
    joint_torques_.assign(n, 0.0);
    joint_positions_.assign(n, 0.0);
    gravity_torques_ = Eigen::VectorXd::Zero(n);
    currentVelocities_.assign(n, 0.0);
    raw_joint_positions_.assign(n, 0.0);
    // Zero is a safe start: the first unwrap step adds wrapAngle(raw - 0),
    // which is raw itself, so the unwrapped angle begins at the reading.
    raw_unwrapped_.assign(n, 0.0);
    prev_raw_wrapped_.assign(n, 0.0);
    raw_torques_scratch_.assign(n, 0.0);
    prev_raw_joint_positions_.clear();      // no previous sample for the new size
    rawVelocities_.assign(n, static_cast<T>(0.0));
    dampingVelocities_.assign(n, static_cast<T>(0.0));
    jacobian = Eigen::MatrixXd::Zero(6, n);

    torque_filters_.clear();
    position_filters_.clear();
    velocity_filters_.clear();

    for (size_t i = 0; i < n; ++i) {
        torque_filters_.emplace_back(torque_cutoff_hz_, filter_rate_hz_, 0.0);
        position_filters_.emplace_back(kStateCutoffHz, filter_rate_hz_, 0.0);
        velocity_filters_.emplace_back(kStateCutoffHz, filter_rate_hz_, 0.0);
    }

    // These buffers now hold zeros, not the robot's state. Re-arm the
    // genuine-feedback latch so neither the control law nor the joint-limit
    // hard-fault check runs against that fictitious state.
    feedback_received_ = false;
    gripper_feedback_seen_ = false;
    prev_joint_positions_.clear();
}

// First-order lag pole for the damping path: a = exp(-2*pi*fc/fs). fs is the
// rate at which the damping value is actually refreshed — here the control-loop
// rate, since readSharedFeedback() runs once per control tick. fc == 0 disables
// the filter and uses the raw velocity directly.
template <typename T>
void FicControlNode<T>::updateDampingAlpha() {
    double fc = kDampingCutoffHz;
    // Experiment hook (mirrors fic_simulation): lets the damping cutoff be
    // swept without a rebuild. Unset in normal operation.
    if (const char* env = std::getenv("FIC_DAMP_HZ")) {
        try { fc = std::stod(env); } catch (...) {}
    }
    if (fc <= 0.0 || filter_rate_hz_ <= 0.0) {
        damping_alpha_ = 0.0;
    } else {
        damping_alpha_ = std::exp(-2.0 * M_PI * fc / filter_rate_hz_);
        if (!std::isfinite(damping_alpha_)) damping_alpha_ = 0.0;
        damping_alpha_ = clamp(damping_alpha_, 0.0, 0.999);
    }
    RCLCPP_INFO(this->get_logger(),
        "Damping-path filter: 1st-order %.1f Hz at %.1f Hz refresh -> alpha %.4f "
        "(phase lag %.0f deg at 15.9 Hz, vs %.0f deg for the 2nd-order %.1f Hz state filter).",
        fc, filter_rate_hz_, damping_alpha_,
        fc > 0.0 ? std::atan(15.9 / fc) * 180.0 / M_PI : 0.0,
        2.0 * std::atan(15.9 / kStateCutoffHz) * 180.0 / M_PI, kStateCutoffHz);
}

template <typename T>
void FicControlNode<T>::loadRobotModel(const std::string& robot_name) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    selected_robot_ = robot_name;
    is_robot_supported_ = false;
    // A new robot starts on the standard path; selectInnerLoop() at the end
    // decides again. Early returns below must not leave the last robot's MCU on.
    use_mcu_ = false;
    mcu_armed_ = false;
    mcu_.reset();

    robot_config_loader_.loadFromDefault();
    auto config_opt = robot_config_loader_.getConfig(robot_name);
    if (config_opt.has_value()) {
        current_robot_config_ = config_opt.value();
        RCLCPP_INFO(this->get_logger(), "Loaded config from YAML for: %s", robot_name.c_str());
    } else {
        current_robot_config_ = fic::getDefaultConfig(robot_name);
        RCLCPP_WARN(this->get_logger(), "Using built-in defaults for: %s", robot_name.c_str());
    }

    if (sim_) {
        // FIC_HARDWARE_MODEL=off: the ideal simulator for this run, whatever robots_config.yaml says
        // (mujoco_native/hardware_model.py reads the same variable).
        if (const char* hm = std::getenv("FIC_HARDWARE_MODEL")) {
            std::string v(hm);
            std::transform(v.begin(), v.end(), v.begin(), ::tolower);
            if (v == "off" || v == "0" || v == "false" || v == "ideal")
                current_robot_config_.hardware_model.enabled = false;
        }
        // The simulator's measured torque is its actuator force (the node's own command, plus the hardware
        // model's friction and noise): the old fic_simulation's sign, here and in the inner loop's CONFIG.
        current_robot_config_.torque_sign = -1.0;
        current_robot_config_.gravity_compensated = false;   // the simulator never compensates gravity
        const auto& hw = current_robot_config_.hardware_model;
        torque_cutoff_hz_ = (hw.enabled && hw.torque_filter_hz > 0.0) ? hw.torque_filter_hz : 10.0;
        // The robot's velocity safety at load, as the old fic_simulation applied it; a simulation-only limit
        // (sim_max_joint_vel) wins unless the hardware model is on.
        if (current_robot_config_.safety_max_joint_vel)
            max_joint_velocity_ = *current_robot_config_.safety_max_joint_vel;
        if (current_robot_config_.safety_sim_max_joint_vel && !hw.enabled)
            max_joint_velocity_ = *current_robot_config_.safety_sim_max_joint_vel;
        if (current_robot_config_.safety_max_cart_vel)
            max_cart_velocity_ = *current_robot_config_.safety_max_cart_vel;
        if (current_robot_config_.safety_velocity_enabled)
            velocity_limits_enabled_ = *current_robot_config_.safety_velocity_enabled;
    }

    ee_frame_name_ = current_robot_config_.ee_frame_name;
    elbow_frame_name_ = current_robot_config_.elbow_frame_name;
    is_robot_supported_ = current_robot_config_.is_supported || sim_;   // every robot can be simulated

    if (!is_robot_supported_) {
        RCLCPP_ERROR(this->get_logger(), "Robot '%s' NOT SUPPORTED for hardware control.", robot_name.c_str());
    }

    ee_target.position.x = current_robot_config_.default_ee_position[0];
    ee_target.position.y = current_robot_config_.default_ee_position[1];
    ee_target.position.z = current_robot_config_.default_ee_position[2];
    ee_target.orientation.x = current_robot_config_.default_ee_orientation[0];
    ee_target.orientation.y = current_robot_config_.default_ee_orientation[1];
    ee_target.orientation.z = current_robot_config_.default_ee_orientation[2];
    ee_target.orientation.w = current_robot_config_.default_ee_orientation[3];

    if (!current_robot_config_.q_home_rad.empty() && num_joints_ >= current_robot_config_.q_home_rad.size()) {
        q_home = Eigen::VectorXd::Zero(num_joints_);
        for (size_t i = 0; i < current_robot_config_.q_home_rad.size(); ++i) {
            q_home(i) = current_robot_config_.q_home_rad[i];
        }
    }
    q_target_ = q_home;

    namespace fs = std::filesystem;
    std::string urdf_file = fs::current_path().string() + "/src/" + current_robot_config_.urdf_path;

    // Create the per-robot kinematics solver (robots_config.yaml: kinematics_solver,
    // default "pinocchio"; "none" = deliberate degraded joint-space-only mode).
    const bool solver_requested =
        !current_robot_config_.kinematics_solver.empty() &&
        current_robot_config_.kinematics_solver != "none";
    // The IK worker must not use solver_ while it is replaced and configured below.
    std::unique_lock<std::mutex> solver_lock(solver_mutex_);
    {
        std::string factory_err;
        solver_ = fic::createKinematicsSolver(current_robot_config_.kinematics_solver, &factory_err);
        if (!solver_ && !factory_err.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Kinematics solver: %s", factory_err.c_str());
        }
    }

    if (solver_ && !solver_->loadModel(urdf_file)) {
        RCLCPP_ERROR(this->get_logger(), "FATAL: Failed to load URDF: %s",
                     solver_->lastError().c_str());
        solver_.reset();
    }

    // Tell the solver which joints are actuated, in control order, so
    // lowerPositionLimit(i)/upperPositionLimit(i) can map an actuated index
    // onto the configuration vector. Without it PinocchioSolver indexed q
    // directly, which only matches when nq == nv: a `continuous` joint takes
    // two entries in q (cos, sin), so every bounded joint after the first one
    // reads a neighbour's limit -- or a +-1 cos/sin bound interpreted as
    // radians. Those limits feed the latching hard-fault check below, so a
    // wrong value makes the arm go limp at a legal pose. This URDF declares
    // its continuous joints as `revolute`, so nq == nv and it was unaffected;
    // Kinova's own GEN3-7DOF-VISION URDF gives nq=11, nv=7 and is not.
    if (solver_) {
        solver_->configureActuatedJoints(current_robot_config_.joint_names);
    }

    // ── Closed-loop / parallel mechanisms (e.g. the CSPM) ──
    //
    // Hand the solver its loop closures + EE so it presents the REDUCED
    // actuated-joint interface. loadModel() deliberately leaves configured_ =
    // false and, by the solver's own contract, "behave[s] as a plain serial
    // solver" until this call arrives: the loop closures are ignored and nq()/
    // nv() report the FULL model, which num_joints_ below then adopts.
    //
    // That failure is SILENT — loadModel() succeeds, isLoaded() is true, no
    // error is logged — so the node would run a parallel mechanism on
    // unconstrained serial kinematics and nothing would say so. fic_simulation
    // has always made this call (fic_control.cpp); hardware was
    // explicitly deferred (2026-07-27-kinova-cspm-wrist-design.md:16,180), which
    // is why it was missing here rather than broken.
    //
    // MUST run before num_joints_ / nq are read below — they are derived from
    // solver_->nv() / nq(), and those change meaning once configured.
    //
    // Guarded on loop_closures being non-empty, and the base-class method is a
    // documented no-op anyway, so every serial robot is completely unaffected.
    if (solver_ && !current_robot_config_.loop_closures.empty()) {
        std::vector<fic::KinematicsSolver::LoopClosure> loops;
        loops.reserve(current_robot_config_.loop_closures.size());
        for (const auto& lc : current_robot_config_.loop_closures) {
            fic::KinematicsSolver::LoopClosure L;
            L.bodyA = lc.bodyA;
            L.bodyB = lc.bodyB;
            L.posA  = Eigen::Vector3d(lc.posA[0], lc.posA[1], lc.posA[2]);
            L.posB  = Eigen::Vector3d(lc.posB[0], lc.posB[1], lc.posB[2]);
            loops.push_back(L);
        }
        solver_->configureClosedLoop(loops, current_robot_config_.joint_names,
                                     ee_frame_name_,
                                     current_robot_config_.ee_orientation_only);

        RCLCPP_INFO(this->get_logger(),
            "  Closed-loop: %zu loop closures, EE '%s'%s (reduced nq=%d nv=%d)",
            loops.size(), ee_frame_name_.c_str(),
            current_robot_config_.ee_orientation_only ? ", orientation-only" : "",
            solver_->nq(), solver_->nv());

        // A closed-loop solver that did NOT take the configuration is the exact
        // silent-serial-fallback case above. Say so loudly: the numbers it goes
        // on to produce would be plausible and wrong.
        if (static_cast<size_t>(solver_->nv()) != current_robot_config_.joint_names.size()) {
            RCLCPP_ERROR(this->get_logger(),
                "Closed-loop configuration did NOT take for '%s': solver reports nv=%d "
                "but %zu actuated joints are configured.\n"
                "  The solver is still in plain-serial mode and its Jacobians, FK and "
                "gravity torques do NOT describe this mechanism.\n"
                "  Check that joint_names %s, ee_frame_name '%s' and every loop-closure "
                "body name exist in the URDF.",
                robot_name.c_str(), solver_->nv(),
                current_robot_config_.joint_names.size(),
                "in robots_config.yaml", ee_frame_name_.c_str());
            is_robot_supported_ = false;
            return;
        }

        // ── Gravity feedforward is ZERO for a configured closed-loop solver ──
        //
        // ClosedLoopPinocchioSolver::gravity() returns Zero(n_act) once
        // configured -- a deliberate simplification whose stated rationale is
        // the light CSPM in SIMULATION, where MuJoCo publishes gravity torques
        // on a separate topic. There is no such publisher on hardware: the block
        // below states that the solver is the ONLY producer of gravity_torques_
        // here, which is why a missing solver is refused outright.
        //
        // So a configured closed-loop robot silently gets what that refusal
        // exists to prevent: zero gravity feedforward, and a latched hard fault
        // that commands zero torque. For the CSPM alone that is defensible --
        // three light legs on a fixed centre of rotation go slack, they do not
        // collapse. For a closed-loop mechanism carried by a heavy serial arm
        // (KinovaGen3_CSPM, 10 joints, gravity_compensated: false) it is NOT:
        // that arm would drop.
        //
        // Warn rather than refuse: refusing would block the CSPM, which is safe,
        // and this must not be discovered at fault time.
        if (!current_robot_config_.gravity_compensated) {
            RCLCPP_WARN(this->get_logger(),
                "'%s' uses a closed-loop solver, whose gravity() returns ZERO on the "
                "actuated coordinates -- so gravity feedforward is zero and a latched "
                "HARD FAULT would command ZERO TORQUE.\n"
                "  Acceptable for a light, balanced mechanism (it goes slack).\n"
                "  NOT acceptable if this robot has to hold its own weight up: it "
                "would drop.\n"
                "  To fix properly, implement constrained gravity in "
                "ClosedLoopPinocchioSolver::gravity(); to silence it deliberately, "
                "set gravity_compensated: true only if zero really is the correct "
                "fault output for this robot.",
                robot_name.c_str());
        }
    }

    if (!solver_ && solver_requested) {
        // A solver was requested but is unavailable/failed — preserve the old
        // "failed URDF load" semantics and mark the robot unsupported.
        is_robot_supported_ = false;
        return;
    }

    if (!solver_) {
        RCLCPP_WARN(this->get_logger(),
            "Running WITHOUT a kinematics solver (kinematics_solver: '%s').\n"
            "  Available:   joint-space control modes 1-4, driver bridge, torque guards\n"
            "  Disabled:    Cartesian modes 5-8, IK targets, EE pose publishing,\n"
            "               Jacobian wrench estimate, gravity feedforward,\n"
            "               Cartesian workspace guard (fail-open) and virtual walls\n"
            "  To restore:  install pinocchio and rebuild, or register a custom solver\n"
            "               (see docs/custom-kinematics-solver.md)",
            current_robot_config_.kinematics_solver.c_str());
    }

    // ── Fail at LOAD time, not at FAULT time ──
    //
    // On hardware the ONLY producer of gravity_torques_ is the kinematics
    // solver (controlLoop: `gravity_torques_ = solver_->gravity(q_current_)`).
    // fic_simulation additionally receives MuJoCo's gravity torques on a topic;
    // there is no such publisher here. So with no solver, gravity_torques_ is
    // permanently zero — and three paths quietly become "command nothing":
    //
    //   * the mode 0 idle baseline,
    //   * the mode 1/2 gravity feedforward,
    //   * and above all the LATCHED HARD FAULT, whose whole purpose is to hold
    //     the robot up on gravity compensation while everything else stops.
    //
    // A robot that cannot produce the compensation its own fault path depends
    // on must not be allowed into torque mode at all: for a 50 kg quadruped a
    // hard fault would be a collapse, discovered at the worst possible moment.
    // Refusing here surfaces it as a startup error instead.
    //
    // Robots whose DRIVER compensates gravity internally (gravity_compensated:
    // true, e.g. Franka_FR3) are deliberately exempt: zero really is the correct
    // fault output for them, which is exactly what the fault path already emits
    // in that case. This keeps the documented degraded joint-space-only mode
    // usable wherever it is genuinely safe, and only closes it where it is not.
    if (!solver_ && !current_robot_config_.gravity_compensated) {
        is_robot_supported_ = false;
        RCLCPP_ERROR(this->get_logger(),
            "REFUSING to enable torque control for '%s': kinematics_solver is '%s' "
            "(no solver) but gravity_compensated is false, so this node is expected "
            "to supply gravity feedforward and has no way to compute it.\n"
            "  gravity_torques_ would stay zero, and the joint-limit HARD FAULT "
            "path would therefore command ZERO TORQUE — the robot would drop.\n"
            "  Fix ONE of:\n"
            "    * set kinematics_solver to a working solver (e.g. pinocchio), or\n"
            "    * set gravity_compensated: true if the vendor driver really does "
            "compensate gravity itself (its fault output of 0 Nm is then correct).",
            robot_name.c_str(), current_robot_config_.kinematics_solver.c_str());
        return;
    }

    num_joints_ = solver_ ? static_cast<size_t>(solver_->nv())
                          : static_cast<size_t>(current_robot_config_.num_joints);
    const int nq = solver_ ? solver_->nq() : static_cast<int>(num_joints_);
    // Publish the active joint count into shared memory so vendor driver
    // bridges can size their own loops against it instead of hardcoding 7.
    publishJointCount(static_cast<uint32_t>(num_joints_));
    q_current_ = Eigen::VectorXd::Zero(nq);
    // Preserve the q_home target set earlier — zeroing here previously caused a
    // large startup error in Mode 1 (Franka drove all joints toward 0 from home,
    // tripping collision/velocity faults).
    if (q_home.size() == nq) {
        q_target_ = q_home;
    } else {
        q_target_ = Eigen::VectorXd::Zero(nq);
    }
    q_target_ik_ = Eigen::VectorXd::Zero(nq);

    resizeVectors(num_joints_);

    // Install robot-level joint metadata (continuity + limits) once per robot load,
    // sourced from the solver's model (URDF). Cartesian modes skip initJointPlanner,
    // so this is the only path that sets it. Continuous (unbounded) joints in the URDF
    // get Pinocchio's huge sentinel (~1e19) — anything above 1e6 rad is effectively
    // infinite, which correctly excludes multi-turn finite joints (e.g. ±720°).
    {
        // Continuity from URDF (authoritative — missing <limit> lower/upper = continuous).
        // Bounded-joint clamp limits from the solver's model, or straight from the
        // URDF XML in degraded mode.
        const size_t n = num_joints_;
        std::vector<bool> continuous =
            fic::deriveJointContinuityFromUrdf(urdf_file, current_robot_config_.joint_names);
        continuous.resize(n, false);
        // Kept for the feedback filter, which must unwrap exactly these.
        joint_continuous_ = continuous;

        std::vector<double> lower(n), upper(n);
        std::vector<double> urdf_lower, urdf_upper;
        bool have_urdf_limits = false;
        if (!solver_) {
            have_urdf_limits = fic::deriveJointLimitsFromUrdf(
                urdf_file, current_robot_config_.joint_names, urdf_lower, urdf_upper);
            if (!have_urdf_limits) {
                RCLCPP_WARN(this->get_logger(),
                    "No solver and URDF limits unreadable — falling back to "
                    "robots_config.yaml joint_limits_deg, then ±2π");
            }
        }
        for (size_t i = 0; i < n; ++i) {
            if (continuous[i]) {
                lower[i] = -M_PI;
                upper[i] =  M_PI;
            } else if (solver_) {
                lower[i] = solver_->lowerPositionLimit(i);
                upper[i] = solver_->upperPositionLimit(i);
            } else if (have_urdf_limits && i < urdf_lower.size()) {
                lower[i] = urdf_lower[i];
                upper[i] = urdf_upper[i];
            } else if (i < current_robot_config_.joint_limits_lower_deg.size() &&
                       i < current_robot_config_.joint_limits_upper_deg.size()) {
                lower[i] = current_robot_config_.joint_limits_lower_deg[i] * M_PI / 180.0;
                upper[i] = current_robot_config_.joint_limits_upper_deg[i] * M_PI / 180.0;
            } else {
                lower[i] = -2.0 * M_PI;
                upper[i] =  2.0 * M_PI;
            }
            RCLCPP_INFO(this->get_logger(),
                "  Joint %zu: limits=[%.4f, %.4f] → %s",
                i, lower[i], upper[i], continuous[i] ? "CONTINUOUS" : "bounded");
        }
        fic_ctrl_.setJointLimits(lower, upper);
        fic_ctrl_.setJointContinuous(continuous);

        // Everything an inner-loop MCU needs to run the same joint layer.
        mcu_limits_ = fic_hw::McuBackend::RobotLimits{};
        mcu_limits_.n = n;
        mcu_limits_.lower = lower;
        mcu_limits_.upper = upper;
        mcu_limits_.continuous = continuous;
        for (size_t i = 0; i < n; ++i) {
            if (i < current_robot_config_.joint_limits_lower_deg.size() &&
                i < current_robot_config_.joint_limits_upper_deg.size()) {
                mcu_limits_.fault_lower.push_back(current_robot_config_.joint_limits_lower_deg[i] * M_PI / 180.0);
                mcu_limits_.fault_upper.push_back(current_robot_config_.joint_limits_upper_deg[i] * M_PI / 180.0);
            }
        }
        if (mcu_limits_.fault_lower.size() != n) {   // partial lists: no hard-fault limits
            mcu_limits_.fault_lower.clear();
            mcu_limits_.fault_upper.clear();
        }
    }

    // Per-joint torque limits from robots_config.yaml (physical actuator limits)
    per_joint_torque_limit_ = current_robot_config_.joint_torque_max;
    hard_fault_ = false;

    // Apply per-robot admittance deadbands loaded from robots_config.yaml; the Cartesian force one is
    // the arm's (cart_force_hardware when set: the joint-torque force estimate reads friction and the
    // arm's own motion as several N, which the task-space admittance would otherwise integrate).
    // The joint target speed of the joint modes (safety_zones.velocity.joint_target_speed); the Studio's
    // velocity limits may change it at run time (their 4th value).
    if (current_robot_config_.safety_joint_target_speed) {
        joint_slew_.speed = std::max(0.0, *current_robot_config_.safety_joint_target_speed);
        RCLCPP_INFO(this->get_logger(), "Joint target speed of the joint modes: %.2f rad/s", joint_slew_.speed);
    }
    // On the arm, and simulating it (hardware model on), the Cartesian deadbands are the arm's; the ideal
    // simulator keeps the robot's plain ones. H-PMP's split drive: the arm's deadbands, or on the ideal
    // simulator the library's narrower 1 N / 0.1 N m (its measured torque has no sensor bias to absorb).
    const bool arm_like = !sim_ || current_robot_config_.hardware_model.enabled;
    const double cart_force_db = arm_like ? current_robot_config_.cartForceDeadbandOnArm()
                                          : current_robot_config_.deadband_cart_force;
    const double cart_torque_db = arm_like ? current_robot_config_.cartTorqueDeadbandOnArm()
                                           : current_robot_config_.deadband_cart_torque;
    fic_ctrl_.setDeadbands(current_robot_config_.deadband_joint_tau, cart_force_db, cart_torque_db);
    const double split_force_db = arm_like ? cart_force_db : 1.0;
    const double split_torque_db = arm_like ? cart_torque_db : 0.1;
    fic_ctrl_.setHpmpSplitDrive(true, split_force_db, split_torque_db);
    mcu_limits_.task_db_force = split_force_db;     // the same, in the inner loop
    mcu_limits_.task_db_torque = split_torque_db;
    // How far a push may move the goal in the Cartesian admittance modes (robots_config.yaml
    // admittance_limits; the library's defaults when the block is absent).
    fic_ctrl_.setCartesianAdmittanceLimits(
        current_robot_config_.cart_adm_push_m, current_robot_config_.cart_adm_push_rad,
        current_robot_config_.cart_adm_haptic_m, current_robot_config_.cart_adm_haptic_rad);
    // The FIC law options (robots_config.yaml fic_law; the library's defaults when absent).
    fic_ctrl_.setLawOptions(current_robot_config_.restart_on_stall,
                            current_robot_config_.planner_max_hz, current_robot_config_.planner_speed_margin);
    // The joint planners never plan faster than the max joint speed (when velocity limits are on).
    fic_ctrl_.setJointSpeedLimit(velocity_limits_enabled_ ? max_joint_velocity_ : 0.0);
    RCLCPP_INFO(this->get_logger(),
        "H-PMP admittance drive: split, task deadband %.2f N / %.3f N m, joint deadband %.2f N m; "
        "Cartesian admittance deadband %.2f N / %.2f N m",
        fic_ctrl_.hpmp_task_db_force_, fic_ctrl_.hpmp_task_db_torque_, fic_ctrl_.deadband_joint_tau_,
        fic_ctrl_.deadband_cart_force_, fic_ctrl_.deadband_cart_torque_);

    // Same clamp as the standard output guard: per-joint limits, or the
    // scalar fallback when the robot config has none.
    mcu_limits_.torque_limit = per_joint_torque_limit_;
    if (mcu_limits_.torque_limit.empty())
        mcu_limits_.torque_limit.assign(num_joints_, output_torque_limit_);
    mcu_limits_.deadband_joint_tau = current_robot_config_.deadband_joint_tau;
    mcu_limits_.torque_sign = current_robot_config_.torque_sign;
    mcu_limits_.damping_alpha = damping_alpha_;
    mcu_limits_.joint_speed_limit = velocity_limits_enabled_ ? max_joint_velocity_ : 0.0;
    mcu_limits_.torque_cutoff_hz = torque_cutoff_hz_;
    mcu_limits_.state_cutoff_hz = kStateCutoffHz;
    mcu_limits_.dt = samplePeriod();
    wall_torque_.assign(num_joints_, 0.0);
    mcu_.reset();
    mcu_armed_ = false;
    standard_driver_launched_ = false;
    selectInnerLoop("robot load");

    initControllerPLanner = false;
    robot_selection_received_ = true;

    RCLCPP_INFO(this->get_logger(), "Loaded %s (nq=%d, nv=%zu, solver: %s). EE: %s, Elbow: %s",
        robot_name.c_str(), nq, num_joints_,
        solver_ ? solver_->name().c_str() : "none",
        ee_frame_name_.c_str(), elbow_frame_name_.c_str());
}

template <typename T>
Eigen::VectorXd FicControlNode<T>::solveFullIterativeIK(
    const Eigen::VectorXd& q_start, const geometry_msgs::msg::Pose& ee_target_des)
{
    if (!solver_ || !solver_->isLoaded()) return q_start;
    if (!solver_->hasFrame(ee_frame_name_)) {
        RCLCPP_ERROR(this->get_logger(), "IK: Frame %s not found.", ee_frame_name_.c_str());
        return q_start;
    }

    fic::KinematicsSolver::IKOptions opts;
    opts.tolerance          = 1e-4;
    opts.max_iterations     = 1000;
    opts.dt                 = 1e-2;
    opts.damping            = 1e-6;
    opts.joint_based        = true;   // joint-based CLIK (parent joint anchor)
    opts.max_joint_velocity = 0.0;

    // Convergence success is intentionally not checked here (matches the
    // pre-refactor behavior: the best iterate is used either way).
    return solver_->solveIK(q_start, poseMsgToIso(ee_target_des), ee_frame_name_, opts).q;
}

template <typename T>
void FicControlNode<T>::readSharedFeedback() {
    if (!shared_feedback) return;
    if (!is_robot_supported_ || num_joints_ == 0) return;

    // MCU path: the MCU is this robot's driver. Put its latest raw arm sample
    // where a vendor driver would, then read it exactly as before.
    if (use_mcu_ && mcu_.configured() && !mcu_.mirrorFeedback(shared_feedback, 0.1)) {
        const double age = mcu_.lastStateAge();
        if (age > 1e5)
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "No STATE from the inner loop (%s) yet — no torque is commanded until it reports the arm.",
                innerLoopWhere());
        else
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "No fresh STATE from the inner loop (%s; last %.0f ms ago) — it holds the arm on its own; "
                "outer loops run on the last sample.", innerLoopWhere(), age * 1e3);
    }
    // Simulation: the filters run at the simulator's rate, once per new sample (one STATE per physics
    // step), as the old fic_simulation filtered once per joint_states message.
    if (sim_) {
        const fic::link::StateMsg& st = mcu_.lastState();
        if (st.tick == last_state_tick_) return;
        last_state_tick_ = st.tick;
        // The simulator's own step, as the inner loop reports it (the scene's timestep, or an experiment's
        // sim_dt): the filters run at it. A change re-primes them from the next sample.
        if (st.dt > 0.0f && std::fabs(st.dt - 1.0 / filter_rate_hz_) > 1e-7) {
            sim_dt_ = st.dt;
            filter_rate_hz_ = 1.0 / sim_dt_;
            mcu_limits_.dt = sim_dt_;
            updateDampingAlpha();
            feedback_received_ = false;
            RCLCPP_INFO(this->get_logger(), "Simulator step %.4f s: the feedback filters run at %.0f Hz",
                        sim_dt_, filter_rate_hz_);
            return;
        }
    }

    const int loop_limit = std::min((int)num_joints_, (int)kMaxJoints);

    // --- Pass 1: pull the raw samples and decide whether they are genuine ---
    //
    // initSharedMemory() zeroes joint_positions[]/torque[] and there is no
    // validity sentinel on them (only commanded_torque gets a NaN one), so
    // "all slots still bit-exactly 0.0f" is precisely the state that means
    // "the vendor driver has not written anything yet". A real encoder does not
    // report a bit-exact 0.0f on every joint at once, so the first genuine
    // sample latches this and it never un-latches for the session.
    bool any_nonzero = false;
    for (int i = 0; i < loop_limit; ++i) {
        float raw_t = shared_feedback->torque[i].load(std::memory_order_acquire);
        float raw_q = shared_feedback->joint_positions[i].load(std::memory_order_acquire);

        if (!std::isfinite(raw_t)) raw_t = 0.0f;
        if (!std::isfinite(raw_q)) raw_q = 0.0f;

        if (raw_t != 0.0f || raw_q != 0.0f) any_nonzero = true;

        raw_torques_scratch_[i]   = (double)raw_t;
        raw_joint_positions_[i]   = (double)raw_q;
    }
    for (size_t i = (size_t)loop_limit; i < num_joints_; ++i) {
        raw_torques_scratch_[i] = 0.0;
        raw_joint_positions_[i] = 0.0;
    }

    if (!feedback_received_ && !any_nonzero) {
        // Still the zero-fill from initSharedMemory(). Leave every derived
        // buffer at its zero state and do NOT step the filters: letting them
        // integrate ~100 ms of fictitious zeros is what used to make the
        // joint-limit hard fault latch against a position the robot was never
        // at. controlLoop() gates on feedback_received_ and commands no torque
        // until this clears.
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Waiting for joint feedback from the inner loop (%s) — no torque will be commanded.%s",
            innerLoopWhere(), (use_mcu_ && mcu_.local() && !sim_)
                ? (" Start the vendor driver for '" + selected_robot_ + "'.").c_str() : "");

        // Keep publishing the (zero) state exactly as before, so downstream
        // subscribers see the same topics at the same rate they always did.
        std_msgs::msg::Float64MultiArray q_msg_idle;
        q_msg_idle.data = joint_positions_;
        q_current_publisher_->publish(q_msg_idle);
        std_msgs::msg::Float64MultiArray vel_msg_idle;
        vel_msg_idle.data.assign(currentVelocities_.begin(), currentVelocities_.end());
        velocity_publisher_->publish(vel_msg_idle);
        return;
    }

    // --- First genuine sample: prime everything from it, not from zero ---
    if (!feedback_received_) {
        for (int i = 0; i < loop_limit; ++i) {
            torque_filters_[i]   = ButterworthFilter(torque_cutoff_hz_, filter_rate_hz_,
                                                     raw_torques_scratch_[i]);
            position_filters_[i] = ButterworthFilter(kStateCutoffHz, filter_rate_hz_,
                                                     raw_joint_positions_[i]);
            raw_unwrapped_[i]    = raw_joint_positions_[i];
            prev_raw_wrapped_[i] = raw_joint_positions_[i];
            // velocity_filters_ is not primed: unlike fic_simulation, this node
            // never steps that bank (velocity comes from finite differences),
            // so it is already at the zero resizeVectors() gave it.
        }
        // No previous sample exists, so there is no finite difference to take
        // this cycle — clearing these makes both velocity paths skip it rather
        // than differentiate a step out of the fictitious zero.
        prev_joint_positions_.clear();
        prev_raw_joint_positions_.clear();
        std::fill(currentVelocities_.begin(), currentVelocities_.end(), static_cast<T>(0.0));
        std::fill(rawVelocities_.begin(), rawVelocities_.end(), static_cast<T>(0.0));
        std::fill(dampingVelocities_.begin(), dampingVelocities_.end(), static_cast<T>(0.0));
        feedback_received_ = true;
        if (use_mcu_) pending_target_snap_ = true;
        RCLCPP_INFO(this->get_logger(),
            "Joint feedback received from the inner loop (%s) — state filters primed, control enabled.",
            innerLoopWhere());
    }

    // --- Pass 2: filter ---
    for (int i = 0; i < loop_limit; ++i) {
        joint_torques_[i]   = current_robot_config_.torque_sign * torque_filters_[i].filter(raw_torques_scratch_[i]);
        if (isContinuousJoint(static_cast<size_t>(i))) {
            // Carry the angle across +-pi, filter that, and wrap only the
            // result. The output then crosses +-pi as a clean 2*pi step --
            // harmless, since FK and the continuous-joint controllers are
            // periodic -- instead of sweeping through every angle between.
            raw_unwrapped_[i] += wrapAngle(raw_joint_positions_[i] - prev_raw_wrapped_[i]);
            prev_raw_wrapped_[i] = raw_joint_positions_[i];
            joint_positions_[i] = wrapAngle(position_filters_[i].filter(raw_unwrapped_[i]));
        } else {
            joint_positions_[i] = position_filters_[i].filter(raw_joint_positions_[i]);
        }
    }

    // Tail beyond what the shared memory reserves — keep these zero so
    // downstream math doesn't pick up garbage if num_joints_ > kMaxJoints
    // (which also logs a warning from publishJointCount at init).
    for (size_t i = (size_t)loop_limit; i < num_joints_; ++i) {
        joint_torques_[i] = 0.0;
        joint_positions_[i] = 0.0;
    }

    q_current_ = Eigen::Map<Eigen::VectorXd>(joint_positions_.data(), num_joints_);

    // MCU path: the MCU engages torque as soon as Start has run and feedback
    // flows, with whatever target this node holds — the startup home pose
    // unless told otherwise. Hold where the arm IS instead: a robot parked far
    // from home must not be pulled there by pressing Start.
    if (pending_target_snap_ && feedback_received_) {
        q_target_ = q_current_;
        q_target_ik_ = q_current_;
        q_postural_ = q_current_;
        initControllerPLanner = false;               // planners re-seed here too
        pending_target_snap_ = false;
        RCLCPP_INFO(this->get_logger(), "Inner loop: joint targets set to the arm's current pose (hold here).");
    }

    // Compute joint velocities via finite differences (1 kHz loop → dt = 0.001)
    if (prev_joint_positions_.size() == num_joints_) {
        for (size_t i = 0; i < num_joints_; ++i) {
            currentVelocities_[i] = jointDelta(i, joint_positions_[i], prev_joint_positions_[i]) / 0.001;
        }
    }
    prev_joint_positions_ = joint_positions_;

    // Damping-path velocity: finite difference of the RAW positions through a
    // dedicated FIRST-order lag. currentVelocities_ above is deliberately left
    // alone — it feeds the published velocity topic and the Cartesian velocity
    // estimate, where smoothness matters and lag does not. The output damping
    // term has the opposite requirement (see kDampingCutoffHz).
    if (prev_raw_joint_positions_.size() == num_joints_) {
        for (size_t i = 0; i < num_joints_; ++i) {
            // Wrapped for continuous joints: across +-pi the plain
            // difference is 2*pi in one 1 ms period, 6283 rad/s, straight
            // into the damping torque.
            const double v_raw =
                jointDelta(i, raw_joint_positions_[i], prev_raw_joint_positions_[i]) / samplePeriod();
            rawVelocities_[i] = static_cast<T>(v_raw);
            if (damping_alpha_ <= 0.0) {
                dampingVelocities_[i] = static_cast<T>(v_raw);
            } else {
                dampingVelocities_[i] = static_cast<T>(
                    damping_alpha_ * static_cast<double>(dampingVelocities_[i]) +
                    (1.0 - damping_alpha_) * v_raw);
            }
        }
    }
    prev_raw_joint_positions_ = raw_joint_positions_;

    std_msgs::msg::Float64MultiArray q_msg;
    q_msg.data = joint_positions_;
    q_current_publisher_->publish(q_msg);

    // Same cycle, unfiltered. See the declaration for why the twin needs this.
    std_msgs::msg::Float64MultiArray q_raw_msg;
    q_raw_msg.data.assign(raw_joint_positions_.begin(),
                          raw_joint_positions_.begin() + num_joints_);
    q_current_raw_publisher_->publish(q_raw_msg);

    // The target the controller is tracking this cycle, so a consumer can
    // form the tracking error. See the declaration.
    if ((size_t)q_target_.size() >= num_joints_) {
        std_msgs::msg::Float64MultiArray q_tgt_msg;
        q_tgt_msg.data.assign(q_target_.data(),
                              q_target_.data() + num_joints_);
        q_target_publisher_->publish(q_tgt_msg);
    }

    // Whichever target the active mode actually tracks. Modes 1-4 are
    // joint-space and follow q_target_; the Cartesian modes follow the IK
    // solution in q_target_ik_. A consumer asking "is the arm sitting on
    // its commanded pose?" needs this one, not whichever happens to be
    // named q_target.
    {
        const Eigen::VectorXd& active =
            (current_control_mode_ >= 5) ? q_target_ik_ : q_target_;
        if ((size_t)active.size() >= num_joints_) {
            std_msgs::msg::Float64MultiArray q_act_msg;
            q_act_msg.data.assign(active.data(), active.data() + num_joints_);
            q_target_active_publisher_->publish(q_act_msg);
        }
    }

    // Publish only on the live path, never the idle path above: a consumer
    // must be able to tell "no contact" (a genuine zero) from "no data".
    std_msgs::msg::Float64MultiArray tau_msg;
    tau_msg.data.assign(joint_torques_.begin(),
                        joint_torques_.begin() + num_joints_);
    torque_publisher_->publish(tau_msg);

    std_msgs::msg::Float64MultiArray vel_msg;
    vel_msg.data.assign(currentVelocities_.begin(), currentVelocities_.end());
    velocity_publisher_->publish(vel_msg);
}

// Measured gripper state from the driver's gripper slot in shared memory
// (shared_params.h, kGripperFeedbackSlot): {rid}_gripper_state, data =
// [width_m (0 = closed), motor_current]. Called once per control cycle after
// feedback_received_; publishes every kGripperStateDecimation cycles (~50 Hz
// at the 1 kHz loop). Nothing is published until the driver has written a
// real sample, so subscribers can tell "no gripper feedback" (sim, an arm
// without a gripper, a driver built before the slot existed) from a reading.
template <typename T>
void FicControlNode<T>::publishGripperState() {
    constexpr unsigned kGripperStateDecimation = 20;
    if (!shared_feedback || !gripper_state_publisher_) return;
    // On a robot this big the slot is a real joint, not a gripper.
    if (num_joints_ >= kGripperFeedbackSlot) return;
    if (++gripper_state_tick_ < kGripperStateDecimation) return;
    gripper_state_tick_ = 0;

    // alive drops when the driver runs cleanupSharedMemory() on exit, which
    // zeroes the slot too; publishing that as "closed, no current" would be
    // a lie. Go quiet and re-arm the latch for the next driver session.
    if (shared_feedback->alive.load(std::memory_order_acquire) == 0) {
        gripper_feedback_seen_ = false;
        return;
    }

    const float width = shared_feedback->joint_positions[kGripperFeedbackSlot]
                            .load(std::memory_order_acquire);
    const float current = shared_feedback->torque[kGripperFeedbackSlot]
                              .load(std::memory_order_acquire);
    if (!std::isfinite(width) || !std::isfinite(current)) return;

    // The zero pair initSharedMemory() leaves behind means "no gripper
    // feedback" until the driver has written one real sample; after that a
    // zero pair is a genuine reading (fully closed, idle motor).
    if (!gripper_feedback_seen_) {
        if (width == 0.0f && current == 0.0f) return;
        gripper_feedback_seen_ = true;
        RCLCPP_INFO(this->get_logger(),
            "Gripper feedback received — publishing %s.",
            gripper_state_publisher_->get_topic_name());
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data = {static_cast<double>(width), static_cast<double>(current)};
    gripper_state_publisher_->publish(msg);
}

template <typename T>
void FicControlNode<T>::initJointControllers(
    size_t n_j, const Eigen::VectorXd& q_curr,
    const std::vector<fic::ScalarParams>& scalars, const std::vector<fic::ListParams>& lists,
    const std::vector<fic::PlannerParams>& planners, const std::vector<fic::AdmittanceParams>& admittance)
{
    for (int i = 0; i < (int)n_j; ++i) {
        fic_ctrl_.initJointController(i, scalars[i], lists[i]);
        fic_ctrl_.initJointPlanner(i, q_curr[i], planners[i]);
        fic_ctrl_.initJointAdmittance(i, lists[i], planners[i], admittance[i]);
    }
}

template <typename T>
void FicControlNode<T>::initCartesianControllers(
    const std::vector<double>& current_pose, const fic::CartesianAllParams& cart_params)
{
    for (int i = 0; i < 3; ++i) {
        const auto& axis = cart_params.pos_axes[i];
        fic_ctrl_.initCartesianController(i, axis);
        fic_ctrl_.initCartesianPlanner(i, current_pose[i], axis);
        fic_ctrl_.initCartesianAdmittance(i, axis);
    }
    for (int i = 3; i < 6; ++i) {
        fic_ctrl_.initCartesianController(i, cart_params.rot_axis);
        // Orientation admittance shares the rot_axis admittance block — same
        // {InvM, damping, dFlow, flow, pose} parameters for all three rotation
        // channels (cartesianAdmittanceControl re-clamps pose to rad limits).
        fic_ctrl_.initCartesianAdmittance(i, cart_params.rot_axis);
    }
    fic_ctrl_.initElbowController(cart_params_);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  ROS CALLBACKS
// ═══════════════════════════════════════════════════════════════════════════════

template <typename T>
void FicControlNode<T>::gripperWidthCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    if (!shared_feedback || !is_robot_supported_) return;
    shared_feedback->commanded_gripper_width.store(msg->data, std::memory_order_release);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                         "Gripper command sent: %.4f m", msg->data);
}

template <typename T>
void FicControlNode<T>::robotSelectionCallback(const std_msgs::msg::String::SharedPtr msg) {
    loadRobotModel(msg->data);
}

template <typename T>
void FicControlNode<T>::gravityCompCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_robot_config_.gravity_compensated = msg->data;
    RCLCPP_INFO(this->get_logger(), "gravity_compensated = %s", msg->data ? "true" : "false");
}

template <typename T>
void FicControlNode<T>::wrenchGravityCompCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_robot_config_.wrench_gravity_compensated = msg->data;
    RCLCPP_INFO(this->get_logger(), "wrench_gravity_compensated = %s", msg->data ? "true" : "false");
}

template <typename T>
void FicControlNode<T>::wrenchFromApiCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_robot_config_.wrench_from_api = msg->data;
    RCLCPP_INFO(this->get_logger(), "wrench_from_api = %s", msg->data ? "true" : "false");
}

template <typename T>
void FicControlNode<T>::wrenchExternalCallback(const geometry_msgs::msg::Wrench::SharedPtr msg) {
    // Store raw and stamped; the control loop does the deadband, the
    // saturation and the staleness fade, so one place decides what the arm
    // actually feels no matter who publishes here or how often.
    const double in[6] = {msg->force.x,  msg->force.y,  msg->force.z,
                          msg->torque.x, msg->torque.y, msg->torque.z};
    for (double v : in) {
        if (!std::isfinite(v)) {           // a NaN here would poison the torque command
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "wrench_external: non-finite value ignored");
            return;
        }
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (int i = 0; i < 6; ++i) external_wrench_[i] = in[i];
    external_wrench_stamp_ = this->now();
    if (!external_wrench_active_) {
        external_wrench_active_ = true;
        RCLCPP_INFO(this->get_logger(),
            "Reflected wrench active: limits %.1f N / %.1f Nm per axis (from "
            "robots_config.yaml), fades to zero %.0f ms after the stream stops.",
            current_robot_config_.teleop_wrench_max_force,
            current_robot_config_.teleop_wrench_max_torque,
            (kExtWrenchStaleS + kExtWrenchFadeS) * 1000.0);
    }
}

template <typename T>
void FicControlNode<T>::activeProfileCallback(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::string new_profile = msg->data.empty() ? "default" : msg->data;
    if (new_profile == active_profile_) return;
    active_profile_ = new_profile;
    tuning_reload_ = true;
    RCLCPP_INFO(this->get_logger(), "Active tuning profile: %s", active_profile_.c_str());
}

template <typename T>
void FicControlNode<T>::paramUpdateCallback(const std_msgs::msg::String::SharedPtr msg) {
    // refreshTuning() then reads config.xml once, off the control thread; the
    // controllers keep the old tuning until the new set is ready and switch to
    // it on the next tick.
    std::lock_guard<std::mutex> lock(data_mutex_);
    tuning_reload_ = true;
    RCLCPP_INFO(this->get_logger(), "Parameter update (%s); reloading tuning from config.xml",
                msg->data.c_str());
}

template <typename T>
void FicControlNode<T>::numberOfJointsCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Number of joints topic received: %d", msg->data);
}

template <typename T>
void FicControlNode<T>::stepPosturalTarget() {
    // Called from controlLoop with data_mutex_ held. Moves the postural
    // target toward the IK solution by at most kPosturalMaxRate per joint
    // per second actually passed -- the loop is not always at its nominal
    // rate -- continuous joints the short way round.
    const Eigen::Index n = q_target_ik_.size();
    const auto now = std::chrono::steady_clock::now();
    const bool first = postural_at_ == std::chrono::steady_clock::time_point{};
    double dt = std::chrono::duration<double>(now - postural_at_).count();
    postural_at_ = now;
    // The first step has no last one; after a pause, no more than 50 ms'
    // worth -- never less than the time passed, which froze the target.
    if (first || !(dt > 0.0)) dt = samplePeriod();
    dt = std::min(dt, 0.05);
    if (q_postural_.size() != n)
        q_postural_ = (q_current_.size() == n) ? q_current_ : q_target_ik_;
    const double max_step = kPosturalMaxRate * dt;
    for (Eigen::Index i = 0; i < n; ++i) {
        const size_t j = static_cast<size_t>(i);
        const double d = jointDelta(j, q_target_ik_[i], q_postural_[i]);
        q_postural_[i] += std::clamp(d, -max_step, max_step);
        if (isContinuousJoint(j)) q_postural_[i] = wrapAngle(q_postural_[i]);
    }
}

template <typename T>
void FicControlNode<T>::posturalEnableCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    postural_enabled_ = msg->data;
    if (postural_enabled_ && q_current_.size() > 0)
        q_postural_ = q_current_;   // start from where the arm is
    RCLCPP_INFO(this->get_logger(), "Postural control %s", postural_enabled_ ? "ENABLED" : "DISABLED");
}

template <typename T>
void FicControlNode<T>::elbowEnableCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    elbow_enabled_ = msg->data;
    if (elbow_enabled_) {
        elbow_target_pose_ = elbow_current_pose_;
    }
    RCLCPP_INFO(this->get_logger(), "Elbow control %s", elbow_enabled_ ? "ENABLED" : "DISABLED");
}

template <typename T>
void FicControlNode<T>::hpmpEnableCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (hpmp_enabled_ == msg->data) return;   // a repeat changes nothing
    hpmp_enabled_ = msg->data;
    initControllerPLanner = false;
    RCLCPP_INFO(this->get_logger(), "H-PMP %s", hpmp_enabled_ ? "ENABLED" : "DISABLED");
}

template <typename T>
void FicControlNode<T>::desiredWrenchCallback(const geometry_msgs::msg::Wrench::SharedPtr msg) {
    const std::vector<double> w = {msg->force.x, msg->force.y, msg->force.z,
                                    msg->torque.x, msg->torque.y, msg->torque.z};
    for (double v : w) {
        if (!std::isfinite(v)) {
            RCLCPP_WARN(this->get_logger(), "Desired wrench refused: non-finite value");
            return;
        }
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    desired_wrench_ = w;
    RCLCPP_INFO(this->get_logger(), "Desired wrench F = (%.2f, %.2f, %.2f) N, M = (%.2f, %.2f, %.2f) N m",
                w[0], w[1], w[2], w[3], w[4], w[5]);
}

template <typename T>
void FicControlNode<T>::admittancePlannerCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    admittance_use_planner_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Admittance planner %s", admittance_use_planner_ ? "ENABLED" : "DISABLED");
}

template <typename T>
void FicControlNode<T>::controlModeCallback(const std_msgs::msg::String::SharedPtr msg) {
    if (!is_robot_supported_) {
        RCLCPP_ERROR(this->get_logger(), "Cannot change mode: Robot not supported.");
        return;
    }

    if (msg->data == "Start") {
        // Same decision for every robot: an MCU that can drive it wins, otherwise the PC inner loop
        // (fic_inner_loop) with the vendor driver below. Once the vendor driver runs, no re-probe: it would
        // stall the control loop that is driving the arm.
        if (!standard_driver_launched_) selectInnerLoop("Start");
        if (use_mcu_ && !mcu_.local()) {
            startMcu();
            return;
        }
        if (use_mcu_ && mcu_.local()) {         // (re)configure the PC inner loop, then the driver below
            mcu_setup_done_ = true;
            pending_target_snap_ = true;
            if (sim_) return;                   // a simulated arm has no driver to start
        }
        if (!use_mcu_) {
            RCLCPP_ERROR(this->get_logger(), "Start refused: no inner loop answers (%s). The arm is driven only "
                         "through one: fic_inner_loop (robot_control_launch.py starts it) or the MCU.",
                         mcu_detail_.c_str());
            return;
        }
        if (current_robot_config_.driver_command.empty()) {
            RCLCPP_WARN(this->get_logger(), "No driver command configured for %s", selected_robot_.c_str());
            return;
        }

        std::string cmd = robotLibraryCommand(current_robot_config_.driver_command);

        std::string terminal_command = "gnome-terminal -- bash -c '" + cmd + "; exec bash'";
        RCLCPP_INFO(this->get_logger(), "Launching driver: %s", cmd.c_str());

        int result = std::system(terminal_command.c_str());
        if (result != 0) {
            RCLCPP_ERROR(this->get_logger(), "Driver launch failed with code: %d", result);
        } else {
            standard_driver_launched_ = true;
        }
        return;
    }

    int new_mode = -1;
    if      (msg->data == "Joint Impedance")               { new_mode = 1; }
    else if (msg->data == "Joint Impedance With Planner")   { new_mode = 2; }
    else if (msg->data == "Joint Admittance")               { new_mode = 3; }
    else if (msg->data == "Joint Admittance Haptic")        { new_mode = 4; }
    else if (msg->data == "Cartesian Impedance")            { new_mode = 5; }
    else if (msg->data == "Cartesian Planner")              { new_mode = 6; }
    else if (msg->data == "Cartesian Admittance")           { new_mode = 7; }
    else if (msg->data == "Cartesian Admittance Haptic")    { new_mode = 8; }
    else if (msg->data == "Stop Controller" || msg->data == "Pause Controller") {
        new_mode = 0;
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Unknown control mode: %s", msg->data.c_str());
        return;
    }

    // Cartesian modes (5-8) need FK/IK/Jacobians from the kinematics solver.
    if (new_mode >= 5 && (!solver_ || !solver_->isLoaded())) {
        RCLCPP_ERROR(this->get_logger(),
            "Control mode %d requires a kinematics solver — staying in mode %d. "
            "Set kinematics_solver in robots_config.yaml or rebuild with pinocchio.",
            new_mode, current_control_mode_);
        return;
    }

    const bool changed = new_mode != current_control_mode_;
    if (changed) {
        if (new_mode == 0) {
            RCLCPP_INFO(this->get_logger(), "Stopping controller — ramping down torques");
            is_ramping_down_ = true;
        } else {
            RCLCPP_INFO(this->get_logger(), "Switching to control mode: %s", msg->data.c_str());
            is_ramping_down_ = false;
            ramp_down_torques_.clear();
            if (shared_feedback)
                shared_feedback->stop_torque_mode.store(0, std::memory_order_release);
            initControllerPLanner = false;
            // The PC inner loop after a Stop: a mode resumes it, as on the old PC path (a driver that stays in
            // torque mode, e.g. libfranka, carries on; one that left, e.g. Kinova's, needs Start, which starts
            // it again). The MCU path keeps needing Start.
            if (use_mcu_ && mcu_.local() && !mcu_armed_) {
                mcu_setup_done_ = true;
                pending_target_snap_ = true;
            }
        }
        current_control_mode_ = new_mode;
    }

    // Snap targets to the robot's current state on a mode CHANGE so it
    // holds position instead of jumping to a stale target. The same mode sent
    // again (the GUI re-asserts it, tools re-send it) leaves the targets alone:
    // re-snapping on every message cancelled a target the arm was moving to.
    //
    // Only once real feedback exists. Before that q_current_ is the zero
    // fill from initSharedMemory(), so snapping would set every joint target
    // to 0 rad and the arm would lunge there the moment the driver entered
    // torque mode. The GUI re-asserts the mode ~5 s after launching this node,
    // which lands in exactly that window whenever the driver is still homing.
    // Skipping leaves the startup target (q_home) in place instead.
    if (changed && current_control_mode_ >= 1 && current_control_mode_ <= 8) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!feedback_received_) {
            RCLCPP_WARN(this->get_logger(),
                "Mode set before joint feedback — keeping the startup target "
                "instead of snapping to an unknown pose");
            return;
        }
        if (q_current_.size() > 0) {
            q_target_ = q_current_;
            q_target_ik_ = q_current_;
            q_postural_ = q_current_;
        }
        if (current_control_mode_ >= 5) {
            ee_target = ee_current;
            RCLCPP_INFO(this->get_logger(),
                "Cartesian mode: snapped EE target to current pose (%.3f, %.3f, %.3f)",
                ee_current.position.x, ee_current.position.y, ee_current.position.z);
        } else {
            RCLCPP_INFO(this->get_logger(),
                "Joint mode: snapped joint targets to current positions");
        }
    }
}

template <typename T>
void FicControlNode<T>::targetPositionCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    if (!is_robot_supported_) return;

    // NOTE: do NOT set initControllerPLanner = false here. A new EE target
    // doesn't change the controller structure or its parameters; at a 100 Hz
    // OT publish rate this used to re-init the controller on every message,
    // preventing it from ever settling. Re-init only fires from mode/profile/
    // robot-selection callbacks, which is what that flag was meant for.
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        ee_target = *msg;
        applyCartesianSafetyLimits(ee_target);
    }

    if (!solver_ || !solver_->isLoaded()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Cartesian target ignored — no kinematics solver (kinematics_solver: '%s')",
            current_robot_config_.kinematics_solver.c_str());
        return;
    }

    geometry_msgs::msg::Pose target_copy;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        target_copy = ee_target;
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Solving IK for new EE Target (clamped to safety zone)...");
    // Hand the newest target to the IK worker; never solve on the executor thread, where the
    // 1 kHz control loop runs (see ikWorker).
    {
        std::lock_guard<std::mutex> lk(ik_mutex_);
        ik_target_ = target_copy;
        ik_pending_ = true;
    }
    ik_cv_.notify_one();
}

template <typename T>
void FicControlNode<T>::ikWorker() {
    for (;;) {
        geometry_msgs::msg::Pose target;
        {
            std::unique_lock<std::mutex> lk(ik_mutex_);
            ik_cv_.wait(lk, [this] { return ik_pending_ || ik_stop_; });
            if (ik_stop_) return;
            target = ik_target_;
            ik_pending_ = false;
        }
        // Inputs under data_mutex_, the solve under solver_mutex_ alone, the result under
        // data_mutex_ again: never both at once (loadRobotModel takes data_mutex_ then
        // solver_mutex_, so holding solver_mutex_ while waiting for data_mutex_ could deadlock).
        Eigen::VectorXd q_start;
        std::vector<double> lower, upper;
        bool safety_on = false;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            q_start = hpmpIkSeed(fic::hpmpActive(current_control_mode_, hpmp_enabled_),
                                 q_current_, q_target_ik_);
            lower = joint_safety_lower_;
            upper = joint_safety_upper_;
            safety_on = joint_safety_enabled_;
        }
        Eigen::VectorXd q_result;
        geometry_msgs::msg::Pose calculated_elbow_pose;
        bool elbow_ok = false;
        {
            std::lock_guard<std::mutex> solver_lock(solver_mutex_);
            if (!solver_ || !solver_->isLoaded() || q_start.size() == 0) continue;
            // solveIK and framePose use their own scratch data, so they run beside the control loop.
            q_result = solveFullIterativeIK(q_start, target);
            Eigen::Isometry3d elb_iso;
            elbow_ok = solver_->framePose(q_result, elbow_frame_name_, elb_iso);
            if (elbow_ok) calculated_elbow_pose = isoToPoseMsg(elb_iso);
        }
        if (safety_on) {
            for (size_t i = 0; i < std::min((size_t)q_result.size(), lower.size()); ++i)
                q_result[i] = clamp(q_result[i], lower[i], upper[i]);
        }
        if (!elbow_ok) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "IK: Elbow frame '%s' not found.", elbow_frame_name_.c_str());
        }
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            q_target_ik_ = q_result;
            // Only refresh the elbow target from IK while hold is OFF. Once the
            // user enables elbow hold, elbowEnableCallback snapshots the current
            // elbow pose — letting IK overwrite it would defeat the hold.
            if (!elbow_enabled_ && elbow_ok) {
                elbow_target_pose_ = calculated_elbow_pose;
            }
        }
    }
}

template <typename T>
void FicControlNode<T>::desiredJointCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (!is_robot_supported_) return;   // (no shared-memory check: this node no longer maps the segment)
    size_t len = std::min((size_t)msg->data.size(), (size_t)num_joints_);

    // Re-init controllers so the planner replans toward the new target.
    // Skip re-init ONLY when admittance planner is disabled (teleop streaming)
    // — the admittance controller accumulates state that resetState() would
    // wipe, causing the robot to lose control.
    //
    // ...and skip it for pure Joint Impedance (mode 1) as well: that law reads
    // none of the planner/admittance state, it just tracks whatever q_target_
    // holds, so re-initialising per message is pure cost. This topic IS
    // streamed (GUI gamepad, joint_trajectory_player, autotuning engine,
    // experiment runner), and every clear makes controlLoop re-parse the
    // 340 KB config.xml once per joint plus a Cartesian pass, and reset the
    // fractal state — the 1 ms loop cannot hold its period through that.
    // Live re-tuning still reaches mode 1 via {rid}_param_update.
    // Mirrors fic_simulation's targetJointCallback fix.
    if (current_control_mode_ != 1 &&
        (admittance_use_planner_ || current_control_mode_ <= 2)) {
        initControllerPLanner = false;
    }

    // Is this a stream or a one-shot? Only a stream can be abandoned halfway,
    // so only a stream arms the control loop's staleness watchdog.
    const double now_s = this->now().seconds();
    if (now_s - prev_target_s_ <= kTargetStreamGapS) {
        if (target_stream_count_ < kTargetStreamMin) ++target_stream_count_;
    } else {
        target_stream_count_ = 1;
    }
    prev_target_s_ = now_s;
    last_target_s_.store(now_s, std::memory_order_release);
    if (target_stream_count_ >= kTargetStreamMin)
        target_stream_active_.store(true, std::memory_order_release);

    Eigen::VectorXd q_candidate = q_target_;
    for (size_t i = 0; i < len; ++i) {
        q_candidate(i) = msg->data[i];
    }
    applyJointSafetyLimits(q_candidate);
    // Workspace guard (joint mode): reject a target that would take the EE
    // outside the enabled Cartesian zone; otherwise accept it.
    if (eeTargetInsideZone(q_candidate) && bodyTargetClear(q_candidate)) q_target_ = q_candidate;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SAFETY CALLBACKS
// ═══════════════════════════════════════════════════════════════════════════════

template <typename T>
void FicControlNode<T>::cartesianSafetyCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 9) {
        cart_safety_limits_.x_min = msg->data[0];
        cart_safety_limits_.x_max = msg->data[1];
        cart_safety_limits_.x_enabled = msg->data[2] > 0.5;
        cart_safety_limits_.y_min = msg->data[3];
        cart_safety_limits_.y_max = msg->data[4];
        cart_safety_limits_.y_enabled = msg->data[5] > 0.5;
        cart_safety_limits_.z_min = msg->data[6];
        cart_safety_limits_.z_max = msg->data[7];
        cart_safety_limits_.z_enabled = msg->data[8] > 0.5;

        RCLCPP_INFO(this->get_logger(), "Updated Cartesian safety limits: X[%.2f, %.2f] Y[%.2f, %.2f] Z[%.2f, %.2f]",
            cart_safety_limits_.x_min, cart_safety_limits_.x_max,
            cart_safety_limits_.y_min, cart_safety_limits_.y_max,
            cart_safety_limits_.z_min, cart_safety_limits_.z_max);
    }
}

template <typename T>
void FicControlNode<T>::zonesCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    fic::safety::ZoneSet z;
    std::string why;
    if (!z.parse(msg->data, &why)) {
        RCLCPP_ERROR(this->get_logger(), "Safety zones refused (%s); keeping the ones in force", why.c_str());
        return;
    }
    std::lock_guard<std::mutex> lock(zones_mutex_);
    zones_ = z;
    zones_active_ = z.active();
    zone_target_safe_ = false;
    int enabled = 0;
    for (const auto& zone : z.zones) enabled += zone.enabled ? 1 : 0;
    RCLCPP_INFO(this->get_logger(), "Safety zones: %d of %zu on, the arm checked from joint %d (radius %.0f mm)",
                enabled, z.zones.size(), z.body.from_joint + 1, 1000.0 * z.body.radius);
}

// The frames along the arm the zones are checked at: the joints from from_joint, then the tool.
template <typename T>
std::vector<std::string> FicControlNode<T>::zoneBodyFrames(int from_joint) const {
    std::vector<std::string> frames;
    if (!solver_ || !solver_->isLoaded()) return frames;
    const auto& names = current_robot_config_.joint_names;
    for (size_t i = static_cast<size_t>(std::max(from_joint, 0)); i < names.size(); ++i)
        if (solver_->hasFrame(names[i])) frames.push_back(names[i]);
    if (solver_->hasFrame(ee_frame_name_)) frames.push_back(ee_frame_name_);
    return frames;
}

// With the kinematics just updated at q (control loop, data_mutex_ held): where the arm is against the zones,
// the wall torque that pushes back every segment past one (2000 N/m, as the box workspace's wall), and the
// arm's worst clearance for the Studio.
template <typename T>
void FicControlNode<T>::updateZoneWall(const Eigen::VectorXd& q) {
    fic::safety::ZoneSet zs;
    {
        std::lock_guard<std::mutex> lock(zones_mutex_);
        if (!zones_active_) {
            zone_tau_.resize(0);
            zone_worst_ = fic::safety::ZoneSet::BodyHit{};
            return;
        }
        zs = zones_;
    }
    const std::vector<std::string> frames = zoneBodyFrames(zs.body.from_joint);
    std::vector<Eigen::Vector3d> pts;
    for (const auto& f : frames) {
        Eigen::Isometry3d tf;
        if (solver_->cachedFramePose(f, tf)) pts.push_back(tf.translation());
    }
    if (pts.empty()) return;
    fic::safety::ZoneSet::BodyHit worst;
    const auto past = zs.check(pts, &worst);
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(num_joints_);
    if (!past.empty()) {
        std::vector<Eigen::MatrixXd> cache(pts.size());
        auto jac = [&](int i) -> const Eigen::MatrixXd& {
            if (cache[i].size() == 0) {
                fic::KinematicsSolver::Matrix6x J(6, num_joints_);
                J.setZero();
                solver_->frameJacobian(q, frames[i], fic::KinematicsSolver::JacobianFrame::LocalWorldAligned, J);
                cache[i] = J.topRows(3);
            }
            return cache[i];
        };
        tau = zs.wallTorque(past, jac, static_cast<int>(num_joints_), 2000.0);
    }
    zone_tau_ = tau;
    {
        std::lock_guard<std::mutex> lock(zones_mutex_);
        zone_worst_ = worst;
    }
    const int seg = std::max(worst.segment, 0);
    zone_worst_point_ = pts.size() == 1 ? pts[0]
        : (1.0 - worst.t) * pts[seg] + worst.t * pts[std::min<size_t>(seg + 1, pts.size() - 1)];
    const double now_s = this->now().seconds();
    if (zone_state_pub_ && now_s - zone_state_last_s_ >= 0.05) {
        zone_state_last_s_ = now_s;
        std_msgs::msg::Float64MultiArray m;
        // clearance (m, negative: past), zone, segment, segments past, the worst point x y z
        m.data = {worst.hit.clearance, static_cast<double>(worst.hit.zone), static_cast<double>(worst.segment),
                  static_cast<double>(past.size()), zone_worst_point_.x(), zone_worst_point_.y(), zone_worst_point_.z()};
        zone_state_pub_->publish(m);
    }
}

// A joint target is refused when it would put the arm further past a zone than it is now.
template <typename T>
bool FicControlNode<T>::bodyTargetClear(const Eigen::VectorXd& q) {
    fic::safety::ZoneSet zs;
    double now_clearance;
    {
        std::lock_guard<std::mutex> lock(zones_mutex_);
        if (!zones_active_) return true;
        zs = zones_;
        now_clearance = zone_worst_.hit.clearance;
    }
    if (!solver_ || !solver_->isLoaded()) return true;   // fail-open as the workspace guard (joint limits still hold)
    if (q.size() != solver_->nq()) return true;          // as the workspace guard
    std::vector<Eigen::Vector3d> pts;
    for (const auto& f : zoneBodyFrames(zs.body.from_joint)) {
        Eigen::Isometry3d tf;
        if (solver_->framePose(q, f, tf)) pts.push_back(tf.translation());
    }
    if (pts.empty()) return true;
    fic::safety::ZoneSet::BodyHit worst;
    zs.check(pts, &worst);
    if (worst.hit.clearance >= 0.0 || worst.hit.clearance >= now_clearance - 1e-4) return true;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Joint target refused: it would take the arm %.0f mm into safety zone %d", -1000.0 * worst.hit.clearance,
        worst.hit.zone + 1);
    return false;
}

template <typename T>
void FicControlNode<T>::jointSafetyCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 3) {
        std::lock_guard<std::mutex> lock(data_mutex_);   // the IK worker copies these under it
        size_t n_joints = (msg->data.size() - 1) / 2;
        joint_safety_lower_.resize(n_joints);
        joint_safety_upper_.resize(n_joints);

        for (size_t i = 0; i < n_joints; ++i) {
            joint_safety_lower_[i] = msg->data[i * 2];
            joint_safety_upper_[i] = msg->data[i * 2 + 1];
        }
        joint_safety_enabled_ = msg->data.back() > 0.5;

        RCLCPP_INFO(this->get_logger(), "Updated joint safety limits for %zu joints (enabled: %s)",
            n_joints, joint_safety_enabled_ ? "true" : "false");
    }
}

template <typename T>
void FicControlNode<T>::velocitySafetyCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 3) {
        max_joint_velocity_ = msg->data[0];
        max_cart_velocity_ = msg->data[1];
        velocity_limits_enabled_ = msg->data[2] > 0.5;
        // optional 4th value: the joint target speed of the joint modes (rad/s, 0: off)
        if (msg->data.size() >= 4 && std::isfinite(msg->data[3])) joint_slew_.speed = std::max(0.0, msg->data[3]);
        fic_ctrl_.setJointSpeedLimit(velocity_limits_enabled_ ? max_joint_velocity_ : 0.0);
        // The MCU takes it with its next CONFIG (a CONFIG resets the MCU, E-STOP latch included,
        // so a speed change does not send one).
        mcu_limits_.joint_speed_limit = velocity_limits_enabled_ ? max_joint_velocity_ : 0.0;

        RCLCPP_INFO(this->get_logger(), "Updated velocity limits: joint=%.2f rad/s, cart=%.2f m/s (enabled: %s), "
            "joint target speed %.2f rad/s%s",
            max_joint_velocity_, max_cart_velocity_, velocity_limits_enabled_ ? "true" : "false",
            joint_slew_.speed, joint_slew_.speed > 0.0 ? "" : " (off)");
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SAFETY CLAMPING
// ═══════════════════════════════════════════════════════════════════════════════

template <typename T>
double FicControlNode<T>::clamp(double value, double min_val, double max_val) {
    return std::max(min_val, std::min(max_val, value));
}

template <typename T>
void FicControlNode<T>::applyCartesianSafetyLimits(geometry_msgs::msg::Pose& target_pose) {
    if (cart_safety_limits_.x_enabled)
        target_pose.position.x = clamp(target_pose.position.x, cart_safety_limits_.x_min, cart_safety_limits_.x_max);
    if (cart_safety_limits_.y_enabled)
        target_pose.position.y = clamp(target_pose.position.y, cart_safety_limits_.y_min, cart_safety_limits_.y_max);
    if (cart_safety_limits_.z_enabled)
        target_pose.position.z = clamp(target_pose.position.z, cart_safety_limits_.z_min, cart_safety_limits_.z_max);

    // Whole-arm safety zones: the tool target kept clear of them; where no clear place is found nearby, the
    // last target that was (or, before any, the tool where it is).
    std::lock_guard<std::mutex> lock(zones_mutex_);
    if (!zones_active_) return;
    const Eigen::Vector3d p(target_pose.position.x, target_pose.position.y, target_pose.position.z);
    Eigen::Vector3d out;
    if (zones_.project(p, zones_.body.tool_radius, &out)) {
        zone_safe_target_ = out;
        zone_target_safe_ = true;
    } else {
        out = zone_target_safe_ ? zone_safe_target_
            : Eigen::Vector3d(ee_current.position.x, ee_current.position.y, ee_current.position.z);
    }
    if ((out - p).norm() > 1e-4)
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Tool target moved %.0f mm to keep it out of the safety zones", 1000.0 * (out - p).norm());
    target_pose.position.x = out.x();
    target_pose.position.y = out.y();
    target_pose.position.z = out.z();
}

template <typename T>
void FicControlNode<T>::applyJointSafetyLimits(Eigen::VectorXd& joint_target) {
    if (!joint_safety_enabled_ || joint_safety_lower_.empty()) return;
    for (size_t i = 0; i < std::min((size_t)joint_target.size(), joint_safety_lower_.size()); ++i) {
        joint_target[i] = clamp(joint_target[i], joint_safety_lower_[i], joint_safety_upper_[i]);
    }
}

template <typename T>
bool FicControlNode<T>::isPositionInSafetyZone(double x, double y, double z) {
    bool safe = true;
    if (cart_safety_limits_.x_enabled && (x < cart_safety_limits_.x_min || x > cart_safety_limits_.x_max)) safe = false;
    if (cart_safety_limits_.y_enabled && (y < cart_safety_limits_.y_min || y > cart_safety_limits_.y_max)) safe = false;
    if (cart_safety_limits_.z_enabled && (z < cart_safety_limits_.z_min || z > cart_safety_limits_.z_max)) safe = false;
    return safe;
}

template <typename T>
bool FicControlNode<T>::eeTargetInsideZone(const Eigen::VectorXd& q) {
    // Forward-kinematics check: would this candidate joint target put the EE
    // outside the enabled Cartesian safety zone? Used to preventively keep the
    // EE inside the workspace box while commanding in JOINT space.
    if (!(cart_safety_limits_.x_enabled || cart_safety_limits_.y_enabled
          || cart_safety_limits_.z_enabled))
        return true;                          // no Cartesian zone active
    if (!solver_ || !solver_->isLoaded()) {
        // FAIL-OPEN by design: joint-space control is the only thing degraded
        // mode supports, so blocking every joint target here would brick it.
        // Joint-space limits and torque hard-fault remain fully enforced.
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
            "Cartesian workspace guard inactive — no kinematics solver");
        return true;
    }
    if (q.size() != solver_->nq()) return true;   // size mismatch -> do not block
    Eigen::Isometry3d ee_iso;
    if (!solver_->framePose(q, ee_frame_name_, ee_iso)) return true;
    const auto& t = ee_iso.translation();
    return isPositionInSafetyZone(t[0], t[1], t[2]);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  MAIN CONTROL LOOP
// ═══════════════════════════════════════════════════════════════════════════════

template <typename T>
bool FicControlNode<T>::refreshTuning(const fic::TuningKey& want, const std::string& profile) {
    // A read has finished: install it if it is still for what the loop wants
    // (a mode or robot change while it ran makes it stale; it is dropped).
    if (tuning_load_.valid() &&
        tuning_load_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        try {
            fic::TuningSet loaded = tuning_load_.get();
            if (loaded.key == want) {
                tuning_ = std::move(loaded);
                have_tuning_ = true;
                joint_scalars_ = tuning_.joint_scalars;
                joint_lists_ = tuning_.joint_lists;
                joint_planners_ = tuning_.joint_planners;
                joint_admittance_ = tuning_.joint_admittance;
                cart_params_ = tuning_.cart;
                postural_scalars_ = tuning_.postural_scalars;
                postural_lists_ = tuning_.postural_lists;
                postural_planner_ = tuning_.postural_planner;
                postural_admittance_ = tuning_.postural_admittance;
                initControllerPLanner = false;   // switch the controllers to the new set
                RCLCPP_INFO(this->get_logger(), "Controllers Initialized (Mode %d, tuning read from config.xml)",
                            (int)current_control_mode_);
                if (tuning_.key.joint_mode == fic::kHpmpTuningMode) {
                    if (tuning_.hpmp_fallback)
                        RCLCPP_WARN(this->get_logger(), "H-PMP tuning: config.xml has no H-PMP block for %s, "
                                    "so H-PMP runs on mode 4's joints and mode 6's Cartesian block", want.robot.c_str());
                    else
                        RCLCPP_INFO(this->get_logger(), "H-PMP tuning: its own block (mode 100), config \"%s\"",
                                    tuning_.profile.c_str());
                }
                // The MCU runs its own copy of the joint tuning: send it the new set.
                if (use_mcu_ && mcu_.configured() &&
                    !mcu_.sendParams(joint_scalars_, joint_lists_, joint_planners_, joint_admittance_)) {
                    RCLCPP_ERROR(this->get_logger(),
                        "Joint params for mode %d do not fit the MCU (more than %d saturation "
                        "points, or missing admittance limits) — the MCU will hold position.",
                        (int)current_control_mode_, fic::core::kMaxSat);
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Init Failed: %s", e.what());
            tuning_failed_ = true;
            tuning_failed_key_ = want;
        }
    }

    // Start a read when the loop needs a set it does not have, or the file changed.
    if (!tuning_load_.valid()) {
        const bool current = have_tuning_ && tuning_.key == want;
        const bool failed = tuning_failed_ && tuning_failed_key_ == want;
        if (tuning_reload_ || (!current && !failed)) {
            tuning_reload_ = false;
            tuning_failed_ = false;
            const std::string xml_file = ament_index_cpp::get_package_share_directory("fic_control") + "/config/config.xml";
            tuning_load_ = std::async(std::launch::async, [xml_file, want, profile] {
                return fic::loadTuningSet(xml_file, want, profile);
            });
        }
    }
    return have_tuning_ && tuning_.key == want;
}

template <typename T>
void FicControlNode<T>::controlLoop() {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (!is_robot_supported_ || num_joints_ == 0) return;
    fic_ctrl_.setDesiredWrench(desired_wrench_);

    if (use_mcu_) {
        if (mcu_setup_done_.exchange(false)) {
            mcu_armed_ = mcu_.configure(current_robot_config_, mcu_limits_);
            mcu_stop_sent_ = false;
            // A reset MCU has no joint params: read the tuning again so installing
            // it sends PARAMS, even when the robot and mode are unchanged.
            tuning_reload_ = true;
            initControllerPLanner = false;
            if (mcu_armed_)
                RCLCPP_INFO(this->get_logger(), "Inner loop configured for %s — the joint layer runs on %s.",
                            selected_robot_.c_str(), innerLoopWhere());
            else
                RCLCPP_ERROR(this->get_logger(), "Could not send CONFIG to the inner loop (%s).", innerLoopWhere());
        }
        mcu_.service();
        if (sim_ && topics_plant_) topicPlantTick();
    }

    readSharedFeedback();

    // No torque before the robot's real state is known: q_current_ is still the
    // zero vector that initSharedMemory() left in shared memory, so the
    // impedance law would command a step away from a fictitious position, the
    // planners would seed themselves at 0 rad, and the joint-limit hard fault
    // would latch against a position the robot was never at. Mirrors
    // fic_simulation's `if (!joint_state_received_) return;` control-loop gate.
    // Withholding torque is strictly more conservative than the previous
    // behaviour: commanded_torque keeps the NaN "nothing commanded yet"
    // sentinel that shared_params.h installs for exactly this case.
    if (!feedback_received_) return;

    // A target stream that stopped. Hold where the arm IS rather than go on
    // pulling toward a setpoint nobody is updating. See the watchdog note in
    // the header for why a one-shot target never triggers this.
    if (target_stream_active_.load(std::memory_order_acquire)) {
        const double age = this->now().seconds() - last_target_s_.load(std::memory_order_acquire);
        if (age > kTargetStaleS) {
            target_stream_active_.store(false, std::memory_order_release);
            q_target_ = q_current_;
            RCLCPP_WARN(this->get_logger(),
                "No streamed target for %.1f s — holding the current pose instead of the "
                "last commanded one. If a teleoperation session was running, it is gone.",
                age);
        }
    }

    // Before anything below can return early (e.g. a missing EE frame): the
    // gripper state does not depend on the kinematics.
    publishGripperState();

    const bool solver_ok = solver_ && solver_->isLoaded();

    Eigen::MatrixXd J_elbow_full = Eigen::MatrixXd::Zero(6, num_joints_);
    std::vector<double> elbow_curr_vec(7, 0.0);

    if (solver_ok) {
        solver_->updateKinematics(q_current_);

        Eigen::Isometry3d ee_transform;
        if (!solver_->cachedFramePose(ee_frame_name_, ee_transform)) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "CRITICAL: EE Frame '%s' not found!", ee_frame_name_.c_str());
            return;
        }

        ee_current = isoToPoseMsg(ee_transform);
        if (ee_pose_publisher_) ee_pose_publisher_->publish(ee_current);

        fic::KinematicsSolver::Matrix6x J_ee(6, num_joints_);
        J_ee.setZero();
        solver_->frameJacobian(q_current_, ee_frame_name_,
                               fic::KinematicsSolver::JacobianFrame::LocalWorldAligned, J_ee);
        jacobian = J_ee;

        Eigen::Isometry3d elb_tf;
        if (solver_->cachedFramePose(elbow_frame_name_, elb_tf)) {
            elbow_curr_vec[0] = elb_tf.translation().x();
            elbow_curr_vec[1] = elb_tf.translation().y();
            elbow_curr_vec[2] = elb_tf.translation().z();
            fic::KinematicsSolver::Matrix6x J_elbow(6, num_joints_);
            J_elbow.setZero();
            solver_->frameJacobian(q_current_, elbow_frame_name_,
                                   fic::KinematicsSolver::JacobianFrame::LocalWorldAligned, J_elbow);
            J_elbow_full = J_elbow;

            // Keep elbow_current_pose_ fresh so the elbow-enable callback can
            // snapshot the current elbow position as the hold target.
            elbow_current_pose_.position.x = elbow_curr_vec[0];
            elbow_current_pose_.position.y = elbow_curr_vec[1];
            elbow_current_pose_.position.z = elbow_curr_vec[2];
        }

        gravity_torques_ = solver_->gravity(q_current_);
        // Simulation: MuJoCo's own gravity (exact for its model), as the old fic_simulation used; the
        // solver's until the simulator's first message.
        if (sim_ && sim_gravity_.size() >= num_joints_)
            for (size_t i = 0; i < num_joints_; ++i) gravity_torques_[i] = sim_gravity_[i];
        updateZoneWall(q_current_);
    } else {
        // Degraded mode: joint-space only. gravity_torques_ stays zero (no
        // feedforward), EE pose / Jacobians are never updated or published.
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
            "Running WITHOUT kinematics solver: joint-space only (no EE pose, "
            "Jacobian, gravity feedforward, or Cartesian features)");
    }

    std::vector<double> traj_data(num_joints_ * 5, 0.0);
    std::vector<double> torque_data(num_joints_, 0.0);

    std::vector<double> wrench_data(6, 0.0);
    if (current_robot_config_.wrench_from_api && sim_) {
        // Simulation: the wrist F/T sensor, rotated from the tool frame to the world (as the old
        // fic_simulation did).
        if (sim_ft_ok_) {
            const Eigen::Matrix3d R = Eigen::Quaterniond(ee_current.orientation.w, ee_current.orientation.x,
                ee_current.orientation.y, ee_current.orientation.z).normalized().toRotationMatrix();
            const Eigen::Vector3d f = R * Eigen::Vector3d(sim_ft_.force.x, sim_ft_.force.y, sim_ft_.force.z);
            const Eigen::Vector3d t = R * Eigen::Vector3d(sim_ft_.torque.x, sim_ft_.torque.y, sim_ft_.torque.z);
            wrench_data = {f.x(), f.y(), f.z(), t.x(), t.y(), t.z()};
        }
    } else if (current_robot_config_.wrench_from_api && shared_feedback) {
        // Read wrench directly from the driver/API via shared memory —
        // no solver needed, stays functional in degraded mode.
        for (int i = 0; i < 6; ++i)
            wrench_data[i] = shared_feedback->tool_wrench[i].load(std::memory_order_acquire);
    } else if (solver_ok) {
        // Compute from Jacobian and measured joint torques: F_ext = pinv(J^T) * tau_ext
        Eigen::VectorXd tau_ext(num_joints_);
        for (size_t i = 0; i < num_joints_; ++i)
            tau_ext[i] = -(joint_torques_[i] + gravity_torques_[i]);
        Eigen::MatrixXd JJt = jacobian * jacobian.transpose();
        Eigen::VectorXd F_ext = JJt.ldlt().solve(jacobian * tau_ext);
        for (int i = 0; i < 6 && i < F_ext.size(); ++i)
            wrench_data[i] = F_ext[i];
    }
    // (no solver and no API wrench → wrench stays zero)

    // H-PMP's task-space admittance on the ideal simulator: the wrench the environment applies,
    // pinv(J^T)(-tau_contact) from MuJoCo's contact torques, as the old fic_simulation did — the simulated
    // measured torque is the node's own command, whose estimate would make the admittance yield to the
    // arm's own tracking effort. With the hardware model (and on the arm): the estimate above.
    std::vector<double> contact_wrench;           // set only on the ideal simulator; else H-PMP uses wrench_data
    if (sim_ && !current_robot_config_.hardware_model.enabled && solver_ok && jacobian.rows() == 6 &&
        (size_t)jacobian.cols() == num_joints_ && sim_contact_.size() >= num_joints_) {
        Eigen::VectorXd tau_c(num_joints_);
        for (size_t i = 0; i < num_joints_; ++i) tau_c[i] = -sim_contact_[i];
        const Eigen::MatrixXd JJt = jacobian * jacobian.transpose();
        const Eigen::VectorXd F_c = JJt.ldlt().solve(jacobian * tau_c);
        contact_wrench.assign(F_c.data(), F_c.data() + 6);
    }

    {
        geometry_msgs::msg::Wrench wmsg;
        wmsg.force.x  = wrench_data[0]; wmsg.force.y  = wrench_data[1]; wmsg.force.z  = wrench_data[2];
        wmsg.torque.x = wrench_data[3]; wmsg.torque.y = wrench_data[4]; wmsg.torque.z = wrench_data[5];
        wrench_publisher_->publish(wmsg);
    }

    // ── Reflected wrench from a teleoperation partner ─────────────────────
    // Added AFTER the publish above, so {rid}_force_torque keeps reporting
    // what this arm is actually touching. Two arms reflecting to each other
    // through that topic would be a positive feedback loop over the network.
    //
    // Only the Cartesian admittance family consumes a wrench as something to
    // move for: mode 6 Hpmp, 7 and 8 admittance. Mode 5 is pure impedance
    // and is passed the wrench only for bookkeeping, so injecting there would
    // change nothing while looking like it should.
    if (external_wrench_active_ &&
        (current_control_mode_ == 6 || current_control_mode_ == 7 || current_control_mode_ == 8)) {
        const double age = (this->now() - external_wrench_stamp_).seconds();
        double fade = 1.0;
        if (age > kExtWrenchStaleS)
            fade = std::max(0.0, 1.0 - (age - kExtWrenchStaleS) / kExtWrenchFadeS);

        if (fade <= 0.0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Reflected wrench is stale (%.0f ms), holding it at zero.", age * 1000.0);
        }
        for (int i = 0; i < 6; ++i) {
            const bool is_force = (i < 3);
            const double dead = is_force ? current_robot_config_.teleop_wrench_deadband_f
                                         : current_robot_config_.teleop_wrench_deadband_t;
            const double lim  = is_force ? current_robot_config_.teleop_wrench_max_force
                                         : current_robot_config_.teleop_wrench_max_torque;
            double w = external_wrench_[i];
            w = (std::abs(w) <= dead) ? 0.0 : (w - std::copysign(dead, w));
            w = std::clamp(w, -lim, lim);
            wrench_data[i] += fade * w;
        }
    }

    std::vector<double> ee_curr_vec = {
        ee_current.position.x, ee_current.position.y, ee_current.position.z,
        ee_current.orientation.x, ee_current.orientation.y, ee_current.orientation.z, ee_current.orientation.w};
    std::vector<double> ee_targ_vec = {
        ee_target.position.x, ee_target.position.y, ee_target.position.z,
        ee_target.orientation.x, ee_target.orientation.y, ee_target.orientation.z, ee_target.orientation.w};

    Eigen::Map<Eigen::VectorXd> q_dot(currentVelocities_.data(), num_joints_);
    Eigen::VectorXd ee_velocity = jacobian * q_dot;

    int param_mode = current_control_mode_;
    const bool hpmp_active = fic::hpmpActive(param_mode, hpmp_enabled_);
    // No restart on a stall in the modes where an admittance moves the goal (fic_control.h).
    fic_ctrl_.setRestartAllowed(fic::ficControl<T>::restartAllowedInMode(param_mode, hpmp_active));
    // The blocks fic_simulation reads for the same mode (fic::tuningKeyFor): H-PMP its own,
    // and on the MCU path every Cartesian mode mode 4's joints.
    const fic::TuningKey tuning_want = fic::tuningKeyFor(selected_robot_, num_joints_, param_mode,
                                                         hpmp_enabled_, use_mcu_);
    // Nothing to run until the tuning for this robot and mode is in memory.
    if (!refreshTuning(tuning_want, active_profile_)) return;

    if (!initControllerPLanner) {
        try {
            if (hpmp_active) {
                initJointControllers(num_joints_, q_current_, joint_scalars_, joint_lists_, joint_planners_, joint_admittance_);
                fic_ctrl_.initHpmpCartesian(cart_params_, ee_curr_vec);
                // Postural channels 9..9+n are free under H-PMP too (joints 0..n-1,
                // Cartesian 16..19, elbow 20..22): initialise them as in the Cartesian modes
                // (their tuning is the robot-wide postural block, read with the rest of the set).
                fic_ctrl_.postural.init(num_joints_, postural_scalars_, postural_lists_);
            } else if (param_mode <= 4) {
                initJointControllers(num_joints_, q_current_, joint_scalars_, joint_lists_, joint_planners_, joint_admittance_);
            } else {
                initCartesianControllers(ee_curr_vec, cart_params_);

                // Null-space postural task lives past the Cartesian (0..5) and
                // elbow (6..8) channels, with its own per-joint FIC controllers
                // (read from its XML block with the rest of the tuning) so
                // postural.apply() does not collide with the Cartesian / elbow tasks.
                fic_ctrl_.postural.init(num_joints_, postural_scalars_, postural_lists_);
            }
            initControllerPLanner = true;
            RCLCPP_DEBUG(this->get_logger(), "Controllers Initialized (Mode %d)", (int)current_control_mode_);
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Init Failed: %s", e.what());
            return;
        }
    }

    int CartAdmMode = 0;
    std::fill(wall_torque_.begin(), wall_torque_.end(), 0.0);
    fic_ctrl_.hpmp_export_only_ = true;   // H-PMP's joint layer runs in the inner loop
    if (current_control_mode_ <= 4 && current_control_mode_ > 0) {
        // The joint law (and the joint-target speed limit of modes 1/2) runs in the inner loop: this node sends
        // it the target, the wall and zone torque and gravity (sendMcuCommand).

        // Cartesian virtual wall for joint modes: when the EE is past an
        // enabled Cartesian safety boundary, add a restoring force via J^T
        // so the robot is bounded at the limit instead of faulting.
        // Requires a kinematics solver — without one ee_current/jacobian are
        // never updated, so wall forces would be computed from stale state.
        if (solver_ok &&
            (cart_safety_limits_.x_enabled || cart_safety_limits_.y_enabled
             || cart_safety_limits_.z_enabled)) {
            Eigen::Vector3d f_wall = Eigen::Vector3d::Zero();
            constexpr double kWall = 2000.0;  // N/m
            const double ex = ee_current.position.x;
            const double ey = ee_current.position.y;
            const double ez = ee_current.position.z;
            if (cart_safety_limits_.x_enabled) {
                if (ex < cart_safety_limits_.x_min)
                    f_wall[0] = kWall * (cart_safety_limits_.x_min - ex);
                else if (ex > cart_safety_limits_.x_max)
                    f_wall[0] = kWall * (cart_safety_limits_.x_max - ex);
            }
            if (cart_safety_limits_.y_enabled) {
                if (ey < cart_safety_limits_.y_min)
                    f_wall[1] = kWall * (cart_safety_limits_.y_min - ey);
                else if (ey > cart_safety_limits_.y_max)
                    f_wall[1] = kWall * (cart_safety_limits_.y_max - ey);
            }
            if (cart_safety_limits_.z_enabled) {
                if (ez < cart_safety_limits_.z_min)
                    f_wall[2] = kWall * (cart_safety_limits_.z_min - ez);
                else if (ez > cart_safety_limits_.z_max)
                    f_wall[2] = kWall * (cart_safety_limits_.z_max - ez);
            }
            if (f_wall.squaredNorm() > 0) {
                Eigen::VectorXd tau_wall = jacobian.topRows(3).transpose() * f_wall;
                for (size_t i = 0; i < num_joints_; ++i) {
                    torque_data[i] += tau_wall[i];
                    if (i < wall_torque_.size()) wall_torque_[i] = tau_wall[i];
                }
            }
        }
    } else if (current_control_mode_ > 4) {
        switch (current_control_mode_) {
            case 5:
                if (hpmp_enabled_) {
                    fic_ctrl_.RC.hpmp(traj_data, torque_data, q_current_, gravity_torques_,
                        ee_curr_vec, ee_targ_vec, jacobian, fic::ficControl<double>::kHpmpHostImpedance,
                        wrench_data, joint_torques_, q_target_ik_, num_joints_);
                } else {
                    fic_ctrl_.RC.impedance(traj_data, torque_data, q_current_, gravity_torques_,
                        ee_curr_vec, ee_targ_vec, jacobian, 0, wrench_data, joint_torques_, q_target_ik_, num_joints_);
                }
                break;
            case 6:
                if (hpmp_enabled_) {
                    fic_ctrl_.RC.hpmp(traj_data, torque_data, q_current_, gravity_torques_,
                        ee_curr_vec, ee_targ_vec, jacobian, 0, wrench_data, joint_torques_, q_target_ik_, num_joints_);
                } else {
                    fic_ctrl_.RC.planner(traj_data, torque_data, q_current_, gravity_torques_,
                        ee_curr_vec, ee_targ_vec, jacobian, 0, wrench_data, joint_torques_, q_target_ik_, num_joints_);
                }
                break;
            case 7:
                CartAdmMode = 1;
                if (hpmp_enabled_) {
                    fic_ctrl_.RC.hpmp(traj_data, torque_data, q_current_, gravity_torques_,
                        ee_curr_vec, ee_targ_vec, jacobian, CartAdmMode, contact_wrench.empty() ? wrench_data : contact_wrench, joint_torques_, q_target_ik_, num_joints_);
                } else {
                    fic_ctrl_.RC.admittance(traj_data, torque_data, q_current_, gravity_torques_,
                        ee_curr_vec, ee_targ_vec, jacobian, CartAdmMode, wrench_data, joint_torques_, q_target_ik_, ee_velocity, num_joints_,
                        current_robot_config_.wrench_gravity_compensated);
                }
                break;
            case 8:
                CartAdmMode = 2;
                if (hpmp_enabled_) {
                    fic_ctrl_.RC.hpmp(traj_data, torque_data, q_current_, gravity_torques_,
                        ee_curr_vec, ee_targ_vec, jacobian, CartAdmMode, contact_wrench.empty() ? wrench_data : contact_wrench, joint_torques_, q_target_ik_, num_joints_);
                } else {
                    fic_ctrl_.RC.admittance(traj_data, torque_data, q_current_, gravity_torques_,
                        ee_curr_vec, ee_targ_vec, jacobian, CartAdmMode, wrench_data, joint_torques_, q_target_ik_, ee_velocity, num_joints_,
                        current_robot_config_.wrench_gravity_compensated);
                }
                break;
        }

        // H-PMP's joint layer runs in the inner loop: this node only exports tau_des (hpmp_export_only_), and
        // the inner loop adds gravity.

        // Null-space helpers — optional additions on top of the Cartesian torques.
        // controlLoop already holds data_mutex_ (acquired at the top of the
        // function), so we read the enable flags and elbow target directly —
        // re-locking a non-recursive std::mutex on the same thread deadlocks.
        if (postural_enabled_ && q_target_ik_.size() == q_current_.size()) {
            stepPosturalTarget();
            fic_ctrl_.postural.apply(q_current_, q_postural_, CartAdmMode, num_joints_, torque_data);
        }
        if (elbow_enabled_ && J_elbow_full.cols() == (int)num_joints_) {
            std::vector<double> elbow_targ_vec = {
                elbow_target_pose_.position.x, elbow_target_pose_.position.y, elbow_target_pose_.position.z,
                elbow_target_pose_.orientation.x, elbow_target_pose_.orientation.y, elbow_target_pose_.orientation.z, elbow_target_pose_.orientation.w};
            fic_ctrl_.applyElbowControl(torque_data, q_current_, jacobian, J_elbow_full,
                                        elbow_curr_vec, elbow_targ_vec, num_joints_,
                                        fic_ctrl_.elbowOffset());
        }
    } else if (current_control_mode_ == 0) {
        // Idle torque baseline: gravity compensation for robots whose driver
        // does NOT compensate gravity internally; zero for those that do.
        std::vector<double> idle_torques(num_joints_, 0.0);
        if (!current_robot_config_.gravity_compensated) {
            for (size_t i = 0; i < num_joints_ && i < (size_t)gravity_torques_.size(); ++i)
                idle_torques[i] = gravity_torques_[i];
        }

        if (is_ramping_down_ && ramp_down_torques_.size() == num_joints_) {
            constexpr double kDecay = 0.995;  // ~200 ms time constant at 1 kHz
            bool converged = true;
            for (size_t i = 0; i < num_joints_; ++i) {
                double diff = ramp_down_torques_[i] - idle_torques[i];
                diff *= kDecay;
                if (std::abs(diff) > 0.05) converged = false;
                else diff = 0.0;
                torque_data[i] = idle_torques[i] + diff;
                ramp_down_torques_[i] = torque_data[i];
            }
            if (converged) {
                is_ramping_down_ = false;
                ramp_down_torques_.clear();
                if (shared_feedback)
                    shared_feedback->stop_torque_mode.store(1, std::memory_order_release);
                RCLCPP_INFO(this->get_logger(), "Torque ramp-down complete — signalling driver to exit torque mode");
            }
        } else {
            torque_data = idle_torques;
        }
    }

    // Hard-fault check: if a joint exceeds the robot config's physical limits,
    // latch a fault and output only gravity comp until a new robot is loaded.
    // Gated on feedback_received_ so it is only ever evaluated against real
    // driver feedback — the zero-initialised q_current_ sits outside the
    // configured range of any robot whose limits exclude 0 deg (Franka_FR3's
    // joint 3 is [-176, -4] deg, so the very first control tick used to latch a
    // permanent fault before the driver had written anything).
    if (!hard_fault_ && feedback_received_ &&
        !current_robot_config_.joint_limits_lower_deg.empty() &&
        !current_robot_config_.joint_limits_upper_deg.empty()) {
        for (size_t i = 0; i < num_joints_; ++i) {
            if (i >= current_robot_config_.joint_limits_lower_deg.size()) break;
            double pos_deg = q_current_[i] * 180.0 / M_PI;
            if (pos_deg < current_robot_config_.joint_limits_lower_deg[i] ||
                pos_deg > current_robot_config_.joint_limits_upper_deg[i]) {
                hard_fault_ = true;
                RCLCPP_ERROR(this->get_logger(),
                    "HARD FAULT: Joint %zu at %.1f° exceeds config limit [%.1f, %.1f]°",
                    i, pos_deg,
                    current_robot_config_.joint_limits_lower_deg[i],
                    current_robot_config_.joint_limits_upper_deg[i]);
                break;
            }
        }
    }
    // Whole-arm safety zones: their wall torque (updateZoneWall), in every mode.
    if (solver_ok && zone_tau_.size() == static_cast<Eigen::Index>(num_joints_))
        for (size_t i = 0; i < num_joints_; ++i) torque_data[i] += zone_tau_[i];

    if (hard_fault_) {
        for (size_t i = 0; i < num_joints_; ++i)
            torque_data[i] = current_robot_config_.gravity_compensated ? 0.0 : gravity_torques_[i];
    }

    // Capture active torques so ramp-down has a starting point
    if (current_control_mode_ > 0) {
        ramp_down_torques_ = torque_data;
        // On the MCU path the node's torque holds no joint law (modes 1-4) and, under H-PMP, no task torque
        // or gravity: start the ramp from what the arm last got (STATE tau_cmd), or the arm drops at Stop.
        if (use_mcu_ && mcu_.configured()) {
            std::vector<double> got(num_joints_, 0.0);
            if (fic::link::rampStartFromState(mcu_.lastState(), mcu_.lastStateAge(), static_cast<int>(num_joints_),
                                              got.data()))
                ramp_down_torques_ = got;
        }
    }

    // Output: to the inner loop (the MCU, or fic_inner_loop on this PC), which guards it (damping, NaN,
    // clamp, rate limit) and applies it. Without an inner loop nothing is sent: the arm is not driven.
    if (use_mcu_) {
        sendMcuCommand(torque_data, solver_ok);
        // The joint law runs in the inner loop: its diagnostics come from STATE (the dashboard's commanded
        // and measured joints).
        if (current_control_mode_ >= 1 && current_control_mode_ <= 4 && (size_t)q_target_.size() >= num_joints_)
            fic::fillJointDiag(mcu_.lastState(), q_target_.data(), static_cast<int>(num_joints_), traj_data);
        if (sim_) publishSimOutputs();
    }

    // Human push offset: published ONLY while joint admittance is active
    // (modes 3/4). Never publish zeros in any other mode — a consumer must be
    // able to tell "no admittance running" from "nobody is pushing" (a real,
    // meaningful zero offset).
    if (current_control_mode_ == 3 || current_control_mode_ == 4) {
        std_msgs::msg::Float64MultiArray offset_msg;
        offset_msg.data.resize(num_joints_);
        const fic::link::StateMsg& st = mcu_.lastState();
        for (size_t i = 0; i < num_joints_; ++i) {
            // The inner loop's own offset (STATE): the joint admittance runs there.
            offset_msg.data[i] = (use_mcu_ && i < st.n) ? st.adm_offset[i] : 0.0;
        }
        admittance_offset_publisher_->publish(offset_msg);
    }

    std_msgs::msg::Float64MultiArray traj_msg;
    traj_msg.data = traj_data;
    trajectory_publisher_->publish(traj_msg);
}

// Simulation: what fic_simulation published, from the inner loop's STATE (once per new physics step).
template <typename T>
void FicControlNode<T>::publishSimOutputs() {
    if (!sim_torque_cmd_pub_ || !mcu_.configured()) return;
    const fic::link::StateMsg& st = mcu_.lastState();
    const size_t n = std::min<size_t>(num_joints_, st.n);
    if (n == 0 || st.tick == last_published_tick_) return;
    last_published_tick_ = st.tick;
    std_msgs::msg::Float64MultiArray law, applied;
    communication::msg::ActuatorCmds cmds;
    law.data.resize(n);
    applied.data.resize(n);
    for (size_t i = 0; i < n; ++i) {
        law.data[i] = st.tau_law[i];
        applied.data[i] = st.tau_cmd[i];
        cmds.actuators_name.push_back(joint_name_prefix_ + "torq_j" + std::to_string(i + 1));
        cmds.torque.push_back(st.tau_cmd[i]);
    }
    sim_torque_cmd_pub_->publish(law);
    sim_actuator_torque_pub_->publish(applied);
    if (!topics_plant_) sim_actuators_pub_->publish(cmds);   // the topic plant publishes the applied torques
    if (jacobian.rows() == 6 && (size_t)jacobian.cols() == num_joints_) {
        communication::msg::Jacobian jm;
        jm.header.stamp = this->get_clock()->now();
        jm.rows = 6;
        jm.cols = num_joints_;
        jm.jacobian.resize(6 * num_joints_);
        Eigen::Map<Eigen::MatrixXd>(jm.jacobian.data(), 6, num_joints_) = jacobian;
        sim_jacobian_pub_->publish(jm);
    }
}

// The topic plant, every control tick (data_mutex_ held): the latest joint_states as a SimStep (on this PC's
// clock: no lockstep), and the inner loop's torque for it as actuators_cmds.
template <typename T>
void FicControlNode<T>::topicPlantTick() {
    if (!mcu_.configured() || plant_q_.empty()) return;
    const auto now = std::chrono::steady_clock::now();
    if (now - plant_stamp_ > std::chrono::milliseconds(200)) return;          // no fresh sample
    fic::link::SimStepMsg s;
    s.t_us = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
        now.time_since_epoch()).count());
    s.n = static_cast<uint16_t>(std::min<size_t>(plant_q_.size(), fic::link::kLinkMaxJoints));
    for (size_t i = 0; i < s.n; ++i) { s.q[i] = static_cast<float>(plant_q_[i]); s.tau[i] = static_cast<float>(plant_tau_[i]); }
    mcu_.sendSimStep(s);
    fic::link::SimTorqueMsg t;
    double age = 0;
    if (!mcu_.latestSimTorque(t, age) || age > 0.05 || !t.torque_mode || !sim_actuators_pub_) return;
    communication::msg::ActuatorCmds cmds;
    for (size_t i = 0; i < t.n; ++i) {
        cmds.actuators_name.push_back(joint_name_prefix_ + "torq_j" + std::to_string(i + 1));
        cmds.torque.push_back(t.tau[i]);
    }
    sim_actuators_pub_->publish(cmds);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  INNER-LOOP MCU
// ═══════════════════════════════════════════════════════════════════════════════

template <typename T>
std::string FicControlNode<T>::robotLibraryCommand(std::string cmd) const {
    const std::string base = std::string(PACKAGE_SOURCE_DIR) + "/../../../robot_library";
    for (size_t pos = cmd.find("{robot_lib}"); pos != std::string::npos; pos = cmd.find("{robot_lib}"))
        cmd.replace(pos, 11, base);
    return cmd;
}

// Decide which inner loop drives the current robot: an MCU that can (robots with an `mcu:` adapter), else
// the PC inner loop (fic_inner_loop). One or two HELLO round trips. Nothing else drives the arm: without an
// inner loop this node sends no torque at all.
template <typename T>
void FicControlNode<T>::selectInnerLoop(const char* when) {
    const auto d = mcu_.detect(current_robot_config_);
    mcu_detail_ = d.detail;
    const bool use = d.usable && is_robot_supported_;
    const bool retry = std::string(when) == "retry";
    if (use != use_mcu_ || (!retry && std::string(when) != "Start")) {
        RCLCPP_INFO(this->get_logger(), "[%s] %s: inner loop %s — %s", when, selected_robot_.c_str(),
                    !use ? "NONE (the arm is not driven)" : (mcu_.local() ? "fic_inner_loop on this PC" : "on the MCU"),
                    d.detail.c_str());
    }
    if (!use) {
        if (use_mcu_ && mcu_.configured()) {
            RCLCPP_ERROR(this->get_logger(),
                "The inner loop no longer answers; it holds, then ramps the arm out of torque mode on its own.");
        }
        mcu_.reset();
        mcu_armed_ = false;
    } else if (mcu_.local() && (!use_mcu_ || !mcu_.configured())) {
        // The PC inner loop needs no arm preparation: configure it now (the control loop sends CONFIG).
        mcu_setup_done_ = true;
        pending_target_snap_ = true;
    }
    use_mcu_ = use;
}

// "Start" on the MCU path: optional Linux-side arm preparation (homing etc.),
// then CONFIG from the control loop. Every Start re-configures from scratch.
template <typename T>
void FicControlNode<T>::startMcu() {
    if (mcu_setup_running_) {
        RCLCPP_WARN(this->get_logger(), "MCU setup_command still running — Start ignored.");
        return;
    }
    if (mcu_setup_thread_.joinable()) mcu_setup_thread_.join();
    mcu_.reset();
    mcu_armed_ = false;
    mcu_stop_sent_ = false;
    mcu_setup_failed_ = false;
    pending_target_snap_ = true;                      // hold the current pose at (re)Start
    if (shared_feedback) shared_feedback->stop_torque_mode.store(0, std::memory_order_release);

    const std::string& setup = current_robot_config_.mcu.setup_command;
    if (setup.empty()) {
        mcu_setup_done_ = true;
        return;
    }
    const std::string cmd = robotLibraryCommand(setup);
    RCLCPP_INFO(this->get_logger(), "MCU path: running setup_command: %s", cmd.c_str());
    mcu_setup_done_ = false;
    mcu_setup_running_ = true;
    mcu_setup_thread_ = std::thread([this, cmd]() {
        const int rc = std::system(cmd.c_str());
        if (rc == 0) mcu_setup_done_ = true;
        else mcu_setup_failed_ = true;
        mcu_setup_running_ = false;
    });
}

// One CMD per control tick. The MCU runs the joint layer; this sends it the
// target, the outer-loop torque and gravity with its local slope, so the MCU
// can keep g(q) right between (and without) CMDs.
template <typename T>
void FicControlNode<T>::sendMcuCommand(const std::vector<double>& torque_data, bool solver_ok) {
    using namespace fic::link;
    if (!mcu_.configured()) return;
    const size_t n = std::min(num_joints_, static_cast<size_t>(kLinkMaxJoints));
    if ((size_t)q_current_.size() < n || (size_t)gravity_torques_.size() < n) return;

    CmdMsg m;
    const bool gc = current_robot_config_.gravity_compensated;
    std::vector<double> g(n, 0.0);
    for (size_t i = 0; i < n; ++i) {
        g[i] = gc ? 0.0 : gravity_torques_[i];
        m.g0[i] = static_cast<float>(g[i]);
        m.q0[i] = static_cast<float>(q_current_[i]);
    }
    if (!gc && solver_ok) {
        Eigen::VectorXd qh = q_current_;
        for (size_t j = 0; j < n; ++j) {
            qh[j] += kGravJacStep;
            const Eigen::VectorXd gp = solver_->gravity(qh);
            qh[j] = q_current_[j];
            if ((size_t)gp.size() < n) break;
            for (size_t i = 0; i < n; ++i)
                m.dgdq[i * n + j] = static_cast<float>((gp[i] - gravity_torques_[i]) / kGravJacStep);
        }
        m.flags |= kCmdHasGravJac;
    }

    auto fill = [n](float* dst, auto&& f) { for (size_t i = 0; i < n; ++i) dst[i] = static_cast<float>(f(i)); };
    const Eigen::VectorXd& q_ik = ((size_t)q_target_ik_.size() >= n) ? q_target_ik_ : q_current_;
    const Eigen::VectorXd& q_jt = ((size_t)q_target_.size() >= n) ? q_target_ : q_current_;

    // The routing table (fic/inner_loop_routing.h): modes 1-4 to the joint layer with wall + zone torque,
    // Cartesian modes passed through, H-PMP to Hapfic with its Jacobian (the inner loop runs its split drive).
    const int mode = current_control_mode_;
    const bool hpmp = hpmp_enabled_ && mode >= 5 && mode <= 8 &&
                      (size_t)fic_ctrl_.hpmp_tau_des_.size() >= n;
    std::vector<double> torque(n, 0.0), q_now(n), q_joint(n), q_task(n), td(n, 0.0), wall(n, 0.0), zone(n, 0.0),
                        jac_rm;
    for (size_t i = 0; i < n; ++i) {
        torque[i] = i < torque_data.size() ? torque_data[i] : 0.0;
        q_now[i] = q_current_[i];
        q_joint[i] = q_jt[i];
        q_task[i] = q_ik[i];
        wall[i] = i < wall_torque_.size() ? wall_torque_[i] : 0.0;
        if ((size_t)zone_tau_.size() == num_joints_) zone[i] = zone_tau_[i];
        if (hpmp) {
            // Each joint's set-point within its admittance bound (the joint law's outer breakpoint, at least
            // 0.1 rad) of where the joint is, after the task-space admittance's shift (hpmp_q_shift_), as
            // cartesianHpmp does on the host path.
            const Eigen::VectorXd xb = fic_ctrl_.controller(i).get_x_b();
            const double shift = (Eigen::Index)i < fic_ctrl_.hpmp_q_shift_.size() ? fic_ctrl_.hpmp_q_shift_[i] : 0.0;
            q_task[i] = fic::ficControl<T>::limitSetpoint(q_ik[i] + shift, q_current_[i],
                fic::ficControl<T>::setpointWindow(xb(xb.size() - 1)), isContinuousJoint(i));
            td[i] = fic_ctrl_.hpmp_tau_des_[i];
        }
    }
    if (hpmp && solver_ok && jacobian.rows() == 6 && (size_t)jacobian.cols() == n) {
        jac_rm.resize(6 * n);
        for (size_t r = 0; r < 6; ++r)
            for (size_t c = 0; c < n; ++c) jac_rm[r * n + c] = jacobian(r, c);
    }
    fic::link::RouteInput in;
    in.mode = mode;
    in.hard_fault = hard_fault_;
    in.hpmp = hpmp;
    in.n = static_cast<int>(n);
    in.torque = torque.data();
    in.gravity = g.data();
    in.q_now = q_now.data();
    in.q_joint = q_joint.data();
    in.q_task = q_task.data();
    in.hpmp_tau_des = td.data();
    in.wall = wall.data();
    in.zone = zone.data();
    in.jac = jac_rm.empty() ? nullptr : jac_rm.data();
    in.use_planner = admittance_use_planner_;
    in.joint_target_speed = joint_slew_.speed;
    fic::link::route(in, m);
    m.joint_speed_limit = static_cast<float>(mcu_limits_.joint_speed_limit);   // the operator's cap, live

    if (velocity_limits_enabled_ && max_joint_velocity_ > 0) {
        fill(m.kd, [&](size_t i) {
            const double lim = i < per_joint_torque_limit_.size() ? per_joint_torque_limit_[i]
                                                                  : output_torque_limit_;
            return lim / max_joint_velocity_;
        });
    }
    if (current_robot_config_.has_gripper && shared_feedback)
        m.gripper_width = shared_feedback->commanded_gripper_width.load(std::memory_order_acquire);

    if (mcu_armed_ && feedback_received_) m.flags |= kCmdEnable;
    if (shared_feedback && shared_feedback->stop_torque_mode.load(std::memory_order_acquire) == 1) {
        m.flags |= kCmdStopTorque;
        mcu_stop_sent_ = true;
    }
    if (mcu_stop_sent_ && mcu_armed_ && mcu_.lastState().state == McuState::Ready) {
        // Out of torque mode, like the vendor driver exiting (logged once).
        mcu_armed_ = false;
        RCLCPP_INFO(this->get_logger(), mcu_.local()
            ? "Inner loop left torque mode — choose a mode to resume (Start if the driver exited)."
            : "Inner-loop MCU left torque mode — press Start to resume.");
    }
    if (!mcu_armed_) mcu_stop_sent_ = false;
    mcu_.sendCmd(m);
}

template <typename T>
void FicControlNode<T>::shutdownMcu() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!use_mcu_ || !mcu_.configured()) return;
    // No Enable: the MCU ramps to gravity (from the pose sent here) and leaves
    // torque mode. Sent a few times — UDP, and nothing will retry after exit.
    mcu_armed_ = false;
    const bool solver_ok = solver_ && solver_->isLoaded();
    const std::vector<double> none(num_joints_, 0.0);
    for (int k = 0; k < 3; ++k) sendMcuCommand(none, solver_ok);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  EXPLICIT TEMPLATE INSTANTIATION
// ═══════════════════════════════════════════════════════════════════════════════

template class FicControlNode<double>;

// ═══════════════════════════════════════════════════════════════════════════════
//  MAIN
// ═══════════════════════════════════════════════════════════════════════════════

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    size_t num_joints = 7;
    auto node = std::make_shared<FicControlNode<double>>(num_joints);
    rclcpp::spin(node);
    node->shutdownMcu();

    // The vendor driver is the inner loop's, not this node's: shutdownMcu() above let the inner loop ramp the
    // arm down, and it then tells the driver to leave torque mode (stop_torque_mode).
    rclcpp::shutdown();
    return 0;
}