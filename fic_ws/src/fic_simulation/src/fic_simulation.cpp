// fic_simulation.cpp — FIC Simulation Node implementation
#include "fic_simulation/fic_simulation.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <cmath>
#include <cstdlib>

// ── geometry_msgs::Pose ↔ Eigen::Isometry3d (solver interface boundary) ──
// Wrap an angle to (-pi, pi], as fic_hardware and the Kinova driver do.
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

template <typename T>
FicSimulationNode<T>::FicSimulationNode(int num_joints)
    : Node("fic_simulation_node"),
      num_joints_(num_joints),
      fic_ctrl_(/* N_s */ 4.0, /* asymmetric */ false, /* N_negSat */ 2.0, 23, Adm_enable)
{
    RCLCPP_INFO(this->get_logger(), "Initializing FicSimulationNode with %zu joints", num_joints_);
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

    // --- Load Robot Configurations from YAML ---
    if (!robot_config_loader_.loadFromDefault()) {
        RCLCPP_WARN(this->get_logger(), "Could not load robots_config.yaml, using built-in defaults");
    } else {
        RCLCPP_INFO(this->get_logger(), "Loaded %zu robot configuration(s) from YAML", robot_config_loader_.size());
    }

    // --- Initialize per-joint state (vectors + filter banks) ---
    // Single sizing path — see resizeJointState. History pre-loaded with 0.0
    // because no measurement exists yet; it is re-seeded from the first genuine
    // sample in jointStateCallback. filter_rate_hz_ is the fallback here;
    // jointStateCallback measures the real joint_states rate and rebuilds these
    // banks once (calibrateFilterRate).
    resizeJointState(num_joints_);

    current_control_mode_ = 1;

    target_pos = Eigen::Vector3d::Ones();
    target_pos << 0.38, 0.004, 0.432;
    current_pos = Eigen::Vector3d::Ones();

    selected_robot_ = "";

    // --- Pre-allocate Loop Variables ---
    loop_current_pose_.resize(7, 0.0);
    loop_target_pose_.resize(7, 0.0);
    loop_wrench_data_.resize(6, 0.0);
    loop_elbow_current_.resize(7, 0.0);
    loop_elbow_target_.resize(7, 0.0);

    feedback_intervals_.reserve(kFilterRateWarmupSamples);
    // Seed the damping-path lag from the fallback rate; recomputed once the
    // real feedback rate is measured (and this value is what the fallback
    // paths in calibrateFilterRate keep).
    updateDampingAlpha();

    // Station ID — determines which robotN topics to use (1-4)
    this->declare_parameter<int>("station_id", 1);
    int station_id = this->get_parameter("station_id").as_int();
    std::string rid = "robot" + std::to_string(station_id);
    RCLCPP_INFO(this->get_logger(), "Station ID: %d — using %s topics", station_id, rid.c_str());

    // Joint name prefix — for multi-robot scenes where MuJoCo element names
    // are prefixed (e.g. "r2_") to avoid collisions.
    this->declare_parameter<std::string>("joint_name_prefix", "");
    joint_name_prefix_ = this->get_parameter("joint_name_prefix").as_string();
    if (!joint_name_prefix_.empty()) {
        RCLCPP_INFO(this->get_logger(), "Joint name prefix: '%s'", joint_name_prefix_.c_str());
    }

    // Source prefix — "simulation" (default) or "vr" for VR headset mode.
    this->declare_parameter<std::string>("source_prefix", "simulation");
    source_prefix_ = this->get_parameter("source_prefix").as_string();
    RCLCPP_INFO(this->get_logger(), "Source prefix: '%s'", source_prefix_.c_str());

    // --- Publishers (station-specific) ---
    trajectory_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(rid + "_trajectory_data", 10);
    torque_command_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/" + rid + "_fic_torque_commands", qos);
    control_publisher_ = this->create_publisher<communication::msg::ActuatorCmds>("/" + source_prefix_ + "/actuators_cmds", qos);
    actuator_torque_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/" + source_prefix_ + "/actuator_torques", qos);
    jacobian_publisher_ = this->create_publisher<communication::msg::Jacobian>(rid + "_jacobian", qos);
    ee_pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>(rid + "_ee_current_pose", 10);

    // Accumulated per-joint "human push offset" (radians) from joint admittance
    // — the controller's own integrated yield, NOT planner lag (contrast
    // trajectory_data blocks 3/5). Published ONLY while joint admittance is
    // active (modes JOINT_ADMITTANCE_M1/M2, i.e. 3/4); see the publish call
    // near the end of the control loop. Matches fic_hardware's publisher.
    admittance_offset_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        rid + "_admittance_offset", rclcpp::SensorDataQoS());

    // --- Subscribers (station-specific topics) ---
    joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/" + source_prefix_ + "/joint_states", qos, std::bind(&FicSimulationNode::jointStateCallback, this, std::placeholders::_1));

    target_position_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "command_ee_position_change_" + rid, qos, std::bind(&FicSimulationNode::targetPositionCallback, this, std::placeholders::_1));

    force_torque_subscriber_ = this->create_subscription<geometry_msgs::msg::Wrench>(
        "/" + source_prefix_ + "/force_torque", qos, std::bind(&FicSimulationNode::forceTorqueCallback, this, std::placeholders::_1));

    gravity_torque_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/" + source_prefix_ + "/gravity_torques", qos, std::bind(&FicSimulationNode::gravityTorqueCallback, this, std::placeholders::_1));

    contact_torque_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/" + source_prefix_ + "/contact_torques", qos, std::bind(&FicSimulationNode::contactTorqueCallback, this, std::placeholders::_1));

    control_mode_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
        rid + "_control_mode", qos, std::bind(&FicSimulationNode::controlModeCallback, this, std::placeholders::_1));

    num_joints_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
        rid + "_number_of_joints", qos, std::bind(&FicSimulationNode::numberOfJointsCallback, this, std::placeholders::_1));

    robot_selection_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        rid + "_selected_robot", qos, std::bind(&FicSimulationNode::robotSelectionCallback, this, std::placeholders::_1));

    // Reliable + transient-local (as FIC Studio and the experiment runner publish it): a
    // profile selected before this node started is still delivered on start-up.
    active_profile_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        rid + "_active_profile", rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&FicSimulationNode::activeProfileCallback, this, std::placeholders::_1));

    // Explicit "the tuning on disk changed, re-read it" signal. targetJointCallback
    // used to reload config.xml on every streamed target, which is why it was
    // stopped for mode 1 — but that was the *only* runtime reload trigger, so
    // stopping it silently killed live re-tuning in Joint Impedance for every
    // robot. This restores the reload without tying it to the target stream.
    param_update_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        rid + "_param_update", 10, std::bind(&FicSimulationNode::paramUpdateCallback, this, std::placeholders::_1));

    desired_joint_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        rid + "_desired_joint_angles", 10, std::bind(&FicSimulationNode::desiredJointCallback, this, std::placeholders::_1));

    joint_target_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        rid + "_target_joint_angles", qos, std::bind(&FicSimulationNode::targetJointCallback, this, std::placeholders::_1));

    gripper_width_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
        rid + "_gripper_width", 10, std::bind(&FicSimulationNode::gripperWidthCallback, this, std::placeholders::_1));

    // --- Safety Limit Subscribers ---
    cart_safety_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        rid + "_safety/cartesian_limits", 10,
        std::bind(&FicSimulationNode::cartesianSafetyCallback, this, std::placeholders::_1));

    joint_safety_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        rid + "_safety/joint_limits", 10,
        std::bind(&FicSimulationNode::jointSafetyCallback, this, std::placeholders::_1));

    vel_safety_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        rid + "_safety/velocity_limits", 10,
        std::bind(&FicSimulationNode::velocitySafetyCallback, this, std::placeholders::_1));

    postural_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        rid + "_postural_enable", 10,
        std::bind(&FicSimulationNode::posturalEnableCallback, this, std::placeholders::_1));

    elbow_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        rid + "_elbow_enable", 10,
        std::bind(&FicSimulationNode::elbowEnableCallback, this, std::placeholders::_1));

    // the old topic name, still accepted
    hpmp_enable_legacy_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        rid + "_hapfic_enable", 10,
        std::bind(&FicSimulationNode::hpmpEnableCallback, this, std::placeholders::_1));
    // H-PMP's topic (since 5.7.0).
    hpmp_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        rid + "_hpmp_enable", 10,
        std::bind(&FicSimulationNode::hpmpEnableCallback, this, std::placeholders::_1));

    desired_wrench_sub_ = this->create_subscription<geometry_msgs::msg::Wrench>(
        rid + "_desired_wrench", 10,
        std::bind(&FicSimulationNode::desiredWrenchCallback, this, std::placeholders::_1));

    admittance_planner_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        rid + "_admittance_planner_enable", 10,
        std::bind(&FicSimulationNode::admittancePlannerCallback, this, std::placeholders::_1));

    gravity_comp_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        rid + "_gravity_compensated", rclcpp::QoS(1).transient_local(),
        std::bind(&FicSimulationNode::gravityCompCallback, this, std::placeholders::_1));

    wrench_gravity_comp_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        rid + "_wrench_gravity_compensated", rclcpp::QoS(1).transient_local(),
        std::bind(&FicSimulationNode::wrenchGravityCompCallback, this, std::placeholders::_1));

    wrench_from_api_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        rid + "_wrench_from_api", rclcpp::QoS(1).transient_local(),
        std::bind(&FicSimulationNode::wrenchFromApiCallback, this, std::placeholders::_1));

    // --- Declare and load robot from parameter ---
    this->declare_parameter<std::string>("selected_robot", "KinovaGen3_7DOF");
    this->declare_parameter<int>("number_of_joints", 7);

    std::string robot_from_param = this->get_parameter("selected_robot").as_string();
    int joints_from_param = this->get_parameter("number_of_joints").as_int();

    if (joints_from_param <= 0) {
        RCLCPP_ERROR(this->get_logger(),
            "Ignoring invalid number_of_joints parameter %d — keeping %zu.",
            joints_from_param, num_joints_);
    } else if (joints_from_param != (int)num_joints_) {
        // One sizing path for every per-joint buffer — see resizeJointState.
        resizeJointState(static_cast<size_t>(joints_from_param));
    }

    if (!robot_from_param.empty() && robot_from_param != "Default_Robot") {
        selected_robot_ = robot_from_param;
        robot_selection_received_ = true;
        number_of_joints_received_ = true;
        loadRobotModel(selected_robot_);
        RCLCPP_INFO(this->get_logger(), "Loaded robot from parameter: %s (%zu joints)",
                    selected_robot_.c_str(), num_joints_);
    }

    // Timer
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&FicSimulationNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "FicSimulationNode initialization complete.");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  PER-JOINT STATE SIZING
// ═══════════════════════════════════════════════════════════════════════════════

// The single place num_joints_ changes, and therefore the single place every
// per-joint buffer is sized. Two callers set the joint count — the launch
// parameter and the GUI's <rid>_number_of_joints topic — and they used to size
// different subsets of the state: the topic path grew num_joints_ but resized
// only loop_traj_data_/loop_torque_data_, leaving q_current_, q_target,
// gravity_torques_, currentVelocities_, current_effort and the filter banks at
// the old length. jointStateCallback and controlLoop then indexed those buffers
// up to the new num_joints_, i.e. off the end of the heap allocation — garbage
// joint angles (and a latent crash) rather than a diverging controller.
template <typename T>
void FicSimulationNode<T>::resizeJointState(size_t n) {
    num_joints_ = n;

    q_current_       = Eigen::VectorXd::Zero(n);
    q_target         = Eigen::VectorXd::Zero(n);
    q_target_ik_     = Eigen::VectorXd::Zero(n);
    gravity_torques_ = Eigen::VectorXd::Zero(n);
    jacobian         = Eigen::MatrixXd::Zero(6, n);

    currentVelocities_.assign(n, static_cast<T>(0.0));
    rawVelocities_.assign(n, static_cast<T>(0.0));
    dampingVelocities_.assign(n, static_cast<T>(0.0));
    current_effort.assign(n, static_cast<T>(0.0));

    loop_traj_data_.assign(n * 8, 0.0);
    loop_torque_data_.assign(n, 0.0);

    // Seeded exactly as the constructor used to seed them: zero history, then
    // re-seeded from the first genuine sample in jointStateCallback. Rebuilt at
    // filter_rate_hz_, which by now may hold the measured feedback rate.
    torque_filters_.assign(n, ButterworthFilter(torqueCutoffHz(), filter_rate_hz_, 0.0));
    position_filters_.assign(n, ButterworthFilter(kStateCutoffHz, filter_rate_hz_, 0.0));
    raw_unwrapped_.assign(n, 0.0);
    prev_raw_.assign(n, 0.0);
    velocity_filters_.assign(n, ButterworthFilter(kStateCutoffHz, filter_rate_hz_, 0.0));

    // The buffers now hold zeros, not the robot's state. Re-arm the
    // genuine-feedback latch so neither the impedance law nor the joint-limit
    // hard-fault check runs against that fictitious state, and force the
    // controllers to be re-initialised at the new joint count.
    joint_state_received_ = false;
    initControllerPLanner = false;
}

// Defensive backstop for the loops that index the per-joint buffers. Returns
// num_joints_ whenever resizeJointState is the only thing that changed it — if
// some future path sets num_joints_ directly again, this clamps to what is
// actually allocated and says so loudly, so the mismatch shows up as degraded
// control in the log instead of a silent out-of-bounds write.
template <typename T>
size_t FicSimulationNode<T>::checkedJointCount(const char* where) {
    size_t n = num_joints_;
    n = std::min(n, static_cast<size_t>(q_current_.size()));
    n = std::min(n, static_cast<size_t>(q_target.size()));
    n = std::min(n, static_cast<size_t>(gravity_torques_.size()));
    n = std::min(n, static_cast<size_t>(jacobian.cols()));
    n = std::min(n, currentVelocities_.size());
    n = std::min(n, rawVelocities_.size());
    n = std::min(n, dampingVelocities_.size());
    n = std::min(n, current_effort.size());
    n = std::min(n, torque_filters_.size());
    n = std::min(n, position_filters_.size());
    n = std::min(n, velocity_filters_.size());
    n = std::min(n, loop_torque_data_.size());
    if (n != num_joints_) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "BUG: num_joints_ is %zu but the per-joint state buffers hold only %zu "
            "entries (%s). Something changed the joint count without going through "
            "resizeJointState(). Clamping to %zu — control is DEGRADED until the "
            "robot is reloaded.",
            num_joints_, n, where, n);
    }
    return n;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  ROBOT MODEL
// ═══════════════════════════════════════════════════════════════════════════════

template <typename T>
void FicSimulationNode<T>::loadRobotModel(const std::string& robot_name) {
    std::string current_path = std::filesystem::current_path().string();

    robot_config_loader_.loadFromDefault();
    auto config_opt = robot_config_loader_.getConfig(robot_name);
    if (config_opt.has_value()) {
        current_robot_config_ = config_opt.value();
        RCLCPP_INFO(this->get_logger(), "Loaded config from YAML for: %s", robot_name.c_str());
    } else {
        current_robot_config_ = fic::getDefaultConfig(robot_name);
        RCLCPP_WARN(this->get_logger(), "Using built-in defaults for: %s", robot_name.c_str());
    }

    // FIC_HARDWARE_MODEL=off: the ideal simulator for this run whatever robots_config.yaml says
    // (mujoco_native/hardware_model.py reads the same variable).
    if (const char* hm = std::getenv("FIC_HARDWARE_MODEL")) {
        std::string v(hm);
        std::transform(v.begin(), v.end(), v.begin(), ::tolower);
        if (v == "off" || v == "0" || v == "false" || v == "ideal")
            current_robot_config_.hardware_model.enabled = false;
    }

    relevant_joints = current_robot_config_.joint_names;
    ee_frame_name_ = current_robot_config_.ee_frame_name;
    elbow_frame_name_ = current_robot_config_.elbow_frame_name;

    prefixed_joint_names_.clear();
    for (const auto& jn : relevant_joints) {
        prefixed_joint_names_.push_back(joint_name_prefix_ + jn);
    }

    ee_target.position.x = current_robot_config_.default_ee_position[0];
    ee_target.position.y = current_robot_config_.default_ee_position[1];
    ee_target.position.z = current_robot_config_.default_ee_position[2];
    ee_target.orientation.x = current_robot_config_.default_ee_orientation[0];
    ee_target.orientation.y = current_robot_config_.default_ee_orientation[1];
    ee_target.orientation.z = current_robot_config_.default_ee_orientation[2];
    ee_target.orientation.w = current_robot_config_.default_ee_orientation[3];

    if (!current_robot_config_.q_home_rad.empty() && num_joints_ >= current_robot_config_.q_home_rad.size()) {
        for (size_t i = 0; i < current_robot_config_.q_home_rad.size() && i < num_joints_; ++i) {
            q_target(i) = current_robot_config_.q_home_rad[i];
        }
    }

    std::string urdf_file = current_path + "/src/" + current_robot_config_.urdf_path;

    // Create the per-robot kinematics solver (robots_config.yaml: kinematics_solver,
    // default "pinocchio"; "none" = deliberate degraded joint-space-only mode).
    {
        std::string factory_err;
        solver_ = fic::createKinematicsSolver(current_robot_config_.kinematics_solver, &factory_err);
        if (!solver_ && !factory_err.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Kinematics solver: %s", factory_err.c_str());
        }
    }

    if (solver_) {
        if (!solver_->loadModel(urdf_file)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load robot model: %s",
                         solver_->lastError().c_str());
            solver_.reset();
        } else {
            // Actuated-joint order, so lowerPositionLimit(i)/upperPositionLimit(i)
            // map onto the configuration vector. Only an identity mapping when
            // nq == nv; a `continuous` joint occupies two entries in q, shifting
            // every bounded joint after it. See KinematicsSolver's contract.
            solver_->configureActuatedJoints(current_robot_config_.joint_names);

            // Closed-loop / parallel mechanism: hand the solver its loop closures
            // + EE so it presents the reduced actuated-joint interface. No-op for
            // serial robots (empty loop_closures / non-closed-loop solvers).
            if (!current_robot_config_.loop_closures.empty()) {
                std::vector<fic::KinematicsSolver::LoopClosure> loops;
                for (const auto& lc : current_robot_config_.loop_closures) {
                    fic::KinematicsSolver::LoopClosure L;
                    L.bodyA = lc.bodyA; L.bodyB = lc.bodyB;
                    L.posA = Eigen::Vector3d(lc.posA[0], lc.posA[1], lc.posA[2]);
                    L.posB = Eigen::Vector3d(lc.posB[0], lc.posB[1], lc.posB[2]);
                    loops.push_back(L);
                }
                solver_->configureClosedLoop(loops, current_robot_config_.joint_names,
                                             ee_frame_name_, current_robot_config_.ee_orientation_only);
                RCLCPP_INFO(this->get_logger(),
                    "  Closed-loop: %zu loop closures, EE '%s'%s (reduced nq=%d nv=%d)",
                    loops.size(), ee_frame_name_.c_str(),
                    current_robot_config_.ee_orientation_only ? ", orientation-only" : "",
                    solver_->nq(), solver_->nv());
            }
            RCLCPP_INFO(this->get_logger(), "Loaded robot model: %s (nq: %d, nv: %d, solver: %s)",
                        robot_name.c_str(), solver_->nq(), solver_->nv(), solver_->name().c_str());
            RCLCPP_INFO(this->get_logger(), "  EE Frame: %s, Elbow Frame: %s", ee_frame_name_.c_str(), elbow_frame_name_.c_str());
        }
    }

    if (!solver_) {
        RCLCPP_WARN(this->get_logger(),
            "Running WITHOUT a kinematics solver (kinematics_solver: '%s').\n"
            "  Available:   joint-space control modes 1-4, MuJoCo gravity compensation\n"
            "  Disabled:    Cartesian modes 5-9, IK targets, EE pose/Jacobian publishing,\n"
            "               Cartesian workspace guard (fail-open) and virtual walls\n"
            "  To restore:  install pinocchio and rebuild, or register a custom solver\n"
            "               (see docs/custom-kinematics-solver.md)",
            current_robot_config_.kinematics_solver.c_str());
    }

    // Install robot-level joint metadata (continuity + limits) once per robot load,
    // sourced from the solver's model (URDF). Cartesian modes skip initJointPlanner,
    // so this is the only path that sets it. Continuous (unbounded) joints in the URDF
    // get Pinocchio's huge sentinel (~1e19) — anything above 1e6 rad is effectively
    // infinite, which correctly excludes multi-turn finite joints (e.g. ±720°).
    {
        // Continuity from URDF (authoritative — missing <limit> lower/upper = continuous).
        // Bounded-joint clamp limits from the solver's model, or straight from the
        // URDF XML in degraded mode.
        const size_t n = solver_ ? static_cast<size_t>(solver_->nv()) : num_joints_;
        std::vector<bool> continuous =
            fic::deriveJointContinuityFromUrdf(urdf_file, current_robot_config_.joint_names);
        continuous.resize(n, false);
        // Kept for the joint-state filter, which must unwrap exactly these.
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
    }

    // Per-joint torque limits from robots_config.yaml (physical actuator limits)
    per_joint_torque_limit_ = current_robot_config_.joint_torque_max;
    hard_fault_ = false;

    // Per-robot velocity safety from robots_config.yaml (safety_zones.velocity).
    // The damping this drives is kd = per_joint_torque_limit / max_joint_velocity,
    // so a robot with large actuators needs a matching velocity limit from the very
    // first control cycle — waiting for the GUI's
    // `robot{N}_safety/velocity_limits` broadcast is too late (and a headless
    // launch never sends it). Absent keys leave the built-in defaults untouched,
    // so this is a no-op for every robot that does not declare the block; runtime
    // messages on the topic still override it.
    if (current_robot_config_.safety_max_joint_vel)
        max_joint_velocity_ = *current_robot_config_.safety_max_joint_vel;
    // A simulation-only limit (sim_max_joint_vel) takes precedence here, unless the robot's
    // hardware_model is on: the simulated arm then has the robot's velocity damper.
    if (current_robot_config_.safety_sim_max_joint_vel && !current_robot_config_.hardware_model.enabled)
        max_joint_velocity_ = *current_robot_config_.safety_sim_max_joint_vel;
    if (current_robot_config_.safety_max_cart_vel)
        max_cart_velocity_ = *current_robot_config_.safety_max_cart_vel;
    if (current_robot_config_.safety_velocity_enabled)
        velocity_limits_enabled_ = *current_robot_config_.safety_velocity_enabled;
    if (current_robot_config_.safety_max_joint_vel ||
        current_robot_config_.safety_sim_max_joint_vel ||
        current_robot_config_.safety_max_cart_vel ||
        current_robot_config_.safety_velocity_enabled) {
        RCLCPP_INFO(this->get_logger(),
            "Velocity safety from robots_config.yaml: joint=%.2f rad/s, "
            "cart=%.2f m/s (enabled: %s)",
            max_joint_velocity_, max_cart_velocity_,
            velocity_limits_enabled_ ? "true" : "false");
    }

    // Apply per-robot admittance deadbands loaded from robots_config.yaml. Simulating the arm
    // (hardware_model on), the Cartesian force deadband is the arm's (cart_force_hardware), as
    // fic_hardware sets it; the ideal simulator keeps cart_force.
    const double cart_force_db = current_robot_config_.hardware_model.enabled
        ? current_robot_config_.cartForceDeadbandOnArm() : current_robot_config_.deadband_cart_force;
    const double cart_torque_db = current_robot_config_.hardware_model.enabled
        ? current_robot_config_.cartTorqueDeadbandOnArm() : current_robot_config_.deadband_cart_torque;
    fic_ctrl_.setDeadbands(
        current_robot_config_.deadband_joint_tau,
        cart_force_db,
        cart_torque_db);
    // H-PMP's split admittance drive keeps the library's task deadband: the simulated measured
    // torque is the node's own command, with no sensor bias for a wider deadband to absorb. With the
    // robot's hardware_model the measured torque carries the arm's friction and sensor noise, and the
    // deadband is the robot's, as fic_hardware sets it.
    if (current_robot_config_.hardware_model.enabled)
        fic_ctrl_.setHpmpSplitDrive(true, cart_force_db, cart_torque_db);
    else
        fic_ctrl_.setHpmpSplitDrive(true, 1.0, 0.1);
    RCLCPP_INFO(this->get_logger(), "Hardware model (robots_config.yaml hardware_model): %s, measured-torque filter %.0f Hz",
        current_robot_config_.hardware_model.enabled ? "on" : "off (ideal simulator)", torqueCutoffHz());
    RCLCPP_INFO(this->get_logger(),
        "H-PMP admittance drive: %s, task deadband %.2f N / %.3f N m, joint deadband %.2f N m; "
        "Cartesian admittance deadband %.2f N / %.2f N m",
        fic_ctrl_.hpmp_split_drive_ ? "split" : "per joint", fic_ctrl_.hpmp_task_db_force_,
        fic_ctrl_.hpmp_task_db_torque_, fic_ctrl_.deadband_joint_tau_, fic_ctrl_.deadband_cart_force_,
        fic_ctrl_.deadband_cart_torque_);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  KINEMATICS
// ═══════════════════════════════════════════════════════════════════════════════

template <typename T>
Eigen::Vector3d FicSimulationNode<T>::rotVecFromQuatError(
    const Eigen::Quaterniond& q_curr, const Eigen::Quaterniond& q_des)
{
    Eigen::Quaterniond q_err = q_des * q_curr.conjugate();
    if (q_err.w() < 0) q_err.coeffs() *= -1.0;
    const double vnorm = q_err.vec().norm();
    if (vnorm < 1e-12) return Eigen::Vector3d::Zero();
    return (2.0 * std::atan2(vnorm, q_err.w()) / vnorm) * q_err.vec();
}

template <typename T>
Eigen::VectorXd FicSimulationNode<T>::solveFullIterativeIK(
    const Eigen::VectorXd& q_start, const geometry_msgs::msg::Pose& ee_target_des)
{
    if (!solver_ || !solver_->isLoaded()) return q_start;
    if (!solver_->hasFrame(ee_frame_name_)) return q_start;

    fic::KinematicsSolver::IKOptions opts;
    opts.tolerance          = 1e-6;
    opts.max_iterations     = 5000;
    opts.dt                 = 1e-2;
    opts.damping            = 1e-6;
    opts.joint_based        = false;   // frame-based CLIK (LOCAL convention)
    opts.max_joint_velocity = 0.5;

    const auto r = solver_->solveIK(q_start, poseMsgToIso(ee_target_des), ee_frame_name_, opts);

    if (r.success) RCLCPP_INFO(this->get_logger(), "IK: Convergence (Error: %.6f).", r.error_norm);
    else RCLCPP_WARN(this->get_logger(), "IK: Failed to converge.");

    return r.q;
}

template <typename T>
void FicSimulationNode<T>::computeAndPublishJacobian(const Eigen::VectorXd& q) {
    if (!solver_ || !solver_->isLoaded()) return;

    solver_->updateKinematics(q);

    Eigen::Isometry3d ee_T;
    if (!solver_->cachedFramePose(ee_frame_name_, ee_T)) {
        RCLCPP_ERROR(this->get_logger(), "EE frame '%s' not found!", ee_frame_name_.c_str());
        return;
    }

    fic::KinematicsSolver::Matrix6x J_local(6, num_joints_);
    J_local.setZero();
    solver_->frameJacobian(q, ee_frame_name_, fic::KinematicsSolver::JacobianFrame::LocalWorldAligned, J_local);
    jacobian = J_local;

    // Elbow
    Eigen::Isometry3d elbow_T;
    if (solver_->cachedFramePose(elbow_frame_name_, elbow_T)) {
        elbow_current_pose_.position.x = elbow_T.translation().x();
        elbow_current_pose_.position.y = elbow_T.translation().y();
        elbow_current_pose_.position.z = elbow_T.translation().z();
        fic::KinematicsSolver::Matrix6x J_elbow(6, num_joints_);
        J_elbow.setZero();
        solver_->frameJacobian(q, elbow_frame_name_, fic::KinematicsSolver::JacobianFrame::LocalWorldAligned, J_elbow);
        J_elbow_6D = J_elbow;
    } else {
        J_elbow_6D.setZero(6, num_joints_);
    }

    // Publish Jacobian
    communication::msg::Jacobian jacobian_msg;
    jacobian_msg.header.stamp = this->get_clock()->now();
    jacobian_msg.rows = 6;
    jacobian_msg.cols = num_joints_;
    jacobian_msg.jacobian.resize(6 * num_joints_);
    Eigen::Map<Eigen::MatrixXd>(jacobian_msg.jacobian.data(), 6, num_joints_) = J_local;
    jacobian_publisher_->publish(jacobian_msg);

    // Gravity — prefer MuJoCo's (the simulated plant) over the solver's (gripper URDF when has_gripper)
    if (!mujoco_gravity_received_) {
        gravity_torques_ = solver_->gravity(q);
    }

    // EE pose cache
    ee_current = isoToPoseMsg(ee_T);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  ROS CALLBACKS
// ═══════════════════════════════════════════════════════════════════════════════

template <typename T>
void FicSimulationNode<T>::gripperWidthCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    // add gripper logic
}

template <typename T>
void FicSimulationNode<T>::numberOfJointsCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (msg->data <= 0) {
        RCLCPP_ERROR(this->get_logger(),
            "Ignoring invalid joint count %d on the number_of_joints topic — keeping %zu.",
            msg->data, num_joints_);
        return;
    }
    if ((size_t)msg->data != num_joints_) {
        // Must go through resizeJointState: growing num_joints_ on its own
        // leaves every per-joint buffer short and the control loop then indexes
        // past the end of them. Same call the constructor's parameter path uses.
        resizeJointState(static_cast<size_t>(msg->data));
    }
    number_of_joints_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Number of joints updated to: %zu", num_joints_);
}

template <typename T>
void FicSimulationNode<T>::targetJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // Re-init exists so the planner/admittance blocks re-seed at the current
    // position when a new goal arrives. Pure joint impedance (mode 1) reads
    // none of that — it tracks whatever q_target holds — so re-initialising it
    // per message is pure cost: at the gait node's 200 Hz it re-parses
    // config.xml for every joint and never lets the loop keep its period.
    if (current_control_mode_ != static_cast<int>(ControlMode::JOINT_IMPEDANCE))
        initControllerPLanner = false;
    if (msg->position.size() != num_joints_) return;
    Eigen::VectorXd q_candidate = q_target;
    // q_target is sized by resizeJointState alongside num_joints_; if it ever
    // is not, writing num_joints_ entries into it would run off the end.
    if (q_candidate.size() != (Eigen::Index)num_joints_) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "BUG: q_target holds %ld entries but num_joints_ is %zu — dropping joint target.",
            (long)q_candidate.size(), num_joints_);
        return;
    }
    for (size_t i = 0; i < num_joints_; ++i) q_candidate(i) = msg->position[i];
    applyJointSafetyLimits(q_candidate);
    // Workspace guard (joint mode): reject a target that would take the EE
    // outside the enabled Cartesian zone; otherwise accept it.
    if (eeTargetInsideZone(q_candidate)) q_target = q_candidate;
}

template <typename T>
void FicSimulationNode<T>::posturalEnableCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    postural_enabled_ = msg->data;
    if (postural_enabled_) postural_reset_ = true;   // start from the arm
    RCLCPP_INFO(this->get_logger(), "Postural control %s", postural_enabled_ ? "ENABLED" : "DISABLED");
}

template <typename T>
void FicSimulationNode<T>::elbowEnableCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    elbow_enabled_ = msg->data;
    if (elbow_enabled_) {
        elbow_target_pose_ = elbow_current_pose_;
    }
    RCLCPP_INFO(this->get_logger(), "Elbow control %s", elbow_enabled_ ? "ENABLED" : "DISABLED");
}

template <typename T>
void FicSimulationNode<T>::hpmpEnableCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (hpmp_enabled_ == msg->data) return;   // a repeat changes nothing
    hpmp_enabled_ = msg->data;
    initControllerPLanner = false;
    RCLCPP_INFO(this->get_logger(), "H-PMP %s", hpmp_enabled_ ? "ENABLED" : "DISABLED");
}

template <typename T>
void FicSimulationNode<T>::desiredWrenchCallback(const geometry_msgs::msg::Wrench::SharedPtr msg) {
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
void FicSimulationNode<T>::admittancePlannerCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    admittance_use_planner_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Admittance planner %s", admittance_use_planner_ ? "ENABLED" : "DISABLED");
}

template <typename T>
void FicSimulationNode<T>::gravityCompCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_robot_config_.gravity_compensated = msg->data;
    RCLCPP_INFO(this->get_logger(), "gravity_compensated = %s", msg->data ? "true" : "false");
}

template <typename T>
void FicSimulationNode<T>::wrenchGravityCompCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_robot_config_.wrench_gravity_compensated = msg->data;
    RCLCPP_INFO(this->get_logger(), "wrench_gravity_compensated = %s", msg->data ? "true" : "false");
}

template <typename T>
void FicSimulationNode<T>::wrenchFromApiCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_robot_config_.wrench_from_api = msg->data;
    RCLCPP_INFO(this->get_logger(), "wrench_from_api = %s", msg->data ? "true" : "false");
}

template <typename T>
void FicSimulationNode<T>::robotSelectionCallback(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    selected_robot_ = msg->data;
    initControllerPLanner = false;
    robot_selection_received_ = true;
    loadRobotModel(selected_robot_);
    RCLCPP_INFO(this->get_logger(), "Robot selected: %s", selected_robot_.c_str());
}

template <typename T>
void FicSimulationNode<T>::activeProfileCallback(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::string new_profile = msg->data.empty() ? "default" : msg->data;
    if (new_profile == active_profile_) return;
    active_profile_ = new_profile;
    tuning_reload_ = true;   // re-read config.xml with the new profile now (as fic_hardware)
    RCLCPP_INFO(this->get_logger(), "Active tuning profile: %s", active_profile_.c_str());
}

template <typename T>
void FicSimulationNode<T>::paramUpdateCallback(const std_msgs::msg::String::SharedPtr msg) {
    // The GUI publishes "<rid>:<param_type>:<value>" here after writing new
    // tuning to config.xml. refreshTuning() then reads the file once, off the
    // control thread; the controllers keep the old tuning until the new set is
    // ready and switch to it on the next tick.
    std::lock_guard<std::mutex> lock(data_mutex_);
    tuning_reload_ = true;
    RCLCPP_INFO(this->get_logger(), "Parameter update (%s); reloading tuning from config.xml",
                msg->data.c_str());
}

template <typename T>
void FicSimulationNode<T>::desiredJointCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    size_t len = std::min(msg->data.size(), size_t(7));
    // Same rule as fic_hardware's desiredJointCallback: pure Joint Impedance
    // (mode 1) just tracks q_target, so a streamed target (teleop, gamepad,
    // trajectory player) must not re-initialise the controller per message.
    if (current_control_mode_ != 1 &&
        (admittance_use_planner_ || current_control_mode_ <= 2)) {
        initControllerPLanner = false;
    }
    Eigen::VectorXd q_candidate = q_target;
    for (size_t i = 0; i < len; ++i) q_candidate(i) = msg->data[i];
    applyJointSafetyLimits(q_candidate);
    // Workspace guard (joint mode): reject a target that would take the EE
    // outside the enabled Cartesian zone; otherwise accept it.
    if (eeTargetInsideZone(q_candidate)) q_target = q_candidate;
}

template <typename T>
void FicSimulationNode<T>::calibrateFilterRate(double stamp_s) {
    // One-shot. Runs only while the warm-up window is filling; once latched the
    // filter banks are never rebuilt again, so nothing here can print or
    // reallocate at the feedback rate.
    if (filter_rate_established_) return;

    if (last_feedback_stamp_s_ > 0.0) {
        const double dt = stamp_s - last_feedback_stamp_s_;
        // Reject only nonsense samples (clock jumps, duplicate stamps,
        // multi-second stalls). Genuine jitter is kept and handled by the
        // median below — discarding it here would bias the estimate.
        if (dt > 1.0e-6 && dt < 1.0) feedback_intervals_.push_back(dt);
    } else {
        first_feedback_stamp_s_ = stamp_s;
        filter_rate_deadline_s_ = this->now().seconds() + 2.0 * kFilterRateWarmupMaxSec;
    }
    last_feedback_stamp_s_ = stamp_s;

    // Commit when the window is full, or when it has taken too long — a slow
    // publisher must not stall the control loop indefinitely.
    const bool window_full = feedback_intervals_.size() >= kFilterRateWarmupSamples;
    const bool timed_out = (stamp_s - first_feedback_stamp_s_) > kFilterRateWarmupMaxSec;
    if (!window_full && !timed_out) return;

    if (feedback_intervals_.size() < kFilterRateMinSamples) {
        filter_rate_established_ = true;
        RCLCPP_WARN(this->get_logger(),
            "Only %zu joint_states intervals in %.1f s — too few to measure a feedback rate; "
            "keeping fallback filter rate %.1f Hz. State filters may lag.",
            feedback_intervals_.size(), kFilterRateWarmupMaxSec, filter_rate_hz_);
        feedback_intervals_.clear();
        return;
    }

    // Median inter-arrival period: robust to the occasional late or bunched
    // message that a mean would let drag the estimate.
    std::vector<double> sorted = feedback_intervals_;
    std::sort(sorted.begin(), sorted.end());
    const double median_dt = sorted[sorted.size() / 2];
    const double measured_hz = (median_dt > 0.0) ? (1.0 / median_dt) : 0.0;

    // Guard 1: plausible range. Guard 2: the stream must actually be periodic —
    // most intervals within +-50% of the median — or the median is meaningless.
    size_t consistent = 0;
    for (double d : sorted) {
        if (d > 0.5 * median_dt && d < 1.5 * median_dt) ++consistent;
    }
    const double consistency = static_cast<double>(consistent) / static_cast<double>(sorted.size());

    filter_rate_established_ = true;   // latch either way: measured once, not per message
    feedback_intervals_.clear();
    feedback_intervals_.shrink_to_fit();

    if (measured_hz < kFilterRateMinHz || measured_hz > kFilterRateMaxHz ||
        consistency < kFilterRateConsistency) {
        RCLCPP_WARN(this->get_logger(),
            "joint_states rate could not be established (median %.1f Hz, %.0f%% periodic); "
            "keeping fallback filter rate %.1f Hz. State filters may lag.",
            measured_hz, consistency * 100.0, filter_rate_hz_);
        return;
    }

    if (std::abs(measured_hz - filter_rate_hz_) < 1.0) {
        RCLCPP_INFO(this->get_logger(),
            "Measured joint_states rate %.1f Hz matches the filter rate already in use.",
            measured_hz);
        return;
    }

    const double old_hz = filter_rate_hz_;
    filter_rate_hz_ = measured_hz;
    // Rebuild seeded from the current filtered state, so swapping coefficients
    // is a bumpless transfer rather than a step into the impedance law.
    for (size_t i = 0; i < torque_filters_.size(); ++i) {
        torque_filters_[i]   = ButterworthFilter(torqueCutoffHz(), filter_rate_hz_,
                                                 static_cast<double>(current_effort[i]));
        velocity_filters_[i] = ButterworthFilter(kStateCutoffHz, filter_rate_hz_,
                                                 static_cast<double>(currentVelocities_[i]));
        // A continuous joint's filter runs on the UNWRAPPED angle; priming
        // it with the wrapped q_current_ would inject a 2*pi step -- the
        // very sweep the unwrap exists to prevent. Bring the filtered
        // value onto the branch the unwrapped raw angle is on.
        const double prime = isContinuousJoint(i)
            ? raw_unwrapped_[i] + wrapAngle(q_current_[i] - raw_unwrapped_[i])
            : q_current_[i];
        position_filters_[i] = ButterworthFilter(kStateCutoffHz, filter_rate_hz_,
                                                 prime);
    }
    updateDampingAlpha();
    RCLCPP_INFO(this->get_logger(),
        "Measured joint_states rate %.1f Hz (was assuming %.1f Hz); state/torque filters "
        "rebuilt at %.1f Hz cutoff. Effective cutoff was %.2f Hz, now %.2f Hz.",
        filter_rate_hz_, old_hz, kStateCutoffHz,
        kStateCutoffHz * filter_rate_hz_ / old_hz, kStateCutoffHz);
}

template <typename T>
void FicSimulationNode<T>::updateDampingAlpha() {
    // First-order lag pole for the damping path. exp(-2*pi*fc/fs): fs is the
    // rate at which the damping value is actually refreshed, i.e. the measured
    // joint_states rate, not the control-loop rate. 0 Hz disables the filter
    // and uses the raw velocity directly.
    double fc = kDampingCutoffHz;
    // Experiment hook: overriding the cutoff from the environment lets the
    // damping path be swept without a rebuild. Unset in normal operation.
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
        "Damping-path filter: 1st-order %.1f Hz at %.1f Hz feedback -> alpha %.4f "
        "(phase lag %.0f deg at 15.9 Hz).",
        fc, filter_rate_hz_, damping_alpha_,
        fc > 0.0 ? std::atan(15.9 / fc) * 180.0 / M_PI : 0.0);
}

template <typename T>
void FicSimulationNode<T>::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!number_of_joints_received_ || !robot_selection_received_ || selected_robot_.empty()) return;

    // Timestamp for the rate measurement: the publisher's own stamp when it has
    // one (it tracks the cadence the samples were produced at), otherwise
    // arrival time. Only messages that reach here step the filters, so this is
    // exactly the rate the filters run at.
    double stamp_s = rclcpp::Time(msg->header.stamp).seconds();
    if (stamp_s <= 0.0) stamp_s = this->now().seconds();

    const bool solver_ok = solver_ && solver_->isLoaded();
    Eigen::VectorXd q = Eigen::VectorXd::Zero(solver_ok ? solver_->nq() : (int)num_joints_);
    size_t joints_updated = 0;
    // Never index past the per-joint buffers: a joint count that grew without
    // them leaves q_current_ partly unset, which the joints_updated latch below
    // then correctly refuses to accept as feedback. checkedJointCount logs the
    // mismatch; with resizeJointState as the only writer it returns num_joints_.
    const size_t n_safe = checkedJointCount("jointStateCallback");
    if (torqueCutoffHz() != torque_cutoff_applied_hz_) {
        // The robot (and so its hardware_model) changed: rebuild the torque filters, seeded from
        // the current filtered value (bumpless).
        torque_cutoff_applied_hz_ = torqueCutoffHz();
        if (joint_state_received_)
            for (size_t i = 0; i < torque_filters_.size() && i < current_effort.size(); ++i)
                torque_filters_[i] = ButterworthFilter(torque_cutoff_applied_hz_, filter_rate_hz_,
                                                       static_cast<double>(current_effort[i]));
    }
    for (size_t i = 0; i < n_safe; ++i) {
        // Nor past the configured joint names — a joint with no name cannot be
        // located in the message anyway.
        if (i >= prefixed_joint_names_.size() && i >= relevant_joints.size()) break;
        const std::string& target = (i < prefixed_joint_names_.size())
            ? prefixed_joint_names_[i] : relevant_joints[i];
        auto it = std::find(msg->name.begin(), msg->name.end(), target);
        if (it == msg->name.end()) continue;
        size_t idx = static_cast<size_t>(std::distance(msg->name.begin(), it));

        if (idx >= msg->position.size()) continue;

        // Prime the filters from the first genuine sample. Their history is
        // pre-loaded with 0.0 at construction, so letting them ramp up from
        // there yields ~100 ms of fictitious state — enough to trip the
        // joint-limit hard fault on a robot whose range excludes 0°.
        if (!joint_state_received_) {
            torque_filters_[i]   = ButterworthFilter(torqueCutoffHz(), filter_rate_hz_, msg->effort[idx]);
            position_filters_[i] = ButterworthFilter(kStateCutoffHz,  filter_rate_hz_, msg->position[idx]);
            velocity_filters_[i] = ButterworthFilter(kStateCutoffHz,  filter_rate_hz_, msg->velocity[idx]);
            if (i < raw_unwrapped_.size() && i < prev_raw_.size()) {
                raw_unwrapped_[i] = msg->position[idx];
                prev_raw_[i]      = msg->position[idx];
            }
        }

        current_effort[i]     = torque_filters_[i].filter(msg->effort[idx]);
        currentVelocities_[i] = velocity_filters_[i].filter(msg->velocity[idx]);
        if (isContinuousJoint(i)) {
            // Filter the unwrapped angle, wrap only the result: the same
            // (-pi, pi] convention fic_hardware gives the controller.
            raw_unwrapped_[i] += wrapAngle(msg->position[idx] - prev_raw_[i]);
            prev_raw_[i]       = msg->position[idx];
            q_current_[i]      = wrapAngle(position_filters_[i].filter(raw_unwrapped_[i]));
        } else {
            q_current_[i]      = position_filters_[i].filter(msg->position[idx]);
        }
        // Unfiltered, for the output damping term only (see header). Kept in
        // step with the filtered estimate so the two never describe different
        // samples; the velocity field is optional in JointState, so a message
        // that omits it leaves the last known value rather than a zero, which
        // would silently switch the damper off.
        if (i < rawVelocities_.size() && idx < msg->velocity.size()) {
            rawVelocities_[i] = static_cast<T>(msg->velocity[idx]);
            // First-order lag, seeded from the first genuine sample so the
            // damper never starts from a fictitious zero.
            if (!joint_state_received_ || damping_alpha_ <= 0.0) {
                dampingVelocities_[i] = rawVelocities_[i];
            } else {
                dampingVelocities_[i] = static_cast<T>(
                    damping_alpha_ * static_cast<double>(dampingVelocities_[i]) +
                    (1.0 - damping_alpha_) * static_cast<double>(rawVelocities_[i]));
            }
        }
        ++joints_updated;

        if (solver_ok && i < relevant_joints.size()) {
            int cfg_idx = solver_->jointConfigIndex(relevant_joints[i]);
            if (cfg_idx >= 0) q[cfg_idx] = q_current_[i];
        }
    }

    // Genuine-feedback latch: only once every configured joint has been read
    // from a real joint_states message does q_current_ describe the robot.
    // Before that it is the constructor's zero vector, which must not reach
    // the impedance law or the joint-limit hard-fault check.
    if (joints_updated == num_joints_) joint_state_received_ = true;

    // Only meaningful once the filters hold real state: the rebuild seeds from
    // current_effort/currentVelocities_/q_current_.
    if (joint_state_received_) calibrateFilterRate(stamp_s);

    if (!solver_ok) return;  // degraded: filtered state is updated, no FK

    solver_->updateKinematics(q);

    Eigen::Isometry3d ee_tf;
    if (solver_->cachedFramePose(ee_frame_name_, ee_tf)) {
        ee_current = isoToPoseMsg(ee_tf);
    }

    computeAndPublishJacobian(q);
}

template <typename T>
void FicSimulationNode<T>::contactTorqueCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    contact_torques_.assign(msg->data.begin(), msg->data.end());
    contact_torques_received_ = true;
}

template <typename T>
void FicSimulationNode<T>::gravityTorqueCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (msg->data.size() >= num_joints_ && gravity_torques_.size() == (Eigen::Index)num_joints_) {
        for (size_t i = 0; i < num_joints_; ++i) {
            gravity_torques_(i) = msg->data[i];
        }
        mujoco_gravity_received_ = true;
    }
}

template <typename T>
void FicSimulationNode<T>::controlModeCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    // Cartesian modes (5-9, incl. CARTESIAN_IK_JOINT_IMPEDANCE) need FK/IK/Jacobians.
    if (msg->data >= 5 && (!solver_ || !solver_->isLoaded())) {
        RCLCPP_ERROR(this->get_logger(),
            "Control mode %d requires a kinematics solver — staying in mode %d. "
            "Set kinematics_solver in robots_config.yaml or rebuild with pinocchio.",
            msg->data, current_control_mode_);
        return;
    }
    initControllerPLanner = false;
    current_control_mode_ = msg->data;

    // Snap targets to the robot's current state on mode switch so it
    // holds position instead of jumping to a stale target.
    if (current_control_mode_ >= 1 && current_control_mode_ <= 8) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (q_current_.size() > 0) {
            q_target = q_current_;
            q_target_ik_ = q_current_;
            postural_reset_ = true;
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
void FicSimulationNode<T>::targetPositionCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        ee_target = *msg;
        applyCartesianSafetyLimits(ee_target);
    }

    // Do NOT reinit the controller on every incoming pose — at the OT
    // publish rate (100 Hz) that prevents the controller from ever settling.
    // Re-init only fires from mode/profile/robot-selection callbacks.

    if (!solver_ || !solver_->isLoaded()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Cartesian target ignored — no kinematics solver (kinematics_solver: '%s')",
            current_robot_config_.kinematics_solver.c_str());
        return;
    }

    // IK async for postural/elbow control
    geometry_msgs::msg::Pose ee_target_copy;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        ee_target_copy = ee_target;
    }
    std::thread([this, ee_target_copy]() {
        Eigen::VectorXd q_start;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            q_start = hpmpIkSeed(fic::hpmpActive(current_control_mode_, hpmp_enabled_),
                                 q_current_, q_target_ik_);
        }
        Eigen::VectorXd q_result = solveFullIterativeIK(q_start, ee_target_copy);
        applyJointSafetyLimits(q_result);

        geometry_msgs::msg::Pose calculated_elbow_pose;
        Eigen::Isometry3d elb_iso;
        if (solver_->framePose(q_result, elbow_frame_name_, elb_iso)) {
            calculated_elbow_pose = isoToPoseMsg(elb_iso);
        }
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            q_target_ik_ = q_result;
            elbow_target_pose_ = calculated_elbow_pose;
        }
    }).detach();
}

template <typename T>
void FicSimulationNode<T>::forceTorqueCallback(const geometry_msgs::msg::Wrench::SharedPtr msg) {
    current_wrench_ = *msg;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SAFETY CALLBACKS
// ═══════════════════════════════════════════════════════════════════════════════

template <typename T>
void FicSimulationNode<T>::cartesianSafetyCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
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
void FicSimulationNode<T>::jointSafetyCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 3) {
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
void FicSimulationNode<T>::velocitySafetyCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 3) {
        max_joint_velocity_ = msg->data[0];
        max_cart_velocity_ = msg->data[1];
        velocity_limits_enabled_ = msg->data[2] > 0.5;
        // optional 4th value: the joint target speed of the joint modes (rad/s, 0: off)
        if (msg->data.size() >= 4 && std::isfinite(msg->data[3])) joint_slew_.speed = std::max(0.0, msg->data[3]);

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
double FicSimulationNode<T>::clamp(double value, double min_val, double max_val) {
    return std::max(min_val, std::min(max_val, value));
}

template <typename T>
void FicSimulationNode<T>::applyCartesianSafetyLimits(geometry_msgs::msg::Pose& target_pose) {
    if (cart_safety_limits_.x_enabled)
        target_pose.position.x = clamp(target_pose.position.x, cart_safety_limits_.x_min, cart_safety_limits_.x_max);
    if (cart_safety_limits_.y_enabled)
        target_pose.position.y = clamp(target_pose.position.y, cart_safety_limits_.y_min, cart_safety_limits_.y_max);
    if (cart_safety_limits_.z_enabled)
        target_pose.position.z = clamp(target_pose.position.z, cart_safety_limits_.z_min, cart_safety_limits_.z_max);
}

template <typename T>
void FicSimulationNode<T>::applyJointSafetyLimits(Eigen::VectorXd& joint_target) {
    if (!joint_safety_enabled_ || joint_safety_lower_.empty()) return;
    for (size_t i = 0; i < std::min((size_t)joint_target.size(), joint_safety_lower_.size()); ++i) {
        joint_target[i] = clamp(joint_target[i], joint_safety_lower_[i], joint_safety_upper_[i]);
    }
}

template <typename T>
void FicSimulationNode<T>::applyVelocityLimits(Eigen::VectorXd& joint_velocities, double dt) {
    if (!velocity_limits_enabled_ || max_joint_velocity_ <= 0) return;
    for (int i = 0; i < joint_velocities.size(); ++i) {
        joint_velocities[i] = clamp(joint_velocities[i], -max_joint_velocity_, max_joint_velocity_);
    }
}

template <typename T>
bool FicSimulationNode<T>::isPositionInSafetyZone(double x, double y, double z) {
    bool safe = true;
    if (cart_safety_limits_.x_enabled && (x < cart_safety_limits_.x_min || x > cart_safety_limits_.x_max)) safe = false;
    if (cart_safety_limits_.y_enabled && (y < cart_safety_limits_.y_min || y > cart_safety_limits_.y_max)) safe = false;
    if (cart_safety_limits_.z_enabled && (z < cart_safety_limits_.z_min || z > cart_safety_limits_.z_max)) safe = false;
    return safe;
}

template <typename T>
bool FicSimulationNode<T>::eeTargetInsideZone(const Eigen::VectorXd& q) {
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
//  CONTROLLER INIT
// ═══════════════════════════════════════════════════════════════════════════════

template <typename T>
void FicSimulationNode<T>::initJointControllers(
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
void FicSimulationNode<T>::initCartesianControllers(
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
        // Orientation admittance: rot_axis admittance block applied to all
        // three rotation channels. cartesianAdmittanceControl re-clamps the
        // accumulator pose limit to a rad-scale value.
        fic_ctrl_.initCartesianAdmittance(i, cart_params.rot_axis);
    }
    fic_ctrl_.initElbowController(cart_params_);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  MAIN CONTROL LOOP
// ═══════════════════════════════════════════════════════════════════════════════

template <typename T>
bool FicSimulationNode<T>::refreshTuning(const fic::TuningKey& want, const std::string& profile) {
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
                RCLCPP_INFO(this->get_logger(), "Controllers Initialized for Mode %d (tuning read from config.xml)",
                            static_cast<int>(current_control_mode_));
                if (tuning_.key.joint_mode == fic::kHpmpTuningMode) {
                    if (tuning_.hpmp_fallback)
                        RCLCPP_WARN(this->get_logger(), "H-PMP tuning: config.xml has no H-PMP block for %s, "
                                    "so H-PMP runs on mode 4's joints and mode 6's Cartesian block", want.robot.c_str());
                    else
                        RCLCPP_INFO(this->get_logger(), "H-PMP tuning: its own block (mode 100), config \"%s\"",
                                    tuning_.profile.c_str());
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
        bool reload = false;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            reload = tuning_reload_;
            if (reload || (!current && !failed)) tuning_reload_ = false;
        }
        if (reload || (!current && !failed)) {
            tuning_failed_ = false;
            const std::string xml_file = ament_index_cpp::get_package_share_directory("fic_simulation") + "/config/config.xml";
            tuning_load_ = std::async(std::launch::async, [xml_file, want, profile] {
                return fic::loadTuningSet(xml_file, want, profile);
            });
        }
    }
    return have_tuning_ && tuning_.key == want;
}

template <typename T>
void FicSimulationNode<T>::controlLoop() {
    size_t local_num_joints;
    ControlMode local_control_mode;
    std::string local_active_profile;
    Eigen::VectorXd local_q_current, local_q_target, Compensation;
    std::vector<double> local_current_effort;
    std::vector<double> local_contact_torques;
    bool local_contact_ok = false;
    std::vector<T> local_damping_velocities;
    Eigen::MatrixXd local_jacobian, local_J_elbow_6D;
    geometry_msgs::msg::Pose local_ee_current, local_ee_target;
    geometry_msgs::msg::Pose local_elbow_current, local_elbow_target;
    geometry_msgs::msg::Wrench local_wrench;
    Eigen::VectorXd local_q_target_ik;
    bool local_postural = false;
    bool local_postural_reset = false;
    bool local_elbow = false;
    bool local_hpmp = false;
    std::vector<double> local_desired_wrench;
    bool local_solver_ok = false;

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!number_of_joints_received_ || !robot_selection_received_ || selected_robot_.empty()) return;
        // No torque before the robot's real state is known: q_current_ is still
        // the zero vector, so the impedance law would command a step away from a
        // fictitious position and the planners would seed themselves at 0 rad.
        if (!joint_state_received_) return;
        // No torque until the feedback rate — and therefore the state filters'
        // real bandwidth — is known. Measuring it takes one warm-up window
        // (~0.2 s at 200 Hz feedback); driving a stiff impedance law through
        // provisionally-tuned filters for that window is exactly the failure
        // this measurement exists to remove. Backstopped by a wall-clock
        // deadline so a stalled publisher cannot block the loop forever.
        if (!filter_rate_established_) {
            if (filter_rate_deadline_s_ > 0.0 && this->now().seconds() > filter_rate_deadline_s_) {
                filter_rate_established_ = true;
                RCLCPP_WARN(this->get_logger(),
                    "joint_states rate not established within %.1f s of the first sample; "
                    "proceeding with fallback filter rate %.1f Hz.",
                    2.0 * kFilterRateWarmupMaxSec, filter_rate_hz_);
            } else {
                return;
            }
        }
        local_solver_ok = solver_ && solver_->isLoaded();
        // Every loop below indexes the per-joint snapshots taken here, so bound
        // them by what is actually allocated rather than by num_joints_ alone.
        local_num_joints = checkedJointCount("controlLoop");
        local_control_mode = static_cast<ControlMode>(current_control_mode_);
        local_active_profile = active_profile_;
        local_q_current = q_current_;
        local_q_target = q_target;
        Compensation = gravity_torques_;
        local_contact_torques = contact_torques_;
        local_contact_ok = contact_torques_received_;
        local_current_effort = current_effort;
        local_damping_velocities = dampingVelocities_;
        local_jacobian = jacobian;
        local_J_elbow_6D = J_elbow_6D;
        local_ee_current = ee_current;
        local_ee_target = ee_target;
        local_wrench = current_wrench_;
        local_elbow_current = elbow_current_pose_;
        local_elbow_target = elbow_target_pose_;
        local_q_target_ik = q_target_ik_;
        local_postural = postural_enabled_;
        local_postural_reset = postural_reset_;
        postural_reset_ = false;
        local_elbow = elbow_enabled_;
        local_hpmp = hpmp_enabled_;
        local_desired_wrench = desired_wrench_;
    }
    fic_ctrl_.setDesiredWrench(local_desired_wrench);

    // Control-tick rate. The library integrates its planners and admittance
    // with the fixed fic::dt (0.001 s), so log the rate the loop actually
    // reaches (every 10 s) to confirm it matches.
    {
        static auto rate_t0 = std::chrono::steady_clock::now();
        static long rate_ticks = 0;
        ++rate_ticks;
        const double span = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - rate_t0).count();
        if (span >= 10.0) {
            RCLCPP_INFO(this->get_logger(),
                "Control loop: %.1f Hz over %.1f s (library dt = %.4f s -> %.1f Hz)",
                rate_ticks / span, span, fic::dt, 1.0 / fic::dt);
            rate_t0 = std::chrono::steady_clock::now();
            rate_ticks = 0;
        }
    }

    static int last_mode = -1;
    if (static_cast<int>(local_control_mode) != last_mode) {
        initControllerPLanner = false;
        last_mode = static_cast<int>(local_control_mode);
    }

    // Bounded by local_num_joints, not num_joints_: a Map is a raw view over
    // currentVelocities_' storage and reads straight off the end if it is
    // longer than the buffer. The product is only defined when the Jacobian was
    // built for the same joint count; otherwise leave the EE velocity at zero.
    Eigen::Map<Eigen::VectorXd> q_dot(currentVelocities_.data(), local_num_joints);
    Eigen::VectorXd ee_velocity = Eigen::VectorXd::Zero(6);
    if (local_jacobian.rows() == 6 && local_jacobian.cols() == (Eigen::Index)local_num_joints)
        ee_velocity = local_jacobian * q_dot;

    std::fill(loop_traj_data_.begin(), loop_traj_data_.end(), 0.0);
    std::fill(loop_torque_data_.begin(), loop_torque_data_.end(), 0.0);

    loop_current_pose_ = {
        local_ee_current.position.x, local_ee_current.position.y, local_ee_current.position.z,
        local_ee_current.orientation.x, local_ee_current.orientation.y, local_ee_current.orientation.z, local_ee_current.orientation.w
    };
    loop_target_pose_ = {
        local_ee_target.position.x, local_ee_target.position.y, local_ee_target.position.z,
        local_ee_target.orientation.x, local_ee_target.orientation.y, local_ee_target.orientation.z, local_ee_target.orientation.w
    };
    {
        Eigen::Quaterniond q_ee(
            local_ee_current.orientation.w, local_ee_current.orientation.x,
            local_ee_current.orientation.y, local_ee_current.orientation.z);
        Eigen::Matrix3d R = q_ee.normalized().toRotationMatrix();

        Eigen::Vector3d f_local(local_wrench.force.x, local_wrench.force.y, local_wrench.force.z);
        Eigen::Vector3d t_local(local_wrench.torque.x, local_wrench.torque.y, local_wrench.torque.z);
        Eigen::Vector3d f_world = R * f_local;
        Eigen::Vector3d t_world = R * t_local;

        loop_wrench_data_ = {f_world.x(), f_world.y(), f_world.z(),
                             t_world.x(), t_world.y(), t_world.z()};
    }
    // External wrench as fic_hardware computes it without an API wrench: the Jacobian estimate
    // F_ext = pinv(J^T) tau_ext from the measured joint torque less gravity (here the applied
    // effort, in the joint-admittance sign). The wrist F/T sensor above carries the gripper's
    // weight, which no gravity reference matches; it is used only with wrench_from_api.
    if (!current_robot_config_.wrench_from_api && local_solver_ok &&
        local_jacobian.rows() == 6 && (size_t)local_jacobian.cols() == local_num_joints &&
        (size_t)Compensation.size() >= local_num_joints && local_current_effort.size() >= local_num_joints) {
        Eigen::VectorXd tau_ext(local_num_joints);
        for (size_t i = 0; i < local_num_joints; ++i)
            tau_ext[i] = static_cast<double>(local_current_effort[i]) - Compensation[i];
        const Eigen::MatrixXd JJt = local_jacobian * local_jacobian.transpose();
        const Eigen::VectorXd F_ext = JJt.ldlt().solve(local_jacobian * tau_ext);
        for (int k = 0; k < 6; ++k) loop_wrench_data_[k] = F_ext[k];
    }
    loop_elbow_current_ = {
        local_elbow_current.position.x, local_elbow_current.position.y, local_elbow_current.position.z,
        local_elbow_current.orientation.x, local_elbow_current.orientation.y, local_elbow_current.orientation.z, local_elbow_current.orientation.w
    };
    loop_elbow_target_ = {
        local_elbow_target.position.x, local_elbow_target.position.y, local_elbow_target.position.z,
        local_elbow_target.orientation.x, local_elbow_target.orientation.y, local_elbow_target.orientation.z, local_elbow_target.orientation.w
    };

    const int mode_int = static_cast<int>(local_control_mode);
    const bool hpmp_active = fic::hpmpActive(mode_int, local_hpmp);
    // The blocks fic_hardware reads for the same mode (fic::tuningKeyFor): H-PMP its own.
    const fic::TuningKey tuning_want = fic::tuningKeyFor(selected_robot_, local_num_joints, mode_int, local_hpmp);
    // Nothing to run until the tuning for this robot and mode is in memory.
    if (!refreshTuning(tuning_want, local_active_profile)) return;

    if (!initControllerPLanner) {
        try {
            if (hpmp_active) {
                initJointControllers(local_num_joints, local_q_current, joint_scalars_, joint_lists_, joint_planners_, joint_admittance_);
                std::vector<double> ee_curr_vec = {
                    local_ee_current.position.x, local_ee_current.position.y, local_ee_current.position.z,
                    local_ee_current.orientation.x, local_ee_current.orientation.y, local_ee_current.orientation.z, local_ee_current.orientation.w
                };
                fic_ctrl_.initHpmpCartesian(cart_params_, ee_curr_vec);
            } else if (mode_int <= 4 || mode_int == 9) {
                initJointControllers(local_num_joints, local_q_current, joint_scalars_, joint_lists_, joint_planners_, joint_admittance_);
            } else {
                initCartesianControllers(loop_current_pose_, cart_params_);
            }
            if (mode_int >= 5 && mode_int <= 8) {
                // Null-space postural task (channels 9..9+n, clear of the Cartesian 0..5,
                // elbow 6..8 / 20..22, H-PMP 0..n-1 and 16..19 channels). Without this the
                // sim never initialised them, so postural.apply() ran on default profiles.
                fic_ctrl_.postural.init(local_num_joints, postural_scalars_, postural_lists_);
            }
            initControllerPLanner = true;
            RCLCPP_DEBUG(this->get_logger(), "Controllers Initialized for Mode %d%s", mode_int, hpmp_active ? " (hpmp)" : "");
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Init Failed: %s", e.what());
            return;
        }
    }

    int CartAdmMode = 0;
    if (static_cast<int>(local_control_mode) <= 4) {
        // modes 1 and 2: the joint target the law sees, at most joint_slew_.speed rad/s from the last (off:
        // the commanded one); the joint admittance modes (3, 4) update their own target
        if (static_cast<int>(local_control_mode) >= 1 && static_cast<int>(local_control_mode) <= 2)
            local_q_target = joint_slew_.apply(local_q_target, local_q_current, static_cast<int>(local_control_mode),
                                               joint_continuous_, fic::dt);
        for (size_t i = 0; i < local_num_joints; ++i) {
            switch (local_control_mode) {
                case ControlMode::JOINT_IMPEDANCE:
                    fic_ctrl_.JC.impedance(i, loop_traj_data_, loop_torque_data_, local_q_target[i], local_q_current[i], local_num_joints);
                    // Gravity compensation: the FIC joint-impedance law has no g(q)
                    // term, so without this the impedance must produce a steady-state
                    // error to balance gravity (the shoulder/elbow "sag"). Add the
                    // model gravity torque g(q) — same convention the Cartesian modes
                    // use internally (tau = J^T w + gravity_torques_). This is gravity
                    // compensation, NOT a controller damping term.
                    loop_torque_data_[i] += Compensation[i];
                    break;
                case ControlMode::JOINT_IMPEDANCE_PLANNER:
                    fic_ctrl_.JC.impedanceWithPlanner(i, loop_traj_data_, loop_torque_data_, local_q_target[i], local_q_current[i], local_num_joints);
                    loop_torque_data_[i] += Compensation[i];
                    break;
                case ControlMode::JOINT_ADMITTANCE_M1:
                case ControlMode::JOINT_ADMITTANCE_M2: {
                    int m = (local_control_mode == ControlMode::JOINT_ADMITTANCE_M1) ? 1 : 2;
                    fic_ctrl_.JC.admittance(i, loop_traj_data_, loop_torque_data_, local_q_target[i], local_q_current[i], Compensation[i], local_current_effort[i], m, local_num_joints);
                    // Gravity fed forward as in modes 1 and 2: the admittance is driven by the
                    // interaction torque alone, so the joint law must not carry the weight.
                    loop_torque_data_[i] += Compensation[i];
                    break;
                }
                default: break;
            }
        }

        // Cartesian virtual wall for joint modes: when the EE is past an
        // enabled Cartesian safety boundary, add a restoring force via J^T
        // so the robot is bounded at the limit instead of faulting.
        // Requires a kinematics solver — without one ee_current/jacobian are
        // never updated, so wall forces would be computed from stale state.
        if (local_solver_ok &&
            (cart_safety_limits_.x_enabled || cart_safety_limits_.y_enabled
             || cart_safety_limits_.z_enabled)) {
            Eigen::Vector3d f_wall = Eigen::Vector3d::Zero();
            constexpr double kWall = 2000.0;  // N/m
            const double ex = local_ee_current.position.x;
            const double ey = local_ee_current.position.y;
            const double ez = local_ee_current.position.z;
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
                Eigen::VectorXd tau_wall = local_jacobian.topRows(3).transpose() * f_wall;
                for (size_t i = 0; i < local_num_joints; ++i)
                    loop_torque_data_[i] += tau_wall[i];
            }
        }
    } else if (local_control_mode == ControlMode::CARTESIAN_IK_JOINT_IMPEDANCE) {
        if (local_q_target_ik.size() == local_q_current.size()) {
            for (size_t i = 0; i < local_num_joints; ++i) {
                fic_ctrl_.JC.impedance(i, loop_traj_data_, loop_torque_data_, local_q_target_ik[i], local_q_current[i], local_num_joints);
                // Gravity compensation (see JOINT_IMPEDANCE above) — mode 9 drives
                // the joint-space impedance from IK targets and likewise lacks g(q).
                loop_torque_data_[i] += Compensation[i];
            }
        }
    } else {
        std::vector<double> neg_torques(local_num_joints);
        for (size_t i = 0; i < local_num_joints; ++i) {
            neg_torques[i] = -local_current_effort[i];
        }

        if (local_control_mode == ControlMode::CARTESIAN_IMPEDANCE) {
            if (local_hpmp) {
                fic_ctrl_.RC.hpmp(loop_traj_data_, loop_torque_data_, local_q_current, Compensation,
                    loop_current_pose_, loop_target_pose_, local_jacobian,
                    fic::ficControl<double>::kHpmpHostImpedance,
                    loop_wrench_data_, neg_torques, q_target_ik_, local_num_joints);
            } else {
                fic_ctrl_.RC.impedance(loop_traj_data_, loop_torque_data_, local_q_current, Compensation, loop_current_pose_, loop_target_pose_, local_jacobian, 0, loop_wrench_data_, neg_torques, q_target_ik_, local_num_joints);
            }
        }
        else if (local_control_mode == ControlMode::CARTESIAN_PLANNER) {
            if (local_hpmp) {
                fic_ctrl_.RC.hpmp(loop_traj_data_, loop_torque_data_, local_q_current, Compensation,
                    loop_current_pose_, loop_target_pose_, local_jacobian, 0,
                    loop_wrench_data_, neg_torques, q_target_ik_, local_num_joints);
            } else {
                fic_ctrl_.RC.planner(loop_traj_data_, loop_torque_data_, local_q_current, Compensation, loop_current_pose_, loop_target_pose_, local_jacobian, 0, loop_wrench_data_, neg_torques, q_target_ik_, local_num_joints);
            }
        } else if (static_cast<int>(local_control_mode) >= 7) {
            CartAdmMode = (local_control_mode == ControlMode::CARTESIAN_ADMITTANCE_M1) ? 1 : 2;
            if (local_hpmp) {
                // H-PMP's task-space admittance: external wrench estimated from the joint torques
                // the environment applies (MuJoCo constraint and applied forces), pinv(J^T)(-tau_c),
                // in the sign of the estimate above (at rest in contact tau_act - g = -tau_c). The
                // applied effort of joint_states is the node's own command, whose estimate makes the
                // admittance yield to the arm's own tracking effort. Without the topic: as above.
                std::vector<double> hpmp_wrench = loop_wrench_data_;
                // With the robot's hardware_model: the estimate above, as on the robot.
                if (!current_robot_config_.hardware_model.enabled && local_contact_ok && local_solver_ok && local_jacobian.rows() == 6 &&
                    (size_t)local_jacobian.cols() == local_num_joints &&
                    local_contact_torques.size() >= local_num_joints) {
                    Eigen::VectorXd tau_c(local_num_joints);
                    for (size_t i = 0; i < local_num_joints; ++i) tau_c[i] = -local_contact_torques[i];
                    const Eigen::MatrixXd JJt = local_jacobian * local_jacobian.transpose();
                    const Eigen::VectorXd F_c = JJt.ldlt().solve(local_jacobian * tau_c);
                    for (int k = 0; k < 6; ++k) hpmp_wrench[k] = F_c[k];
                }
                fic_ctrl_.RC.hpmp(loop_traj_data_, loop_torque_data_, local_q_current, Compensation,
                    loop_current_pose_, loop_target_pose_, local_jacobian, CartAdmMode,
                    hpmp_wrench, neg_torques, q_target_ik_, local_num_joints);
            } else {
                fic_ctrl_.RC.admittance(loop_traj_data_, loop_torque_data_, local_q_current, Compensation, loop_current_pose_, loop_target_pose_, local_jacobian, CartAdmMode, loop_wrench_data_, neg_torques, q_target_ik_, ee_velocity, local_num_joints,
                    current_robot_config_.wrench_gravity_compensated);
            }
        }

        // H-PMP's joint loop sends no gravity of its own (its admittance is driven by the
        // interaction torque alone): feed it forward, as in modes 1-4.
        if (hpmp_active) {
            for (size_t i = 0; i < local_num_joints; ++i) loop_torque_data_[i] += Compensation[i];
        }

        // Null-space helpers — optional additions on top of the Cartesian torques.
        if (local_postural && local_q_target_ik.size() == local_q_current.size()) {
            // Toward the IK solution at no more than kPosturalMaxRate per
            // joint (see the header), continuous joints the short way.
            const Eigen::Index n = local_q_target_ik.size();
            const auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(now - postural_at_).count();
            postural_at_ = now;
            // Step by the time actually passed. The loop itself ticks at its
            // 1 ms timer (~1 kHz, see the rate log above); the joint states
            // arrive at ~160 Hz, and a stalled timer must not jump the target.
            if (!(dt > 0.0)) dt = 0.001;
            dt = std::min(dt, 0.05);   // a pause: no more than 50 ms' worth
            if (local_postural_reset || q_postural_.size() != n)
                q_postural_ = local_q_current;
            const double max_step = kPosturalMaxRate * dt;
            for (Eigen::Index i = 0; i < n; ++i) {
                double d = local_q_target_ik[i] - q_postural_[i];
                if (isContinuousJoint(static_cast<size_t>(i))) d = wrapAngle(d);
                q_postural_[i] += std::clamp(d, -max_step, max_step);
                if (isContinuousJoint(static_cast<size_t>(i)))
                    q_postural_[i] = wrapAngle(q_postural_[i]);
            }
            fic_ctrl_.postural.apply(local_q_current, q_postural_, CartAdmMode, local_num_joints, loop_torque_data_);
        }
        if (local_elbow && local_J_elbow_6D.cols() == (int)local_num_joints) {
            fic_ctrl_.applyElbowControl(loop_torque_data_, local_q_current, local_jacobian, local_J_elbow_6D,
                                        loop_elbow_current_, loop_elbow_target_, local_num_joints,
                                        fic_ctrl_.elbowOffset());
        }
    }

    // Hard-fault check: if a joint exceeds the robot config's physical limits,
    // latch a fault and output only gravity comp until a new robot is loaded.
    // Gated on joint_state_received_ so it is only ever evaluated against real
    // feedback — the zero-initialised q_current_ sits outside the configured
    // range of any robot whose limits exclude 0° (Spot's knees, Franka's J3).
    if (!hard_fault_ && joint_state_received_ &&
        !current_robot_config_.joint_limits_lower_deg.empty() &&
        !current_robot_config_.joint_limits_upper_deg.empty()) {
        for (size_t i = 0; i < local_num_joints; ++i) {
            if (i >= current_robot_config_.joint_limits_lower_deg.size()) break;
            double pos_deg = local_q_current[i] * 180.0 / M_PI;
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
    if (hard_fault_) {
        for (size_t i = 0; i < local_num_joints; ++i)
            loop_torque_data_[i] = Compensation[i];
    }

    // Safety: defensive output guard — replace non-finite torques with 0 and
    // clamp per-joint (config limits) or to a safe ceiling before publishing.
    for (size_t i = 0; i < loop_torque_data_.size(); ++i) {
        double tau = loop_torque_data_[i];
        if (!std::isfinite(tau)) tau = 0.0;
        double limit = (i < per_joint_torque_limit_.size())
                       ? per_joint_torque_limit_[i] : output_torque_limit_;
        loop_torque_data_[i] = clamp(tau, -limit, limit);
    }

    // Publish
    std_msgs::msg::Float64MultiArray traj_msg, torq_msg;
    traj_msg.data = loop_traj_data_;
    torq_msg.data = loop_torque_data_;
    trajectory_publisher_->publish(traj_msg);
    torque_command_publisher_->publish(torq_msg);
    if (ee_pose_publisher_) ee_pose_publisher_->publish(ee_current);

    // Human push offset: published ONLY while joint admittance is active
    // (modes 3/4). Never publish zeros in any other mode — a consumer must be
    // able to tell "no admittance running" from "nobody is pushing" (a real,
    // meaningful zero offset).
    if (local_control_mode == ControlMode::JOINT_ADMITTANCE_M1 ||
        local_control_mode == ControlMode::JOINT_ADMITTANCE_M2) {
        std_msgs::msg::Float64MultiArray offset_msg;
        offset_msg.data.resize(local_num_joints);
        for (size_t i = 0; i < local_num_joints; ++i) {
            offset_msg.data[i] = fic_ctrl_.humanOffset(i);
        }
        admittance_offset_publisher_->publish(offset_msg);
    }

    communication::msg::ActuatorCmds actuator_cmds;
    actuator_cmds.header.stamp = this->get_clock()->now();
    std_msgs::msg::Float64MultiArray actuator_torques;
    actuator_torques.data.resize(local_num_joints);
    for (size_t i = 0; i < local_num_joints; ++i) {
        double final_torque = loop_torque_data_[i];
        if (velocity_limits_enabled_ && max_joint_velocity_ > 0
            && i < local_damping_velocities.size()) {
            double kd = (i < per_joint_torque_limit_.size())
                        ? per_joint_torque_limit_[i] / max_joint_velocity_
                        : output_torque_limit_ / max_joint_velocity_;
            // The damping-path velocity, NOT the 10 Hz 2nd-order-filtered
            // currentVelocities_: a damper fed a signal that lags by more than
            // 90 deg at the plant's resonance injects energy instead of
            // removing it, and a 2nd-order filter reaches 159 deg at Spot's
            // knee resonance. See kDampingCutoffHz in the header.
            final_torque -= kd * static_cast<double>(local_damping_velocities[i]);
        }
        if (!std::isfinite(final_torque)) final_torque = 0.0;
        double limit = (i < per_joint_torque_limit_.size())
                       ? per_joint_torque_limit_[i] : output_torque_limit_;
        final_torque = clamp(final_torque, -limit, limit);
        actuator_cmds.actuators_name.push_back(joint_name_prefix_ + "torq_j" + std::to_string(i + 1));
        actuator_cmds.torque.push_back(static_cast<float>(final_torque));
        actuator_torques.data[i] = final_torque;
    }
    control_publisher_->publish(actuator_cmds);
    actuator_torque_publisher_->publish(actuator_torques);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  EXPLICIT TEMPLATE INSTANTIATION
// ═══════════════════════════════════════════════════════════════════════════════

template class FicSimulationNode<double>;

// ═══════════════════════════════════════════════════════════════════════════════
//  MAIN
// ═══════════════════════════════════════════════════════════════════════════════

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    size_t num_joints = 7;
    rclcpp::spin(std::make_shared<FicSimulationNode<double>>(num_joints));
    rclcpp::shutdown();
    return 0;
}