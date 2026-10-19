#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <chrono> // Include for time measurement
#include <thread> // Include for multithreading
#include <unistd.h>
#include <time.h>
#include <functional>
#include <future>
#include <filesystem>
#include <csignal>

// Set by SIGTERM/SIGINT handler so the torque loop exits gracefully
// and switches back to position mode before the process dies.
static volatile sig_atomic_t g_stop_requested = 0;

static void stop_signal_handler(int /*signum*/) {
    g_stop_requested = 1;
}

#include <KDetailedException.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <InterconnectCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientUdp.h>
#include <TransportClientTcp.h>

#include "utilities.h"
#include <tinyxml2.h>
#include <google/protobuf/util/json_util.h>

#include <Eigen/Dense>


#include <Eigen/Geometry> 

#include <fic/fic_control.h>
#include <fic/shared_params.h>

// Kinova Gen3 (7DOF) — this driver is physically 7-DOF. kNumJoints is named
// here (instead of hardcoding 7 inline) so every shared-memory loop is
// obviously tied to the driver's DOF, and the static_assert catches an
// accidental kMaxJoints regression below 7.
static constexpr size_t kNumJoints = 7;
static_assert(kNumJoints <= kMaxJoints,
              "kNumJoints must fit inside the shared-memory reservation (kMaxJoints).");
// The measured gripper state rides in joint slot kGripperFeedbackSlot (see
// shared_params.h), which is only free while the arm has fewer joints.
static_assert(kNumJoints < kGripperFeedbackSlot,
              "The gripper feedback slot would overlap a real joint.");
#if defined(_MSC_VER)
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <iomanip>  // std::fixed, std::setprecision
#include <cmath>    // std::isfinite

namespace k_api = Kinova::Api;
constexpr auto TIMEOUT_PROMISE_DURATION = std::chrono::seconds{20};
#define PORT 10000
#define PORT_REAL_TIME 10001

using k_api::ActuatorConfig::LoopSelection;
using k_api::ActuatorConfig::ControlLoopSelection;
using k_api::ActuatorConfig::ControlLoopParameters;

// ─────────────────────────────────────────────────────────────────────────────
//  Gripper background thread
//
//  Reads commanded_gripper_width (metres) from shared memory and drives the
//  gripper via Base::SendGripperCommand at 20 Hz.
//
//  Position: 0 = fully open, 1 = fully closed (normalised).
//  FIC width: 0 m = closed, kGripperMaxWidth = open.
//  So:  position = 1 - (width_m / kGripperMaxWidth)
// ─────────────────────────────────────────────────────────────────────────────
// Helper: read gripper width from shared memory and return normalised position [0,1]
// 0 = fully open, 1 = fully closed.
static constexpr double kGripperMaxWidth = 0.085;

float get_gripper_position_from_shm()
{
    if (!shared_feedback) return 0.0f;
    float raw = shared_feedback->commanded_gripper_width.load(std::memory_order_acquire);
    double width = std::clamp(static_cast<double>(raw), 0.0, kGripperMaxWidth);
    return 1.0f - static_cast<float>(width / kGripperMaxWidth);
}

// The inverse direction: publish the gripper's MEASURED state into the shared
// gripper feedback slot (shared_params.h, kGripperFeedbackSlot) — the opening
// in metres (0 = closed, kGripperMaxWidth = open) and the motor current as
// Kortex reports it. Call only after a successful cyclic Refresh() on a robot
// with a gripper. Runs in the 1 kHz loop: no allocation, no printing.
static void store_gripper_feedback_to_shm(const k_api::BaseCyclic::Feedback& fb)
{
    if (!shared_feedback) return;
    const auto& gripper = fb.interconnect().gripper_feedback();
    if (gripper.motor_size() < 1) return;
    const auto& motor = gripper.motor(0);
    const float pos_pct = motor.position();   // percent closed, 0 = open
    const float current = motor.current_motor();
    if (!std::isfinite(pos_pct) || !std::isfinite(current)) return;
    const double closed = std::clamp(static_cast<double>(pos_pct), 0.0, 100.0) / 100.0;
    const float width = static_cast<float>(kGripperMaxWidth * (1.0 - closed));
    shared_feedback->joint_positions[kGripperFeedbackSlot].store(width, std::memory_order_release);
    shared_feedback->torque[kGripperFeedbackSlot].store(current, std::memory_order_release);
}

// ---- Fault handling -------------------------------------------------------
// Say WHY the arm faulted, not just that it did.
//
// Until this existed a fault produced one line, "Arm entered FAULT during
// control", and nothing else: no joint, no cause. The 25 September 2026
// double-arm fault was still unexplained days later for exactly that reason.
//
// The cause is already in hand. Every cyclic Refresh carries fault and warning
// bitfields on the base and on each actuator (BaseCyclic.pb.h: BaseFeedback
// and ActuatorFeedback, fault_bank_a/b and warning_bank_a/b). They cost no
// extra RPC and were simply being thrown away.
//
// The warning banks are printed too, and usually matter more: a Gen3 generally
// warns before it trips, so a warning standing on one joint is the thread to
// pull. The bits are printed in hex rather than decoded because the meaning of
// each is a firmware-version detail, and a wrong decode would be worse than
// none: hex can be handed to Kinova's documentation or to their support as is.
static void dumpFaultBanks(const k_api::BaseCyclic::Feedback& fb)
{
    const auto& base = fb.base();
    std::cout << "[FAULT] base fault_bank a=0x" << std::hex << base.fault_bank_a()
              << " b=0x" << base.fault_bank_b()
              << "  warning_bank a=0x" << base.warning_bank_a()
              << " b=0x" << base.warning_bank_b() << std::dec << std::endl;
    for (int i = 0; i < fb.actuators_size(); ++i) {
        const auto& a = fb.actuators(i);
        if (a.fault_bank_a() == 0 && a.fault_bank_b() == 0 &&
            a.warning_bank_a() == 0 && a.warning_bank_b() == 0)
            continue;                         // quiet joints are not the story
        std::cout << "[FAULT]   joint " << i
                  << " fault a=0x" << std::hex << a.fault_bank_a()
                  << " b=0x" << a.fault_bank_b()
                  << " warning a=0x" << a.warning_bank_a()
                  << " b=0x" << a.warning_bank_b() << std::dec
                  << "  pos=" << a.position() << " deg"
                  << " vel=" << a.velocity() << " deg/s"
                  << " torque=" << a.torque() << " Nm"
                  << " current=" << a.current_motor() << " A"
                  << " temp=" << a.temperature_motor() << " C"
                  << std::endl;
    }
    std::cout << "[FAULT] (bits are hex; look them up in the Kortex "
                 "documentation for this firmware)" << std::endl;
}

// Query the Kinova arm state. On RPC failure, logs and returns
// ARMSTATE_UNSPECIFIED so callers treat the state as "unknown".
static k_api::Common::ArmState query_arm_state(k_api::Base::BaseClient* base,
                                               std::string& state_name)
{
    try {
        k_api::Base::ArmStateInformation info = base->GetArmState();
        k_api::Common::ArmState st = info.active_state();
        state_name = k_api::Common::ArmState_Name(st);
        return st;
    } catch (...) {
        state_name = "UNKNOWN (GetArmState RPC failed)";
        return k_api::Common::ARMSTATE_UNSPECIFIED;
    }
}

// True if the arm is currently in a fault state.
static bool arm_in_fault(k_api::Base::BaseClient* base, std::string& state_name)
{
    return query_arm_state(base, state_name) == k_api::Common::ARMSTATE_IN_FAULT;
}

// Clear faults and confirm the arm leaves the fault state (polls up to ~2 s).
// Returns true iff the arm is NOT in fault afterwards. ONLY safe at startup
// (arm idle, before torque mode); clearing during active control jerks the arm.
static bool try_clear_faults(k_api::Base::BaseClient* base)
{
    try {
        base->ClearFaults();
    } catch (...) {
        std::cout << "[FAULT] ClearFaults() RPC failed" << std::endl;
        return false;
    }
    std::string name;
    for (int attempt = 0; attempt < 20; ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (!arm_in_fault(base, name)) return true;
    }
    return false;
}

template <typename T>
class FICNode{ 
	public:
    explicit FICNode(T N_s, bool asymmetric=false, T N_negSat=T{})
    : fic_ctrl_(N_s, asymmetric, N_negSat) {}

	int TIME_DURATION = 60000;  // Use integer instead of floatint timer_count = 0;

	Eigen::Vector3d current_position = Eigen::Vector3d::Zero();
	Eigen::Vector3d current_rotation = Eigen::Vector3d::Zero();
	Eigen::VectorXd invM = Eigen::VectorXd::Ones(7);  // Example mass for each joint

	Eigen::VectorXd required_torques = Eigen::VectorXd::Zero(7);

    Eigen::VectorXd current_positions = Eigen::VectorXd::Zero(7); 
    Eigen::VectorXd previous_positions = Eigen::VectorXd::Zero(7); 


	int controlmode;  // Variable to store the current control mode
	const double DT = 0.001;
	int Stop = 0;
	//std::vector<double> desiredPositions_(7, 0.0);

	std::vector<double> target_pos;
	std::vector<double> current_pos;

	int num_joints_ = 7;
	std::vector<double> traj_data;
	std::vector<double> torque_data;
	std::vector<double> current_wrench_;

	Eigen::Vector3d planned_pos;
	std::string selected_robot_ = "KinovaGen3_7DOF"; // Change to sub Data
	
	Eigen::Vector3d velocity_;
	Eigen::Vector3d pose_;
	Eigen::Vector3d desired_position =Eigen::Vector3d (0.5, 0.5, 0.5);  // Example target position
	


    T stiffness, velDesired, accMax, dampingRatio;
    Eigen::MatrixXd jacobian;
    Eigen::VectorXd q_current_, q_target, dq_current_, ddq_desired_, gravity_torques_;

    // Min/max limits for Cartesian Admittance

    double desired_effort ; // Replace with actual scalar values if necessary
    std::vector<T> current_effort = std::vector<T>(7, static_cast<T>(0.0));  // Set 7 values to 0

    Eigen::VectorXd joint_torques;
    
    Eigen::Vector2d minMax_dFlow{-1.0, 1.0};  // Example values
    Eigen::Vector2d minMax_Flow{-2.0, 2.0};   // Example values
    Eigen::Vector2d minMax_Pose{10.0, 10.0}; // Example values
    
    std::vector<T> initialPositions;

    fic::ficControl<T> fic_ctrl_;

    Eigen::MatrixXd jacobian_transpose;
    // std::vector<std::shared_ptr<AdmittanceController<T>>> admittance_controllers_;
    std::vector<T> currentPositions_, targetPositions_, currentVelocities_, targetVelocities_, previousPositions_, desiredPositions_, currentEfforts_, admittanceDesiredPosition;
    T plannerTargetPos;
    T adm_delta;
    T plannerAdmPos;
    T dt = static_cast<T>(0.001);


    T diff_effort_max , vel_desired , acc_max , damping_ratio , frequency ;
    std::vector<T> error_max, force_max, saturation_ratios;

/*****************************
 * Example related function *
 *****************************/
int64_t GetTickUs()
{
#if defined(_MSC_VER)
    LARGE_INTEGER start, frequency;

    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&start);

    return (start.QuadPart * 1000000) / frequency.QuadPart;
#else
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
#endif
}

// Rate-limit a per-cycle std::cout so it prints at most ~once per second.
// At 1 kHz a faulting arm would otherwise flood the studio log and can
// hang/crash the GUI. Each call site gets its own static gate keyed on the
// monotonic GetTickUs() clock (1 s = 1000000 us).
#define THROTTLED_COUT(stream_expr)                                            \
    do {                                                                       \
        static int64_t _throttle_last_us = 0;                                  \
        int64_t _throttle_now_us = GetTickUs();                                \
        if (_throttle_now_us - _throttle_last_us > 1000000) {                  \
            _throttle_last_us = _throttle_now_us;                              \
            std::cout << stream_expr << std::endl;                             \
        }                                                                      \
    } while (0)

// Create an event listener that will set the promise action event to the exit value
// Will set promise to either END or ABORT
// Use finish_promise.get_future.get() to wait and get the value
std::function<void(k_api::Base::ActionNotification)>
    create_event_listener_by_promise(std::promise<k_api::Base::ActionEvent>& finish_promise)
{
    return [&finish_promise] (k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
        {
        case k_api::Base::ActionEvent::ACTION_END:
        case k_api::Base::ActionEvent::ACTION_ABORT:
            finish_promise.set_value(action_event);
            break;
        default:
            break;
        }
    };
}



void updateSharedFeedback(
    const std::vector<double> &current_torques,
    const std::vector<double> &current_joint_positions)
{
    if (!shared_feedback) return;

    // Joint torques
    for (size_t i = 0; i < std::min(current_torques.size(), kNumJoints); ++i) {
        shared_feedback->torque[i].store(static_cast<float>(current_torques[i]), std::memory_order_release);
    }

    // Joint positions
    for (size_t i = 0; i < std::min(current_joint_positions.size(), kNumJoints); ++i) {
        shared_feedback->joint_positions[i].store(static_cast<float>(current_joint_positions[i]), std::memory_order_release);
    }
}




/**************************
 * Example core functions *
 **************************/
bool example_move_to_home_position(k_api::Base::BaseClient* base)
{
    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    std::cout << "Moving the arm to a safe position" << std::endl;
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = k_api::Base::ActionHandle();
    action_handle.set_identifier(0);
    for (auto action : action_list.action_list())
    {
        if (action.name() == "Home")
        {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0)
    {
        std::cout << "Can't reach safe position, exiting" << std::endl;
        return false;
    }
    else
    {
        // Connect to notification action topic
        std::promise<k_api::Base::ActionEvent> finish_promise;
        auto finish_future = finish_promise.get_future();
        auto promise_notification_handle = base->OnNotificationActionTopic(
            create_event_listener_by_promise(finish_promise),
            k_api::Common::NotificationOptions()
        );

        // Execute action
        base->ExecuteActionFromReference(action_handle);

        // Wait for future value from promise
        const auto status = finish_future.wait_for(TIMEOUT_PROMISE_DURATION);
        base->Unsubscribe(promise_notification_handle);

        if(status != std::future_status::ready)
        {
            std::cout << "Timeout on action notification wait" << std::endl;
            return false;
        }
        const auto promise_event = finish_future.get();

        std::cout << "Move to Home completed" << std::endl;
        std::cout << "Promise value : " << k_api::Base::ActionEvent_Name(promise_event) << std::endl;

        // Try to open gripper while still in single-level servoing
        try {
            k_api::Base::GripperCommand gripper_command;
            gripper_command.set_mode(k_api::Base::GRIPPER_POSITION);
            auto finger = gripper_command.mutable_gripper()->add_finger();
            finger->set_finger_identifier(1);
            finger->set_value(get_gripper_position_from_shm());
            base->SendGripperCommand(gripper_command);
            std::cout << "[Gripper] Initialized to shared memory position" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        } catch (const std::exception& ex) {
            std::cerr << "[Gripper] No gripper detected, continuing without it: " << ex.what() << std::endl;
        }

        }
        return true;
}



inline double wrapToPi(double x) {
            return std::atan2(std::sin(x), std::cos(x));
        }

bool example_cyclic_torque_control(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic, k_api::ActuatorConfig::ActuatorConfigClient* actuator_config)
{
    bool return_status = true;

    // Get actuator count
    unsigned int actuator_count = base->GetActuatorCount().count();
    
    fic::ficControl<T> fic_ctrl_{ static_cast<T>(4), false, static_cast<T>(0), 7, true };


    
    // --- Fault check BEFORE entering torque mode ---
    // Entering LOW_LEVEL/torque servoing on a faulted arm is unsafe. Report the
    // arm state, attempt to clear any fault, and refuse to start torque mode if
    // the fault persists — the operator must clear it before the next run.
    {
        std::string state_name;
        if (arm_in_fault(base, state_name)) {
            std::cout << "[FAULT] Robot is in FAULT (" << state_name
                      << ") — refusing to enter torque mode and closing. Clear the "
                         "fault manually and restart." << std::endl;
            return false;
        }
    }


    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command  base_command;

    std::vector<float> commands;

    auto servoing_mode = k_api::Base::ServoingModeInformation();
    int timer_count = 0;

    int64_t now = 0;
    int64_t last = 0;
 	traj_data.resize(num_joints_ * 5, 0.0);
    torque_data.resize(num_joints_, 0.0);
    current_wrench_.resize(3, 1.0); // Creates a vector of size 3 with all elements set to 0
    gravity_torques_ = Eigen::VectorXd::Ones(num_joints_);
    std::cout << "Initializing the arm for torque control example" << std::endl;



    // Create and configure LoopSelection message
    LoopSelection loop_selection;
    loop_selection.set_loop_selection(ControlLoopSelection::JOINT_TORQUE);  // Or JOINT_POSITION, etc.

    // Now call GetControlLoopParameters with the proper device ID
    uint32_t actuator_device_id = 1;  // Assuming actuator 1
    auto control_params = actuator_config->GetControlLoopParameters(loop_selection, actuator_device_id);


    // ── TEARDOWN WATCHDOG ──
    // Declared before try so it's visible in the catch and teardown below.
    // Every Kortex RPC in the stop path (GetArmState, ClearFaults,
    // SetControlMode ×7, SetServoingMode) is a blocking UDP call.  If any
    // one hangs, the whole process blocks; the GUI SIGKILLs it, which can
    // leave the kernel USB/network stack in a bad state and freeze the
    // machine.  The watchdog calls _exit() after 3 s so the process
    // force-quits cleanly.  It is armed at exit-time (after the torque
    // loop), not here — but must be declared at this scope.
    std::atomic<bool> teardown_done{false};

    try
    {
        // ── Wait for fic_control to be commanding BEFORE handing over the arm ──
        //
        // In TORQUE mode the actuators apply only what they are told. fic_control
        // cannot tell them anything until it has seen joint feedback, and the
        // feedback it reads is what this process publishes. Entering torque mode
        // first therefore opened a window — about a second, longer when the node was
        // slow to start — in which commanded_torque was still unset, the read below
        // substituted 0.0, and the arm was held by nothing. It visibly fell.
        // shared_params.h:190 stores NaN into commanded_torque for exactly this
        // reason: "no command yet" is meant to be distinguishable from "command 0".
        //
        // So publish feedback from SINGLE_LEVEL servoing, where the arm holds itself,
        // until every active joint carries a finite command. Only then go low-level.
        // The first cyclic tick then has a real torque to send. Nothing here runs
        // inside the 1 kHz loop, so the servoing watchdog is never at risk.
        if (shared_feedback)
        {
            bool announced = false;
            while (!g_stop_requested &&
                   !shared_feedback->stop_torque_mode.load(std::memory_order_acquire))
            {
                // Same units and conventions as the main loop, or fic_control
                // reads nonsense: joint angles in RADIANS wrapped to +-pi, from
                // GetMeasuredJointAngles (which reports DEGREES, 0..360), and
                // torque raw from the cyclic feedback. Passing the raw degrees
                // here made fic_control read 15 deg as 15 rad = 858 deg and hard
                // fault, which drops it to its gravity-only output: the arm then
                // holds against gravity but with no impedance at all.
                base_feedback = base_cyclic->RefreshFeedback();
                k_api::Base::JointAngles wait_angles = base->GetMeasuredJointAngles();
                std::vector<double> q(actuator_count, 0.0), tau(actuator_count, 0.0);
                for (unsigned int i = 0; i < actuator_count; i++) {
                    if (static_cast<int>(i) < wait_angles.joint_angles_size())
                        q[i] = wrapToPi(wait_angles.joint_angles(i).value() * M_PI / 180.0);
                    tau[i] = base_feedback.actuators(i).torque();
                }
                updateSharedFeedback(tau, q);

                bool ready = true;
                for (unsigned int i = 0; i < actuator_count && i < kNumJoints; ++i) {
                    if (!std::isfinite(shared_feedback->commanded_torque[i]
                                       .load(std::memory_order_acquire))) { ready = false; break; }
                }
                if (ready) {
                    std::cout << "fic_control is commanding - entering torque control"
                              << std::endl;
                    break;
                }
                if (!announced) {
                    announced = true;
                    std::cout << "Holding position: waiting for fic_control to command "
                                 "(the arm stays held until it does)" << std::endl;
                }
                // No timeout on purpose: holding the arm indefinitely is correct, and
                // safer than torque mode with nothing driving it. Ctrl-C or the GUI
                // stop still exits through g_stop_requested.
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }

        // Set the base in low-level servoing mode
        servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoing_mode);

        base_feedback = base_cyclic->RefreshFeedback();

        // Initialize each actuator to their current position
        for (unsigned int i = 0; i < actuator_count; i++)
        {
            commands.push_back(base_feedback.actuators(i).position());

            // Save the current actuator position, to avoid a following error
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
        }

        // Try to initialize gripper — some robots don't have one
        k_api::GripperCyclic::MotorCommand* gripper_motor = nullptr;
        try {
            base_command.mutable_interconnect()->mutable_command_id()->set_identifier(0);
            gripper_motor = base_command.mutable_interconnect()
                                       ->mutable_gripper_command()
                                       ->add_motor_cmd();
            gripper_motor->set_position(base_feedback.interconnect().gripper_feedback().motor(0).position());
            gripper_motor->set_velocity(0.0f);
            gripper_motor->set_force(100.0f);
            std::cout << "[Gripper] Initialized OK" << std::endl;
        } catch (const std::exception& ex) {
            std::cerr << "[Gripper] No gripper detected, continuing without it: " << ex.what() << std::endl;
            gripper_motor = nullptr;
            base_command.clear_interconnect();
        }

        // Send a first frame
        base_feedback = base_cyclic->Refresh(base_command);
        if (gripper_motor) store_gripper_feedback_to_shm(base_feedback);
        std::array<double, 6> tool_wrench{};   // <--- this fixes “not declared”
        const auto& b = base_feedback.base();
        tool_wrench = {
            b.tool_external_wrench_force_x(),
            b.tool_external_wrench_force_y(),
            b.tool_external_wrench_force_z(),
            b.tool_external_wrench_torque_x(),
            b.tool_external_wrench_torque_y(),
            b.tool_external_wrench_torque_z()
        };

        if (shared_feedback) {
            shared_feedback->tool_wrench[0].store(static_cast<float>(tool_wrench[0]), std::memory_order_release); // Fx
            shared_feedback->tool_wrench[1].store(static_cast<float>(tool_wrench[1]), std::memory_order_release); // Fy
            shared_feedback->tool_wrench[2].store(static_cast<float>(tool_wrench[2]), std::memory_order_release); // Fz
            shared_feedback->tool_wrench[3].store(static_cast<float>(tool_wrench[3]), std::memory_order_release); // Tx
            shared_feedback->tool_wrench[4].store(static_cast<float>(tool_wrench[4]), std::memory_order_release); // Ty
            shared_feedback->tool_wrench[5].store(static_cast<float>(tool_wrench[5]), std::memory_order_release); // Tz

            // optional fence to keep all stores ordered as a group
            std::atomic_thread_fence(std::memory_order_release);
        }

        // Set first actuator in torque mode now that the command is equal to measure
        auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);




		for (int i = 1; i <= 7; ++i)  
		{
			actuator_config->SetControlMode(control_mode_message, i);
		}

        // Retrieve and print control loop parameters for actuator 1


    //std::cout << "Running torque control example for " << static_cast<int>(T::TIME_DURATION) << " seconds" << std::endl;

		std::vector<double> velocities(7, 0.0); // Initialize with zero velocities
		std::array<double, 7> prev_torque_cmd{};
		int consecutive_state_loss = 0;
		int consecutive_refresh_errors = 0;

		auto previous_time = std::chrono::high_resolution_clock::now();
		auto current_time = previous_time;
		const int ramp_up_iterations = 10;  // Number of iterations over which to ramp up the torque
		int iteration_count = 0;  // To track the number of iterations
        // Real-time loop



while (timer_count < (60 * 10000000))
       {
        // Exit gracefully on: SIGTERM/SIGINT, fic_control stop signal,
        // or fic_control process died (alive flag dropped).
        if (g_stop_requested) {
            std::cout << "Signal received — exiting torque control loop" << std::endl;
            break;
        }
        if (shared_feedback) {
            if (shared_feedback->stop_torque_mode.load(std::memory_order_acquire)) {
                std::cout << "FIC stop signal — exiting torque control loop" << std::endl;
                break;
            }
            if (!shared_feedback->alive.load(std::memory_order_acquire)) {
                std::cout << "FIC hardware node died — exiting torque control loop" << std::endl;
                break;
            }
        }

       /////////////KORTEX CODE//////////////////////////
		// Update current time
		current_time = std::chrono::high_resolution_clock::now();

		k_api::Base::JointAngles input_joint_angles;
		try
		{
		    input_joint_angles = base->GetMeasuredJointAngles();
		}
		catch (k_api::KDetailedException& ex)
		{
		    THROTTLED_COUT("Unable to get joint angles");
		    return false;
		}

		// Retrieve current torque feedback
		k_api::BaseCyclic::Feedback feedback;
		try
		{
		    feedback = base_cyclic->RefreshFeedback();
		}
		catch (k_api::KDetailedException& ex)
		{
		    THROTTLED_COUT("Unable to get feedback");
		    return false;
		}

		// ── Session / fault guard ──
		// The cyclic feedback carries the arm state for free (no extra RPC).
		// If the arm leaves LOW_LEVEL servoing (fault, someone else took
		// control via the web UI, E-stop, etc.) there's no point continuing
		// — exit so the user can fix the issue and click Start again.
		{
		    auto arm_state = feedback.base().active_state();
		    if (arm_state == k_api::Common::ARMSTATE_IN_FAULT) {
		        std::cout << "[FAULT] Arm entered FAULT during control — "
		                     "stopping automatically." << std::endl;
		        dumpFaultBanks(feedback);
		        return_status = false;
		        break;
		    }
		    if (arm_state != k_api::Common::ARMSTATE_SERVOING_LOW_LEVEL) {
		        THROTTLED_COUT("[SESSION] Arm state changed to "
		            << k_api::Common::ArmState_Name(arm_state)
		            << " — no longer in low-level control. Stopping.");
		        ++consecutive_state_loss;
		        if (consecutive_state_loss > 50) {
		            std::cout << "[SESSION] Lost low-level control for "
		                      << consecutive_state_loss
		                      << " cycles — stopping automatically." << std::endl;
		            return_status = false;
		            break;
		        }
		    } else {
		        consecutive_state_loss = 0;
		    }
		}
		///////////////////////
		
		// Wrench is now read from base_feedback (cyclic Refresh response)
        // instead of feedback (RefreshFeedback), which returns stale data.

		//Update joint positions and velocities
		for (int i = 0; i < input_joint_angles.joint_angles_size(); ++i)
		{
		    double angle_in_radians = input_joint_angles.joint_angles(i).value() * M_PI / 180.0;
		    current_positions[i] = angle_in_radians;
            current_positions[i] = wrapToPi(current_positions[i]);

		}

		
		// Calculate velocities
		for (size_t i = 0; i < 7; ++i)
		{
            
		    double velocity = (wrapToPi(current_positions[i] - previous_positions[i])) / DT;
		    velocities[i] = velocity;
		}

		Eigen::VectorXd q_target(7);  // Create a 7-element vector
        q_target << 0.0, M_PI/12, M_PI, -2.2, 0.0, 1.04, M_PI/2;
		
		//std::cout << "q_target: ";
		for (size_t i = 0; i < kNumJoints; ++i)
		{
			q_target[i] = shared_desired_joints->desired_joint_angles[i].load(std::memory_order_acquire);
			//std::cout << q_target[i] << " ";
		}
		std::cout << std::endl;

		//main loop
		for (size_t i = 0; i < 7; ++i)
		{
		    current_effort[i] = feedback.actuators(i).torque();
		}
		
        std::vector<double> joint_pos_vector(current_positions.data(),
                                current_positions.data() + current_positions.size());

                            
		updateSharedFeedback(current_effort, joint_pos_vector);

        std::array<float, kNumJoints> torque_cmd{};
        if (!shared_feedback) {
            std::fill(torque_cmd.begin(), torque_cmd.end(), 0.0f);
        } else {
            for (size_t i = 0; i < kNumJoints; ++i) {
                float f = shared_feedback->commanded_torque[i].load(std::memory_order_acquire);
                torque_cmd[i] = std::isfinite(f) ? f : 0.0f; // default to 0 if NaN/Inf/unset
            }
        }

        // If you still want torque_data populated:
        for (size_t i = 0; i < 7; ++i) {
            torque_data[i] = static_cast<double>(torque_cmd[i]);
        }


        // fic_ctrl_.initializeControllersAndPlanners(selected_robot_,current_effort, num_joints_);
        // for (size_t i = 0; i < 7; ++i){
        //   fic_ctrl_.handleJointImpedanceControlWithPlanner(i, traj_data, torque_data, q_target[i], current_positions[i], num_joints_);
        //  }
            ///////////////////////////////////////////////////////////////////////////////////////////////////

        
        // Rate-limit torque commands against the previous command (per 1 kHz tick).
        // Joints 0-3 (large actuators): ±2 Nm/ms, joints 4-6 (small): ±1 Nm/ms.
        for (size_t i = 0; i < 7; ++i)
        {
            double max_rate = (i < 4) ? 2.0 : 1.0;
            torque_data[i] = std::clamp(
                torque_data[i], prev_torque_cmd[i] - max_rate, prev_torque_cmd[i] + max_rate
            );
        }


		previous_positions = current_positions;
		previous_time = current_time;
	   
		now = GetTickUs();

		if (now - last > 1000)
		    {	        		    

        for (size_t i = 0; i < 7; ++i){
			base_command.mutable_actuators(i)->set_position(base_feedback.actuators(i).position());
            if (i < 4)
                torque_data[i] = std::clamp(torque_data[i], -39.0, 39.0);
            else
                torque_data[i] = std::clamp(torque_data[i], -13.0, 13.0);
            prev_torque_cmd[i] = torque_data[i];
        }

        for (size_t i = 0; i < 7; ++i){

			base_command.mutable_actuators(i)->set_torque_joint(torque_data[i]);

        }

            base_command.set_frame_id(base_command.frame_id() + 1);
            if (base_command.frame_id() > 65535)

            for (int idx = 0; idx < actuator_count; idx++)
            {
                base_command.mutable_actuators(idx)->set_command_id(base_command.frame_id());
            }
            		            
            // Update gripper if present
            if (gripper_motor) {
                float target_pos = get_gripper_position_from_shm() * 100.0f;
                float actual_pos = base_feedback.interconnect().gripper_feedback().motor(0).position();
                float error = target_pos - actual_pos;
                float velocity = std::min(2.2f * std::abs(error), 100.0f);

                gripper_motor->set_position(target_pos);
                gripper_motor->set_velocity(velocity);
                base_command.mutable_interconnect()->mutable_command_id()->set_identifier(base_command.frame_id());
            }

            try
            {
                base_feedback = base_cyclic->Refresh(base_command, 0);
                consecutive_refresh_errors = 0;

                const auto& bf = base_feedback.base();
                tool_wrench[0] = bf.tool_external_wrench_force_x();
                tool_wrench[1] = bf.tool_external_wrench_force_y();
                tool_wrench[2] = bf.tool_external_wrench_force_z();
                tool_wrench[3] = bf.tool_external_wrench_torque_x();
                tool_wrench[4] = bf.tool_external_wrench_torque_y();
                tool_wrench[5] = bf.tool_external_wrench_torque_z();

                if (shared_feedback) {
                    for (size_t i = 0; i < 6; ++i) {
                        shared_feedback->tool_wrench[i].store(
                            static_cast<float>(tool_wrench[i]), std::memory_order_release);
                    }
                }

                // Measured gripper opening + motor current, for Studio
                // extensions (grasp detection). Only after a good Refresh.
                if (gripper_motor) store_gripper_feedback_to_shm(base_feedback);
            }
            catch (k_api::KDetailedException& ex)
            {
                THROTTLED_COUT("Kortex exception: " << ex.what()
                    << " | Error sub-code: "
                    << k_api::SubErrorCodes_Name(k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));
                ++consecutive_refresh_errors;
            }
            catch (std::runtime_error& ex2)
            {
                THROTTLED_COUT("runtime error: " << ex2.what());
                ++consecutive_refresh_errors;
            }
            catch(...)
            {
                THROTTLED_COUT("Unknown error.");
                ++consecutive_refresh_errors;
            }
            if (consecutive_refresh_errors > 50) {
                std::cout << "[SESSION] " << consecutive_refresh_errors
                          << " consecutive Refresh errors — communication lost. "
                             "Stopping automatically." << std::endl;
                return_status = false;
                break;
            }

            timer_count++;
            last = GetTickUs();
		        }
		    }

		    std::cout << "Torque control example completed" << std::endl;

		    // Arm the teardown watchdog now that the torque loop has exited.
		    std::thread teardown_watchdog([&teardown_done]() {
		        for (int i = 0; i < 30 && !teardown_done.load(); ++i)
		            std::this_thread::sleep_for(std::chrono::milliseconds(100));
		        if (!teardown_done.load()) {
		            std::cout << "[TEARDOWN] Watchdog: teardown hung for 3 s — force exiting." << std::endl;
		            std::cout.flush();
		            _exit(1);
		        }
		    });
		    teardown_watchdog.detach();

		    // --- Fault check BEFORE re-engaging POSITION control mode ---
		    std::string stop_state_name;
		    bool stop_position_ok = true;
		    if (arm_in_fault(base, stop_state_name)) {
		        std::cout << "[FAULT] Robot is in FAULT (" << stop_state_name
		                  << ") at stop — clearing it before returning to position mode."
		                  << std::endl;
		        if (try_clear_faults(base)) {
		            std::cout << "[FAULT] Fault cleared — returning to position mode."
		                      << std::endl;
		        } else {
		            std::cout << "[FAULT] Could not clear the fault — skipping position "
		                         "re-engage. Clear it manually before the next run."
		                      << std::endl;
		            stop_position_ok = false;
		        }
		    }
		    if (stop_position_ok) {
		        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
		        for (int i = 1; i <= 7; ++i)
		            actuator_config->SetControlMode(control_mode_message, i);
		        std::cout << "Torque control example clean exit" << std::endl;
		    }
		}
		catch (k_api::KDetailedException& ex)
		{
		    std::cout << "API error: " << ex.what() << std::endl;
		    return_status = false;
		}
		catch (std::runtime_error& ex2)
		{
		    std::cout << "Error: " << ex2.what() << std::endl;
		    return_status = false;
		}

		// Set the servoing mode back to Single Level — but NOT while faulted.
		std::string teardown_state_name;
		if (arm_in_fault(base, teardown_state_name)) {
			std::cout << "[FAULT] Robot is in FAULT (" << teardown_state_name
			          << ") at teardown — skipping servoing-mode switch."
			          << std::endl;
		} else {
			servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
			base->SetServoingMode(servoing_mode);
		}

		teardown_done.store(true);

		if (shared_feedback)
		    shared_feedback->stop_torque_mode.store(0, std::memory_order_release);

		std::this_thread::sleep_for(std::chrono::milliseconds(25));

		return return_status;
}
private:
};

int main(int argc, char **argv)
{
    auto parsed_args = ParseExampleArguments(argc, argv);
    // Resolve URDF relative to the FIC_Interface project root so the example
    // works regardless of where the repo is checked out. Set FIC_ROOT to
    // override (e.g. running outside the standard layout).
    const char* fic_root_env = getenv("FIC_ROOT");
    std::string fic_root;
    if (fic_root_env) {
        fic_root = std::string(fic_root_env);
    } else {
        // Walk up from the executable looking for the fic_library directory
        // as a landmark for the project root.
        namespace fs = std::filesystem;
        fs::path dir = fs::canonical("/proc/self/exe").parent_path();
        while (!dir.empty() && dir != dir.root_path()) {
            if (fs::is_directory(dir / "fic_library")) {
                fic_root = dir.string();
                break;
            }
            dir = dir.parent_path();
        }
        if (fic_root.empty()) {
            cerr << "ERROR: Could not locate FIC_Interface root. "
                    "Set the FIC_ROOT environment variable." << endl;
            return 1;
        }
    }
    std::string urdf_file = fic_root + "/fic_ws/src/menagerie/model/KinovaGen3_7DOF/URDF/KinovaGen3_7DOF.urdf";
;
	initSharedMemory(); // atomic funciton once at startup 
    // Create API objects
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };

    std::cout << "Creating transport objects" << std::endl;
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(parsed_args.ip_address, PORT);
    
    auto controller = std::make_shared<FICNode<double>>(4.0, false, 0.0);

    std::cout << "Creating transport real time objects" << std::endl;
    auto transport_real_time = new k_api::TransportClientUdp();
    auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(parsed_args.ip_address, PORT_REAL_TIME);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username("admin");
    create_session_info.set_password("admin");
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating sessions for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    auto session_manager_real_time = new k_api::SessionManager(router_real_time);
    session_manager_real_time->CreateSession(create_session_info);
    std::cout << "Sessions created" << std::endl;

    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);
    auto actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(router);
    // Catch SIGTERM/SIGINT so the torque loop can exit gracefully
    // (switch back to position mode) before the process dies.
    std::signal(SIGTERM, stop_signal_handler);
    std::signal(SIGINT,  stop_signal_handler);

    // Example core
    bool success = true;

    // --- Fault check BEFORE any motion ---
    // The home move below uses ExecuteActionFromReference, which FAILS on a
    // faulted arm ("Command failed arm is in fault state"); torque mode is also
    // unsafe on a faulted arm. Report the state, clear the fault, and abort the
    // run (skipping home + torque) if it cannot be cleared — the operator then
    // clears the fault on the robot and restarts.
    std::string startup_state_name;
    if (arm_in_fault(base, startup_state_name)) {
        std::cout << "[FAULT] Robot is in FAULT (" << startup_state_name
                  << ") at startup — clearing it before starting the controller."
                  << std::endl;
        if (try_clear_faults(base)) {
            std::cout << "[FAULT] Fault cleared — starting controller." << std::endl;
        } else {
            std::cout << "[FAULT] Could not clear the fault. Closing — clear it "
                         "manually (Kinova web app / E-stop reset) and restart."
                      << std::endl;
            success = false;
        }
    } else {
        std::cout << "[Startup] Arm state: " << startup_state_name << std::endl;
    }

    if (success) {
        success &= controller->example_move_to_home_position(base);
        success &= controller->example_cyclic_torque_control(base, base_cyclic, actuator_config);
    }

    if (!success)
    {
        std::cout << "There has been an unexpected error." << endl;
    }

    // ── CLEANUP WATCHDOG ──
    // CloseSession / disconnect can also hang on a faulted arm's transport.
    // Force-exit after 3 s so it never locks the system.
    std::thread cleanup_watchdog([]() {
        std::this_thread::sleep_for(std::chrono::seconds(3));
        std::cout << "[CLEANUP] Watchdog: session cleanup hung for 3 s — force exiting." << std::endl;
        std::cout.flush();
        _exit(1);
    });
    cleanup_watchdog.detach();

    // Close API sessions
    session_manager->CloseSession();
    session_manager_real_time->CloseSession();

    // Deactivate routers and disconnect transports
    router->SetActivationStatus(false);
    transport->disconnect();
    router_real_time->SetActivationStatus(false);
    transport_real_time->disconnect();
	cleanupSharedMemory(); // At shutdown for atmoic
    // Destroy the API
    delete base;
    delete base_cyclic;
    delete actuator_config;
    delete session_manager;
    delete session_manager_real_time;
    delete router;
    delete router_real_time;
    delete transport;
    delete transport_real_time;

    return success ? 0 : 1;
}
