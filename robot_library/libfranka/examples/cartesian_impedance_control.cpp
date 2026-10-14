// Copyright (c) 202 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
// This header is required for all Franka SDK examples.

#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <algorithm> // Provides std::min, std::fill, and std::clamp
#include <limits>    // Provides std::numeric_limits (for isfinite checks)
#include <optional>  // Required for older libfranka versions to terminate control loops (std::nullopt)
#include <thread>    // Provides std::this_thread::sleep_for for controlled pausing
#include <chrono>    // Provides std::chrono::milliseconds for sleep durations
#include <atomic>

#include <Eigen/Dense> // Used for matrix/vector operations, common in robotics control

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/gripper.h>  // <-- ADDED FOR GRIPPER CONTROL
#include "examples_common.h" // Provides Franka example boilerplate functions (like setDefaultBehavior)

// SharedFeedback struct and init/cleanup functions live in fic_library.
#include <fic/shared_params.h>

// Franka FR3 is physically 7-DOF. kNumJoints is named here so every
// shared-memory loop is obviously tied to this driver's DOF, and the
// static_assert catches an accidental kMaxJoints regression below 7.
static constexpr size_t kNumJoints = 7;
static_assert(kNumJoints <= kMaxJoints,
              "kNumJoints must fit inside the shared-memory reservation (kMaxJoints).");

// --- Global Variables/Constants for Control Loop Safety ---

// Stores the FINAL torque commanded to the robot in the *previous* 1ms cycle.
std::array<double, 7> previous_commanded_torques{};

// Maximum allowable torque change per 1ms cycle (Torque Delta Limit).
const double MAX_TORQUE_JUMP_PER_MS = 0.7;

// Per-joint soft velocity cap (rad/s). Franka FR3 hard limits are ~2.6 rad/s
// for joints 1-4 and ~2.6-4.2 rad/s for joints 5-7; we stay well below so
// reflexes don't trip. When |dq[i]| exceeds this and the commanded torque
// would accelerate the joint further in the same direction, we replace the
// active torque with pure damping so the joint brakes instead of faulting.
const std::array<double, 7> SOFT_VELOCITY_LIMIT = {
    1.5, 1.5, 1.5, 1.5, 1.8, 1.8, 1.8
};

// Flag to safely shutdown the background gripper thread
std::atomic<bool> run_gripper_thread{true};

// --- Function Prototypes ---
bool attempt_automatic_recovery(const char* hostname);
std::function<franka::Torques(const franka::RobotState&, franka::Duration)> create_torque_callback();


// -----------------------------------------------------------------------------------------
// Background Gripper Control Thread
// -----------------------------------------------------------------------------------------

/**
 * @brief Continuously monitors shared memory and commands the Franka Gripper.
 * Runs in a separate thread because gripper TCP commands block the 1kHz UDP real-time loop.
 */
void gripper_control_loop(const char* hostname) {
    double current_width = 0.08; // Default assumed open
    double speed = 0.1;          // Gripper movement speed (m/s)

    while (run_gripper_thread) {
        try {
            franka::Gripper gripper(hostname);
            std::cout << "Gripper connected successfully." << std::endl;
            
            // Optional: Uncomment the line below if you want the gripper to home itself on startup
            // gripper.homing();

            while (run_gripper_thread) {
                if (shared_feedback) {
                    // Read the target width from the GUI via shared memory
                    double target_width = shared_feedback->commanded_gripper_width.load(std::memory_order_acquire);
                    
                    // Clamp to Franka's physical limits (0.0m closed, ~0.08m open)
                    target_width = std::clamp(target_width, 0.0, 0.08);

                    // Only send a command if the width changed by more than 2 millimeters (0.002m)
                    // This prevents spamming the network with identical commands.
                    if (std::abs(target_width - current_width) > 0.002) {
                        gripper.move(target_width, speed);
                        current_width = target_width;
                    }
                }
                // Sleep for 50ms (20Hz update rate is plenty for the gripper)
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        } catch (const franka::Exception& ex) {
            if (run_gripper_thread) {
                std::cerr << "Gripper error: " << ex.what() << ". Retrying in 2 seconds..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }
        }
    }
}


// -----------------------------------------------------------------------------------------
// Core Data Exchange and Feedback Logic
// -----------------------------------------------------------------------------------------

void updateSharedFeedback(const franka::RobotState& robot_state)
{
    if (!shared_feedback) return;

    const std::array<double, 7>& current_torques = robot_state.tau_ext_hat_filtered;
    for (size_t i = 0; i < current_torques.size(); ++i) {
        shared_feedback->torque[i].store(static_cast<float>(current_torques[i]), std::memory_order_release);
    }

    const std::array<double, 7>& current_joint_positions = robot_state.q;
    for (size_t i = 0; i < current_joint_positions.size(); ++i) {
        shared_feedback->joint_positions[i].store(static_cast<float>(current_joint_positions[i]), std::memory_order_release);
    }

    const std::array<double, 6>& tool_wrench_franka = robot_state.O_F_ext_hat_K;
    for (size_t i = 0; i < tool_wrench_franka.size(); ++i) {
        shared_feedback->tool_wrench[i].store(static_cast<float>(tool_wrench_franka[i]), std::memory_order_release);
    }
}


// -----------------------------------------------------------------------------------------
// Real-Time Control Loop Definition
// -----------------------------------------------------------------------------------------

std::function<franka::Torques(const franka::RobotState&, franka::Duration)> create_torque_callback() {
    return [&](const franka::RobotState& robot_state,
               franka::Duration /*duration*/) -> franka::Torques {
        
        updateSharedFeedback(robot_state);

        std::array<double, 7> current_commanded_torques{};
        std::array<double, 7> Damping_Vector{5.0,11.0,8.0,11.0,0.0,0.05,0.0}; 

        if (!shared_feedback) {
            current_commanded_torques = previous_commanded_torques;
        } else {
            for (size_t i = 0; i < kNumJoints; ++i) {
                float f = shared_feedback->commanded_torque[i].load(std::memory_order_acquire);
                
                if (std::isfinite(f)) {
                    current_commanded_torques[i] = static_cast<double>(f);
                } else {
                    current_commanded_torques[i] = previous_commanded_torques[i]; 
                }

                current_commanded_torques[i] -= Damping_Vector[i] * robot_state.dq[i];

                // Velocity cap: if this joint is already moving fast and the
                // commanded torque would accelerate it further in the same
                // direction, replace with pure damping so the joint brakes.
                const double dq_i = robot_state.dq[i];
                if (std::abs(dq_i) >= SOFT_VELOCITY_LIMIT[i]
                    && (current_commanded_torques[i] > 0.0) == (dq_i > 0.0)
                    && current_commanded_torques[i] != 0.0
                    && dq_i != 0.0) {
                    current_commanded_torques[i] = -Damping_Vector[i] * dq_i;
                }

                current_commanded_torques[i] = std::clamp(
                    current_commanded_torques[i],
                    previous_commanded_torques[i] - MAX_TORQUE_JUMP_PER_MS,
                    previous_commanded_torques[i] + MAX_TORQUE_JUMP_PER_MS
                );
            }
        }
        
        previous_commanded_torques = current_commanded_torques; 
        return current_commanded_torques;
    };
}


// -----------------------------------------------------------------------------------------
// Automatic Recovery and Main Execution
// -----------------------------------------------------------------------------------------

bool attempt_automatic_recovery(const char* hostname) {
    std::cerr << "Attempting automatic recovery from reflex..." << std::endl;
    
    cleanupSharedMemory(); 
    initSharedMemory(); 
    if (!shared_feedback) {
        std::cerr << "FATAL: Shared memory re-initialization failed during recovery." << std::endl;
        return false; 
    }
    
    try {
        franka::Robot robot(hostname);
        robot.automaticErrorRecovery(); 
        std::cout << "Robot fault cleared successfully. Reconnecting to control..." << std::endl;
        
        std::fill(previous_commanded_torques.begin(), previous_commanded_torques.end(), 0.0);
        
        robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                   {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                   {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                   {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
        
        auto torque_command_callback = create_torque_callback();
        robot.control(torque_command_callback);
        
        return true; 
        
    } catch (const franka::Exception& recovery_ex) {
        std::cerr << "Recovery attempt failed with new exception: " << recovery_ex.what() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); 
        return true; 
    }
}


int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }
    const char* hostname = argv[1];

    initSharedMemory(); 
    if (!shared_feedback) {
        std::cerr << "FATAL: Failed to initialize shared memory segments. Exiting." << std::endl;
        cleanupSharedMemory(); 
        return -1;
    }
    
    std::fill(previous_commanded_torques.begin(), previous_commanded_torques.end(), 0.0);
    
    // --- STEP 2: Start the Background Gripper Thread ---
    std::thread g_thread(gripper_control_loop, hostname);
    
    bool running = true;
    
    while (running) {
        try {
            franka::Robot robot(hostname);
            setDefaultBehavior(robot);
            robot.readOnce();
            
            robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                       {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                       {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                       {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

            std::cout << "\nStarting Real-time Torque Control (Automatic Launch)." << std::endl;
            
            auto torque_command_callback = create_torque_callback();
            robot.control(torque_command_callback);
            
            running = false; 

        } catch (const franka::Exception& ex) {
            std::cerr << "\n--- CRITICAL EXCEPTION CAUGHT ---" << std::endl;
            std::cerr << "Franka Exception: " << ex.what() << std::endl;
            
            if (!attempt_automatic_recovery(hostname)) {
                std::cerr << "FATAL: Unrecoverable error. Shutting down." << std::endl;
                running = false; 
            } else {
                std::cout << "Recovery signaled. Main loop will automatically retry connection." << std::endl;
            }
        }
    }

    // --- STEP 3: Stop the Gripper Thread and Cleanup ---
    run_gripper_thread = false;
    if (g_thread.joinable()) {
        g_thread.join();
    }

    cleanupSharedMemory();
    std::cout << "Cleaned up shared memory and exited." << std::endl;

    return 0;
}