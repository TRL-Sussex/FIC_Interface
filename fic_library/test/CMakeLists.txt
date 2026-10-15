# Host tests for the portable inner-loop code (fic_joint_core.h, fic_link.h,
# inner_loop.h, mcu_node.h). Standalone — not part of the colcon workspace:
#
#   cmake -S fic_library/test -B build/fic_tests && cmake --build build/fic_tests
#   ctest --test-dir build/fic_tests --output-on-failure
cmake_minimum_required(VERSION 3.16)
project(fic_library_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
add_compile_options(-Wall -Wextra)

enable_testing()
set(FIC_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../include)

# Portable code only: no Eigen, no ROS.
add_executable(test_inner_loop test_inner_loop.cpp)
target_include_directories(test_inner_loop PRIVATE ${FIC_INCLUDE})
add_test(NAME inner_loop COMMAND test_inner_loop)

# Equivalence against fic_control.h, and H-PMP's tuning, need Eigen + tinyxml2.
find_package(Eigen3 QUIET NO_MODULE)
find_path(TINYXML2_INCLUDE tinyxml2.h)
find_library(TINYXML2_LIB tinyxml2)
if(Eigen3_FOUND AND TINYXML2_INCLUDE AND TINYXML2_LIB)
  add_executable(test_joint_core_equivalence test_joint_core_equivalence.cpp)
  target_include_directories(test_joint_core_equivalence PRIVATE ${FIC_INCLUDE} ${TINYXML2_INCLUDE})
  target_link_libraries(test_joint_core_equivalence PRIVATE Eigen3::Eigen ${TINYXML2_LIB})
  # Bit-identical comparison: keep the compiler from contracting a*b+c into
  # FMAs differently in the two code paths.
  target_compile_options(test_joint_core_equivalence PRIVATE -ffp-contract=off)
  add_test(NAME joint_core_equivalence COMMAND test_joint_core_equivalence)
  # H-PMP's tuning: which blocks each mode reads (both nodes), its own block and the fallback.
  add_executable(test_hpmp_tuning test_hpmp_tuning.cpp)
  target_include_directories(test_hpmp_tuning PRIVATE ${FIC_INCLUDE} ${TINYXML2_INCLUDE})
  target_link_libraries(test_hpmp_tuning PRIVATE ${TINYXML2_LIB})
  add_test(NAME hpmp_tuning COMMAND test_hpmp_tuning)
  # Haptic joint admittance: driven by the interaction torque only (no creep, push yields, release holds).
  add_executable(test_haptic_admittance test_haptic_admittance.cpp)
  target_include_directories(test_haptic_admittance PRIVATE ${FIC_INCLUDE} ${TINYXML2_INCLUDE})
  target_link_libraries(test_haptic_admittance PRIVATE Eigen3::Eigen ${TINYXML2_LIB})
  target_compile_options(test_haptic_admittance PRIVATE -ffp-contract=off)
  add_test(NAME haptic_admittance COMMAND test_haptic_admittance)

  add_executable(test_hpmp_caps test_hpmp_caps.cpp)
  target_include_directories(test_hpmp_caps PRIVATE ${FIC_INCLUDE} ${TINYXML2_INCLUDE})
  target_link_libraries(test_hpmp_caps PRIVATE Eigen3::Eigen ${TINYXML2_LIB})
  add_test(NAME hpmp_caps COMMAND test_hpmp_caps)
  # H-PMP on host modes 7/8: the task-space admittance shifts the task goal (as the host's), IK shift J# dx.
  add_executable(test_hpmp_task_admittance test_hpmp_task_admittance.cpp)
  target_include_directories(test_hpmp_task_admittance PRIVATE ${FIC_INCLUDE} ${TINYXML2_INCLUDE})
  target_link_libraries(test_hpmp_task_admittance PRIVATE Eigen3::Eigen ${TINYXML2_LIB})
  add_test(NAME hpmp_task_admittance COMMAND test_hpmp_task_admittance)
  # H-PMP's split admittance drive: a task-space deadband on the end-effector part of the torque gap.
  add_executable(test_hpmp_split_drive test_hpmp_split_drive.cpp)
  target_include_directories(test_hpmp_split_drive PRIVATE ${FIC_INCLUDE} ${TINYXML2_INCLUDE})
  target_link_libraries(test_hpmp_split_drive PRIVATE Eigen3::Eigen ${TINYXML2_LIB})
  add_test(NAME hpmp_split_drive COMMAND test_hpmp_split_drive)
  # Joint planner on a continuous joint crossing +-pi: no lag from the 2*pi seam.
  add_executable(test_planner_wrap test_planner_wrap.cpp)
  target_include_directories(test_planner_wrap PRIVATE ${FIC_INCLUDE} ${TINYXML2_INCLUDE})
  target_link_libraries(test_planner_wrap PRIVATE Eigen3::Eigen ${TINYXML2_LIB})
  add_test(NAME planner_wrap COMMAND test_planner_wrap)
else()
  message(WARNING "Eigen3/tinyxml2 not found: skipping test_joint_core_equivalence")
endif()

# End-to-end over UDP: McuLink (fic_hardware's end) against fic_mcu_sim (the
# MCU code on a simulated arm). ctest starts and stops the simulator. Only when
# the MCU code is in this copy of the repository.
set(FIC_MCU_SIM_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../../robot_library/fic_mcu/sim/fic_mcu_sim.cpp)
if(EXISTS ${FIC_MCU_SIM_SRC})
find_package(Threads REQUIRED)
add_executable(fic_mcu_sim ${FIC_MCU_SIM_SRC})
target_include_directories(fic_mcu_sim PRIVATE ${FIC_INCLUDE})
add_executable(test_mcu_link_loop test_mcu_link_loop.cpp)
target_include_directories(test_mcu_link_loop PRIVATE ${FIC_INCLUDE})
target_link_libraries(test_mcu_link_loop PRIVATE Threads::Threads)
add_test(NAME mcu_link_end_to_end
         COMMAND sh -c "$<TARGET_FILE:fic_mcu_sim> --port 7351 >/dev/null & pid=$!; sleep 0.3; $<TARGET_FILE:test_mcu_link_loop> 127.0.0.1:7351; r=$?; kill $pid; exit $r")
endif()
