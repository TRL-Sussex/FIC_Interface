cmake_minimum_required(VERSION 3.14)
project(fic_hardware)

if(NOT CMAKE_CXX_STANDARD)
  # pinocchio >= 3 (Jazzy ships 4.x) requires C++17; building its headers under
  # C++14 breaks boost::variant template instantiation (JointModelMimic).
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fic_simulation REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tinyxml2 REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Pinocchio is OPTIONAL: it backs the default "pinocchio" kinematics solver.
# Without it the node still builds; robots then need kinematics_solver: none
# (degraded joint-space-only mode) or a custom solver registered in
# fic_library/include/fic/solvers/solver_factory.hpp.
option(FIC_WITH_PINOCCHIO "Enable the built-in Pinocchio kinematics/dynamics solver" ON)
if(FIC_WITH_PINOCCHIO)
  find_package(pinocchio QUIET)
  if(pinocchio_FOUND)
    # pinocchio/parsers/urdf.hpp includes <urdf_model/model.h>, which on ROS Jazzy
    # lives under .../include/urdfdom_headers/. pinocchio's CMake config does not
    # propagate that include dir, so pull it in explicitly.
    find_package(urdfdom_headers REQUIRED)
  else()
    message(WARNING "pinocchio not found — building ${PROJECT_NAME} WITHOUT the built-in "
      "solver. Cartesian IK/FK/Jacobian features will be disabled at runtime unless a "
      "custom kinematics_solver is registered.")
    set(FIC_WITH_PINOCCHIO OFF)
  endif()
endif()

# Node: Robot_Control_Node
add_executable(fic_hardware src/fic_hardware.cpp)

# Add the compile definition (AFTER add_executable)
target_compile_definitions(fic_hardware PRIVATE
  PACKAGE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

# Reference the executable target name for dependencies
ament_target_dependencies(fic_hardware
  rclcpp
  std_msgs
  geometry_msgs
  ament_index_cpp
)

# Reference the executable target name for include directories
target_include_directories(fic_hardware PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${fic_simulation_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../fic_library/include

)

# Reference the executable target name for linking libraries
target_link_libraries(fic_hardware
  ${fic_simulation_LIBRARIES}
  tinyxml2::tinyxml2
  yaml-cpp
)

if(FIC_WITH_PINOCCHIO)
  # Link the modern pinocchio::pinocchio imported target (not ${pinocchio_LIBRARIES}):
  # it propagates the boost::variant limits the joint model needs
  # (BOOST_MPL_LIMIT_LIST_SIZE=30 etc.) plus the urdf parser and include dirs.
  # The legacy variable drops those definitions, which breaks the 26-type
  # JointModel variant on pinocchio >= 3.
  target_compile_definitions(fic_hardware PRIVATE FIC_WITH_PINOCCHIO)
  target_include_directories(fic_hardware PRIVATE ${urdfdom_headers_INCLUDE_DIRS})
  target_link_libraries(fic_hardware pinocchio::pinocchio)
endif()

# Install the executables
install(TARGETS
  fic_hardware
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch directory (if any)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install Python script
install(PROGRAMS
  scripts/plotter_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()