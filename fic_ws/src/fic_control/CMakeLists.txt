cmake_minimum_required(VERSION 3.14)
project(fic_control)

# macOS drops DYLD_LIBRARY_PATH whenever a program starts through a system binary (/usr/bin/env, /bin/bash: the
# launcher's and the Studio's own way of starting things), so the workspace's libraries (communication's messages)
# are found through each binary's own rpath instead: every linked library's folder goes into it.
if(APPLE)
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)
endif()

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
find_package(sensor_msgs REQUIRED)
find_package(communication REQUIRED)
find_package(Eigen3 REQUIRED)
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
add_executable(fic_control src/fic_control.cpp)

# Add the compile definition (AFTER add_executable)
target_compile_definitions(fic_control PRIVATE
  PACKAGE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

# Reference the executable target name for dependencies
ament_target_dependencies(fic_control
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  communication
  ament_index_cpp
)

# Reference the executable target name for include directories
target_include_directories(fic_control PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../fic_library/include
)

# Reference the executable target name for linking libraries
target_link_libraries(fic_control
  tinyxml2::tinyxml2
  yaml-cpp
)

if(FIC_WITH_PINOCCHIO)
  # Link the modern pinocchio::pinocchio imported target (not ${pinocchio_LIBRARIES}):
  # it propagates the boost::variant limits the joint model needs
  # (BOOST_MPL_LIMIT_LIST_SIZE=30 etc.) plus the urdf parser and include dirs.
  # The legacy variable drops those definitions, which breaks the 26-type
  # JointModel variant on pinocchio >= 3.
  target_compile_definitions(fic_control PRIVATE FIC_WITH_PINOCCHIO)
  target_include_directories(fic_control PRIVATE ${urdfdom_headers_INCLUDE_DIRS})
  target_link_libraries(fic_control pinocchio::pinocchio)
endif()

# Install the executables
# The joint inner loop as a PC process (on the vendor driver's shared memory): no ROS, no Eigen.
add_executable(fic_inner_loop src/fic_inner_loop.cpp)
target_include_directories(fic_inner_loop PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../../fic_library/include)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_link_libraries(fic_inner_loop rt)  # shm_open on older glibc; macOS has it in libSystem, and no librt
endif()

install(TARGETS
  fic_control
  fic_inner_loop
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch directory: robot_control_launch.py (a real robot) and sim_control_launch.py (a simulated one)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(PROGRAMS
  scripts/plotter_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# The node reads config.xml from share/fic_control/config, and the Studio edits
# fic_library/config/config.xml. On Linux, colcon --symlink-install makes the installed files links to
# it, so a save or a live trial reaches the controller on its next reload. macOS builds without
# --symlink-install (build.sh), where a copy would only change at the next build: link the folder
# itself there. A copy from an earlier build is replaced.
get_filename_component(FIC_CONFIG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../fic_library/config REALPATH)
if(APPLE)
  install(CODE "
    set(_share \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}\")
    set(_dst \"\${_share}/config\")
    if(IS_SYMLINK \"\${_dst}\")
      file(REMOVE \"\${_dst}\")
    elseif(EXISTS \"\${_dst}\")
      file(REMOVE_RECURSE \"\${_dst}\")
    endif()
    file(MAKE_DIRECTORY \"\${_share}\")
    file(CREATE_LINK \"${FIC_CONFIG_DIR}\" \"\${_dst}\" SYMBOLIC)
    message(STATUS \"Linking: \${_dst} -> ${FIC_CONFIG_DIR}\")
  ")
else()
  install(DIRECTORY ${FIC_CONFIG_DIR}
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

ament_package()