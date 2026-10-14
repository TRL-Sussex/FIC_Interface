cmake_minimum_required(VERSION 3.8)
project(fic_simulation)

if(NOT CMAKE_CXX_STANDARD)
  # pinocchio >= 3 (Jazzy ships 4.x) requires C++17; keep it regardless so the
  # solver interface builds identically with or without pinocchio.
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Python3 COMPONENTS Development NumPy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(communication REQUIRED)
find_package(tinyxml2 REQUIRED)
find_package(menagerie REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Pinocchio is OPTIONAL: it backs the default "pinocchio" kinematics solver.
# Without it the node still builds; robots then need kinematics_solver: none
# (degraded joint-space-only mode) or a custom solver registered in
# fic_library/include/fic/solvers/solver_factory.hpp.
option(FIC_WITH_PINOCCHIO "Enable the built-in Pinocchio kinematics/dynamics solver" ON)
if(FIC_WITH_PINOCCHIO)
  find_package(pinocchio QUIET)
  if(NOT pinocchio_FOUND)
    message(WARNING "pinocchio not found — building ${PROJECT_NAME} WITHOUT the built-in "
      "solver. Cartesian IK/FK/Jacobian features will be disabled at runtime unless a "
      "custom kinematics_solver is registered.")
    set(FIC_WITH_PINOCCHIO OFF)
  endif()
endif()

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${Python3_INCLUDE_DIRS}
  ${Python3_NumPy_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../fic_library/include
)

# Add executables
add_executable(fic_simulation src/fic_simulation.cpp)

# Link dependencies for fic_node
ament_target_dependencies(fic_simulation
  rclcpp
  std_msgs
  Eigen3
  Python3
  geometry_msgs
  communication
  tinyxml2
  sensor_msgs
  menagerie
  ament_index_cpp
)

# Explicitly link Python, NumPy, and yaml-cpp for fic_node
target_link_libraries(fic_simulation
  ${Python3_LIBRARIES}
  ${Python3_NumPy_LIBRARIES}
  tinyxml2
  ${EIGEN3_LIBRARIES}
  yaml-cpp
)

if(FIC_WITH_PINOCCHIO)
  # The modern pinocchio::pinocchio imported target propagates include dirs and
  # the boost::variant limits the joint model needs (BOOST_MPL_LIMIT_LIST_SIZE=30
  # etc.) — the legacy ${pinocchio_LIBRARIES} variable drops those definitions.
  target_compile_definitions(fic_simulation PRIVATE FIC_WITH_PINOCCHIO)
  target_link_libraries(fic_simulation pinocchio::pinocchio)
endif()

# Install executables
install(TARGETS fic_simulation
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python script
install(PROGRAMS
  scripts/plotter_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config directories
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# fic_simulation and fic_hardware read config.xml from share/fic_simulation/config, and the
# Studio edits fic_library/config/config.xml. On Linux, colcon --symlink-install makes the
# installed files links to it, so a save or a live trial reaches the controller on its next
# reload. macOS builds without --symlink-install (build.sh), where a copy would only change at
# the next build: link the folder itself there. A copy from an earlier build is replaced.
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


install(DIRECTORY include/
  DESTINATION include
)

# Testing configuration
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()
