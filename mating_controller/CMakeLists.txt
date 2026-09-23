cmake_minimum_required(VERSION 3.8)
project(mating_controller)

# The portable, robot-agnostic half of the connector-mating cell: the phase
# machine, the pose maths, the tracking law, and the two nodes that drive
# them. Nothing here knows which arm it is on - the robot-specific wiring
# lives in the params yaml and the launch files (FR3: tools/fr3/). The
# MELFA-only GPIO/HMI demos live in the melfa_cell package.

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

set(INSTALL_TARGETS "")

# mating_node plans through MoveIt. The switch is NOT cosmetic: this machine's
# MoveIt install raises hard CMake errors from inside its own config files
# (missing tl::expected and random_numbers imported targets), and QUIET does
# not suppress those - a find_package here takes the WHOLE package down with
# it, including the pure headers and their gtests, which need neither MoveIt
# nor a robot.
#
# OFF by default since 2026-09-23: the autonomous path is parked (the FR3 is
# driven from tools/fr3/cell_panel.py + tracking_node), and its MELFA launch
# files and params now live in melfa/parked/ - see the README there. The
# phase machine's gtests still build and run. On a machine with a working
# MoveIt:
#
#   colcon build --packages-select mating_controller --cmake-args -DBUILD_MATING_NODE=ON
#
option(BUILD_MATING_NODE "Build the mating_node MoveIt node (needs a working MoveIt install)" OFF)
if(BUILD_MATING_NODE)
  find_package(moveit_ros_planning_interface REQUIRED)
  add_executable(mating_node src/mating_node.cpp)
  target_include_directories(mating_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(mating_node PUBLIC cxx_std_17)
  ament_target_dependencies(mating_node
    rclcpp moveit_ros_planning_interface geometry_msgs tf2 tf2_ros tf2_geometry_msgs std_srvs diagnostic_msgs controller_manager_msgs)
  list(APPEND INSTALL_TARGETS mating_node)
else()
  message(STATUS "BUILD_MATING_NODE=OFF - skipping mating_node (parked); the phase machine, pose maths and tracking law still build and test")
endif()

# franka_msgs only exists where the FR3 is commissioned; the tracking node
# reads o_t_ee from it, so gate the executable. tracking_law.hpp and its
# gtest stay unconditional - the law is tested everywhere, only its
# plumbing is not built.
find_package(franka_msgs QUIET)
if(franka_msgs_FOUND)
  add_executable(tracking_node src/tracking_node.cpp)
  target_include_directories(tracking_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(tracking_node PUBLIC cxx_std_17)
  ament_target_dependencies(tracking_node
    rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs std_srvs franka_msgs lifecycle_msgs
    controller_manager_msgs diagnostic_msgs)
  list(APPEND INSTALL_TARGETS tracking_node)
else()
  message(WARNING "franka_msgs not found - skipping tracking_node (source ~/franka_ros2_ws/install/setup.sh to build it)")
endif()

if(INSTALL_TARGETS)
  install(TARGETS ${INSTALL_TARGETS}
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_mating_geometry test/test_mating_geometry.cpp)
  target_include_directories(test_mating_geometry PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_mating_geometry tf2)

  ament_add_gtest(test_phase_machine test/test_phase_machine.cpp)
  target_include_directories(test_phase_machine PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)

  ament_add_gtest(test_tracking_law test/test_tracking_law.cpp)
  target_include_directories(test_tracking_law PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_tracking_law tf2)
endif()

ament_package()
