cmake_minimum_required(VERSION 3.8)
project(melfa_cell)

# MELFA RV-5AS cell hardware only: the GPIO/gripper/safety demos from the
# ROS-I AP Annual Summit 2024 Masterclass, plus one untouched legacy demo.
# The portable mating controller lives in the mating_controller package;
# nothing here is needed to run the FR3, which is the active target.

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# The MELFA GPIO stack only exists on the robot PC; build the PLC/HMI nodes
# conditionally so this package still configures anywhere.
find_package(melfa_msgs QUIET)
find_package(melfa_masterclass_msgs QUIET)

set(INSTALL_TARGETS "")

if(melfa_msgs_FOUND AND melfa_masterclass_msgs_FOUND)
  add_executable(plc_ src/plc_.cpp)
  target_compile_features(plc_ PUBLIC cxx_std_17)
  ament_target_dependencies(plc_ rclcpp melfa_msgs melfa_masterclass_msgs std_msgs)

  add_executable(hmi_ src/hmi_.cpp)
  target_compile_features(hmi_ PUBLIC cxx_std_17)
  ament_target_dependencies(hmi_ rclcpp melfa_msgs melfa_masterclass_msgs std_msgs)

  list(APPEND INSTALL_TARGETS plc_ hmi_)
else()
  message(WARNING "melfa_msgs / melfa_masterclass_msgs not found - skipping plc_ and hmi_")
endif()

# Untouched legacy demo: blind scripted motion, no vision, no guards. Off by
# default so it cannot be run on a live cell by accident; enable with
# -DBUILD_LEGACY_DEMOS=ON if you really need it.
option(BUILD_LEGACY_DEMOS "Build legacy demo executables (pick_n_place_)" OFF)
if(BUILD_LEGACY_DEMOS)
  find_package(moveit_ros_planning_interface REQUIRED)
  add_executable(pick_n_place_ src/pick_n_place_.cpp)
  target_compile_features(pick_n_place_ PUBLIC cxx_std_17)
  ament_target_dependencies(pick_n_place_ rclcpp moveit_ros_planning_interface std_msgs)
  list(APPEND INSTALL_TARGETS pick_n_place_)
endif()

if(INSTALL_TARGETS)
  install(TARGETS ${INSTALL_TARGETS}
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
