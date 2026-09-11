cmake_minimum_required(VERSION 3.8)
project(melfa_rv5as_masterclass)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

# The MELFA GPIO stack only exists on the robot PC; build the PLC/HMI
# nodes conditionally so the rest of the package compiles anywhere.
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
  add_executable(pick_n_place_ src/pick_n_place_.cpp)
  target_compile_features(pick_n_place_ PUBLIC cxx_std_17)
  ament_target_dependencies(pick_n_place_ rclcpp moveit_ros_planning_interface std_msgs)
  list(APPEND INSTALL_TARGETS pick_n_place_)
endif()

add_executable(move_l src/move_l.cpp)
target_include_directories(move_l PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(move_l PUBLIC cxx_std_17)
ament_target_dependencies(move_l
  rclcpp moveit_ros_planning_interface geometry_msgs tf2 tf2_ros tf2_geometry_msgs std_srvs diagnostic_msgs controller_manager_msgs)

list(APPEND INSTALL_TARGETS move_l)

# Install C++ binaries
install(TARGETS ${INSTALL_TARGETS}
  DESTINATION lib/${PROJECT_NAME}
)

# Install additional resources
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mating_geometry test/test_mating_geometry.cpp)
  target_include_directories(test_mating_geometry PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_mating_geometry tf2)

  ament_add_gtest(test_phase_machine test/test_phase_machine.cpp)
  target_include_directories(test_phase_machine PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()

ament_package()

