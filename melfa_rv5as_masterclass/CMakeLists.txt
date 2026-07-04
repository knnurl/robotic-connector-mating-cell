cmake_minimum_required(VERSION 3.8)
project(melfa_rv5as_masterclass)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(melfa_msgs REQUIRED)
find_package(melfa_masterclass_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Add C++ executables
add_executable(plc_ src/plc_.cpp)
target_include_directories(plc_ PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(plc_ PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(plc_ rclcpp melfa_msgs melfa_masterclass_msgs std_msgs)

add_executable(hmi_ src/hmi_.cpp)
target_include_directories(hmi_ PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hmi_ PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(hmi_ rclcpp melfa_msgs melfa_masterclass_msgs std_msgs)

add_executable(pick_n_place_ src/pick_n_place_.cpp)
target_include_directories(pick_n_place_ PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pick_n_place_ PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(pick_n_place_ rclcpp moveit_ros_planning_interface melfa_msgs melfa_masterclass_msgs std_msgs)

add_executable(move_l src/move_l.cpp)
target_include_directories(move_l PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(move_l PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(move_l rclcpp moveit_ros_planning_interface melfa_msgs melfa_masterclass_msgs std_msgs)

# Install Python script for cam_pub

install(PROGRAMS
  scripts/cam_pub.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install C++ binaries
install(TARGETS plc_ hmi_ pick_n_place_ move_l
  DESTINATION lib/${PROJECT_NAME}
)

# Install additional resources
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

