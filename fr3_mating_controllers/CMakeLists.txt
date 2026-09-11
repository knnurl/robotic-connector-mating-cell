cmake_minimum_required(VERSION 3.8)
project(fr3_mating_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# The Franka stack exists only where the FR3 is commissioned (this PC via
# ~/franka_ros2_ws, or the robot PC). Elsewhere - e.g. the MELFA cell -
# build nothing, mirroring the plc_/hmi_ conditional pattern.
find_package(franka_semantic_components QUIET)

if(NOT franka_semantic_components_FOUND)
  message(WARNING
    "franka_semantic_components not found - skipping fr3_mating_controllers "
    "(source ~/franka_ros2_ws/install/setup.sh to build it)")
  ament_package()
  return()
endif()

find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/cartesian_impedance_stroke_controller.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  franka_semantic_components)

pluginlib_export_plugin_description_file(
  controller_interface fr3_mating_controllers.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle
  geometry_msgs franka_semantic_components)

ament_package()
