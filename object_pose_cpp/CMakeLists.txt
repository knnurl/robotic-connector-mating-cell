cmake_minimum_required(VERSION 3.16)
project(object_pose_cpp CXX)

# Release unless told otherwise: an unoptimised build is several times
# slower, and the frame budget is the point of this package.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG REQUIRED)

# The Python package (object_pose_cpp/__init__.py), then the extension
# module next to it.
ament_python_install_package(${PROJECT_NAME})

pybind11_add_module(_object_pose_cpp src/bindings.cpp src/estimator.cpp)
target_include_directories(_object_pose_cpp PRIVATE src ${EIGEN3_INCLUDE_DIR})
target_link_libraries(_object_pose_cpp PRIVATE ${OpenCV_LIBS})
install(TARGETS _object_pose_cpp DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")

ament_package()
