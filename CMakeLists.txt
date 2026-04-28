cmake_minimum_required(VERSION 3.5)
project(visionconnect)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(OpenCV 4 REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBRARIES}")
message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
find_package(CUDA REQUIRED)
find_package(jetson-utils REQUIRED)
find_package(jetson-inference REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(vpi 3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(message_filters REQUIRED)
find_package(X11 REQUIRED)

link_directories(/usr/lib/aarch64-linux-gnu/tegra)
link_directories(/opt/nvidia/vpi2/lib64)

# CUDA library for stereo rotation kernel
cuda_add_library(stereo_rotate_cuda STATIC src/cuda/stereo_rotate.cu)
set_target_properties(stereo_rotate_cuda PROPERTIES CUDA_ARCHITECTURES "87")

# CUDA preprocessing kernel for Fast-FoundationStereo stereo_depth node
cuda_add_library(stereo_preproc_cuda STATIC src/cuda/stereo_preproc.cu)
set_target_properties(stereo_preproc_cuda PROPERTIES CUDA_ARCHITECTURES "87")

# Shared library
file(GLOB_RECURSE common_src RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/common/*.cpp)
include_directories(src/common/include)
add_library(common STATIC ${common_src})
target_link_libraries(common ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} jetson-inference cudart)

# TensorRT related files
set(TENSORRT_LIBS jetson-inference nvinfer cudart)
add_definitions(-DROS2)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Ofast -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")


# Create executable nodes
add_executable(camera src/node_camera.cpp ${common_src})
add_executable(camera_stereo src/node_camera_stereo.cpp ${common_src})
add_executable(detect src/node_detect.cpp ${common_src})
add_executable(classify src/node_classify.cpp ${common_src})
add_executable(lanedet src/node_lanedet.cpp ${common_src})
add_executable(preview src/node_preview.cpp ${common_src})
add_executable(dashboard src/node_dashboard.cpp ${common_src})
add_executable(gui src/node_gui.cpp ${common_src})
add_executable(adas src/node_adas.cpp ${common_src})
add_executable(driver_monitor src/node_driver_monitor.cpp ${common_src})
add_executable(imu_gps src/node_imu_gps.cpp ${common_src})
add_executable(bev src/node_bev.cpp ${common_src})
add_executable(stereo_depth src/node_stereo_depth.cpp)

# 'include' directories
include_directories(src/include ${catkin_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(/opt/ros/humble/include/rmw)
include_directories(/opt/ros/humble/include/rosidl_runtime_c)
include_directories(/opt/ros/humble/include/rosidl_typesupport_interface)
include_directories(${rcl_interfaces_INCLUDE_DIRS})
include_directories(${sensor_msgs_INCLUDE_DIRS})
include_directories(${vision_msgs_INCLUDE_DIRS})


target_link_libraries(camera
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${TENSORRT_LIBS}
    jetson-inference
    common
)
target_link_libraries(camera_stereo
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${TENSORRT_LIBS}
    common
    stereo_rotate_cuda
)
target_link_libraries(detect
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${TENSORRT_LIBS}
    jetson-inference
    common
)
target_link_libraries(classify
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${TENSORRT_LIBS}
    jetson-inference
    common
)
target_link_libraries(lanedet
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${TENSORRT_LIBS}
    jetson-inference
    common
)
target_link_libraries(preview
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${TENSORRT_LIBS}
    jetson-inference
    common
)

target_link_libraries(dashboard
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${TENSORRT_LIBS}
    jetson-inference
    common
)

target_link_libraries(gui
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${TENSORRT_LIBS}
    ${X11_LIBRARIES}
    jetson-inference
    common
)

target_link_libraries(adas
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${TENSORRT_LIBS}
    jetson-inference
    common
)

target_link_libraries(driver_monitor
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    common
    nvinfer
    cudart
)

target_link_libraries(bev
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${TENSORRT_LIBS}
    jetson-inference
    common
)

target_link_libraries(imu_gps
    ${catkin_LIBRARIES}
    common
    cudart
    nvinfer
    gps
)

target_link_libraries(stereo_depth
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    nvinfer
    cudart
    dl
    stereo_preproc_cuda
)

# Include custom messages
set(msg_files
  "msg/Box.msg"
  "msg/Detect.msg"
  "msg/Lanes.msg"
  "msg/Signs.msg"
  "msg/Track.msg"
  "msg/ADAS.msg"
  "msg/SceneData.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files} 
			DEPENDENCIES builtin_interfaces sensor_msgs  std_msgs rosidl_typesupport_cpp)
rosidl_target_interfaces(detect ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(classify ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(lanedet ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(preview ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(camera_stereo ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(dashboard ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(gui ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(adas ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(bev ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_export_dependencies(rosidl_default_runtime)

ament_target_dependencies(camera
    ament_index_cpp
    rclcpp
    sensor_msgs
    vision_msgs
    std_msgs
)

ament_target_dependencies(camera_stereo
    ament_index_cpp
    rclcpp
    sensor_msgs
    vision_msgs
    std_msgs
    OpenCV
)

ament_target_dependencies(detect
    ament_index_cpp
    rclcpp
    sensor_msgs
    vision_msgs
    std_msgs
)

ament_target_dependencies(classify
    ament_index_cpp
    rclcpp
    sensor_msgs
    vision_msgs
    std_msgs
)

ament_target_dependencies(lanedet
    ament_index_cpp
    rclcpp
    sensor_msgs
    vision_msgs
    std_msgs
)

ament_target_dependencies(preview
    ament_index_cpp
    rclcpp
    sensor_msgs
    vision_msgs
    std_msgs
)

ament_target_dependencies(dashboard
    ament_index_cpp
    rclcpp
    sensor_msgs
    vision_msgs
    std_msgs
)

ament_target_dependencies(gui
    ament_index_cpp
    rclcpp
    sensor_msgs
    vision_msgs
    std_msgs
)

ament_target_dependencies(adas
    ament_index_cpp
    rclcpp
    sensor_msgs
    vision_msgs
    std_msgs
)

ament_target_dependencies(bev
    ament_index_cpp
    rclcpp
    sensor_msgs
    geometry_msgs
    vision_msgs
    std_msgs
)

 ament_target_dependencies(driver_monitor
    ament_index_cpp
    rclcpp
    sensor_msgs
    std_msgs
    OpenCV
)

ament_target_dependencies(imu_gps
    ament_index_cpp
    rclcpp
    sensor_msgs
    geometry_msgs
)

ament_target_dependencies(stereo_depth
    ament_index_cpp
    rclcpp
    sensor_msgs
    std_msgs
    message_filters
)

# Install executables to ROS project
install(TARGETS camera DESTINATION lib/${PROJECT_NAME})
install(TARGETS camera_stereo DESTINATION lib/${PROJECT_NAME})
install(TARGETS detect DESTINATION lib/${PROJECT_NAME})
install(TARGETS classify DESTINATION lib/${PROJECT_NAME})
install(TARGETS lanedet DESTINATION lib/${PROJECT_NAME})
install(TARGETS preview DESTINATION lib/${PROJECT_NAME})
install(TARGETS dashboard DESTINATION lib/${PROJECT_NAME})
install(TARGETS gui DESTINATION lib/${PROJECT_NAME})
install(TARGETS adas DESTINATION lib/${PROJECT_NAME})
install(TARGETS bev DESTINATION lib/${PROJECT_NAME})
install(TARGETS driver_monitor DESTINATION lib/${PROJECT_NAME})
install(TARGETS imu_gps DESTINATION lib/${PROJECT_NAME})
install(TARGETS stereo_depth DESTINATION lib/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
    scripts/display_driver_monitor.py
    scripts/test_stereo_from_files.py
    scripts/display_disparity.py
    DESTINATION lib/${PROJECT_NAME}
)

# Copy config to build
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

# Copy graphs to build
install (DIRECTORY src/graphs DESTINATION share/${PROJECT_NAME}/)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
