cmake_minimum_required(VERSION 3.8)
project(uav_control)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C99
if(NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

#################
# Setup targets #
#################
include_directories(include)
# Add frame_transforms lib
add_library(frame_transforms SHARED src/lib/frame_transforms.cpp)
ament_target_dependencies(frame_transforms Eigen3 geometry_msgs sensor_msgs)
target_include_directories(frame_transforms PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_library(tarj_generator SHARED src/lib/tarj_generator.cpp)
ament_target_dependencies(tarj_generator Eigen3 rclcpp px4_msgs visualization_msgs geometry_msgs std_msgs)
target_link_libraries(tarj_generator frame_transforms)
target_include_directories(tarj_generator PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_library(poly_solver SHARED src/poly_solver.cpp)
ament_target_dependencies(poly_solver Eigen3)
target_include_directories(poly_solver PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# examples/uav_control/px4_msg_listener
add_executable(px4_msg_listener src/px4_msg_listener.cpp)
ament_target_dependencies(px4_msg_listener rclcpp px4_msgs)
target_link_libraries(px4_msg_listener frame_transforms)
install(TARGETS px4_msg_listener DESTINATION lib/${PROJECT_NAME})

add_executable(mocap_to_px4 src/mocap_to_px4.cpp)
ament_target_dependencies(mocap_to_px4 rclcpp px4_msgs geometry_msgs)
target_link_libraries(mocap_to_px4 frame_transforms)
install(TARGETS mocap_to_px4 DESTINATION lib/${PROJECT_NAME})

add_executable(lio_to_px4 src/lio_to_px4.cpp)
ament_target_dependencies(lio_to_px4 rclcpp px4_msgs nav_msgs)
target_link_libraries(lio_to_px4 frame_transforms)
install(TARGETS lio_to_px4 DESTINATION lib/${PROJECT_NAME})

add_executable(send_px4_pos_cmd src/send_px4_pos_cmd.cpp)
ament_target_dependencies(send_px4_pos_cmd rclcpp px4_msgs)
install(TARGETS send_px4_pos_cmd DESTINATION lib/${PROJECT_NAME})

# examples/get_uav_state/sub_px4_msgs
add_executable(sub_all_px4_msgs get_uav_state/sub_all_px4_msgs.cpp)
ament_target_dependencies(sub_all_px4_msgs rclcpp px4_msgs)
target_link_libraries(sub_all_px4_msgs frame_transforms)
install(TARGETS sub_all_px4_msgs DESTINATION lib/${PROJECT_NAME})

add_executable(px4_msg_bridge src/px4_msg_bridge.cpp)
ament_target_dependencies(px4_msg_bridge rclcpp px4_msgs std_msgs nav_msgs quadrotor_msgs tf2 tf2_ros visualization_msgs ament_index_cpp)
target_link_libraries(px4_msg_bridge frame_transforms)
install(TARGETS px4_msg_bridge DESTINATION lib/${PROJECT_NAME})

add_executable(px4_so3_example src/px4_so3_example.cpp)
ament_target_dependencies(px4_so3_example rclcpp px4_msgs quadrotor_msgs)
target_link_libraries(px4_so3_example frame_transforms)
install(TARGETS px4_so3_example DESTINATION lib/${PROJECT_NAME})

add_executable(poly_tarj_publish_demo src/poly_tarj_publish_demo.cpp)
ament_target_dependencies(poly_tarj_publish_demo rclcpp geometry_msgs visualization_msgs)
target_link_libraries(poly_tarj_publish_demo poly_solver)
install(TARGETS poly_tarj_publish_demo DESTINATION lib/${PROJECT_NAME})

add_executable(uav_control_terminal src/uav_control_terminal.cpp)
ament_target_dependencies(uav_control_terminal rclcpp px4_msgs visualization_msgs quadrotor_msgs)
target_link_libraries(uav_control_terminal frame_transforms tarj_generator poly_solver)
install(TARGETS uav_control_terminal DESTINATION lib/${PROJECT_NAME})

############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(
	ament_cmake
	rclcpp
	rosidl_default_runtime
	eigen3_cmake_module
	Eigen3
	px4_msgs
	geometry_msgs
	sensor_msgs
	std_msgs
	nav_msgs
	quadrotor_msgs
	tf2
	tf2_ros
	visualization_msgs
	ament_index_cpp
)

ament_export_targets(export_frame_transforms HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_libraries(frame_transforms poly_solver tarj_generator)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

install(TARGETS frame_transforms poly_solver tarj_generator
	EXPORT export_frame_transforms
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

ament_package()
