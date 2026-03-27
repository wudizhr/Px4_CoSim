// common
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <cstdio>
#include <algorithm>

// 自定义头文件库
#include "sunray_logger.h"
#include "uav_control/frame_transforms.h"



// Eigen
#include <Eigen/Dense>

// ROS话题消息头文件

// px4_msgs
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/estimator_status_flags.hpp>
#include <px4_msgs/msg/failsafe_flags.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_imu.hpp>
#include <px4_msgs/msg/vehicle_magnetometer.hpp>

// 命名空间
using namespace std;
using namespace sunray_logger;
using namespace px4_msgs::msg;
