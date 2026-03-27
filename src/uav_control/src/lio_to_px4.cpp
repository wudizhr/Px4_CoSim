#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <uav_control/frame_transforms.h>

#include <cmath>
#include <limits>
#include <memory>
#include <string>

using namespace px4_msgs::msg;

class Lio2Px4 : public rclcpp::Node
{
public:
    Lio2Px4() : Node("lio_to_px4")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        lio_pose_topic_ = this->declare_parameter<std::string>(
            "lio_pose_topic", "/Odometry");

        // 创建订阅者
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            lio_pose_topic_,
            qos,  // QoS队列深度
            std::bind(&Lio2Px4::topic_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "LIO odom subscriber started on topic: %s", lio_pose_topic_.c_str());
        
        // 创建发布者
        publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", qos);
        RCLCPP_INFO(this->get_logger(), "Vehicle visual vehicle_odometry_msg_ publisher started");
    }

private:
    void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pub_vehicle_odometry_msg();
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr publisher_;
    nav_msgs::msg::Odometry lio_odom_;
    Eigen::Vector3d lio_att_rpy_;
    std::string lio_pose_topic_;

    VehicleOdometry vehicle_odometry_msg_;
};

void Lio2Px4::topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    lio_odom_ = *msg;
    lio_att_rpy_ = uav_control::frame_transforms::utils::quaternion::quaternion_to_euler_321(Eigen::Quaterniond(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z
    )); 

    // 在这里处理接收到的消息
    RCLCPP_INFO(this->get_logger(), "Received LIO odom position: [%.2f, %.2f, %.2f]",
                msg->pose.pose.position.x,
                msg->pose.pose.position.y,
                msg->pose.pose.position.z);

    RCLCPP_INFO(this->get_logger(), "LIO Yaw: %.2f degrees", lio_att_rpy_.z() * 180.0 / M_PI);

    pub_vehicle_odometry_msg();
}

void Lio2Px4::pub_vehicle_odometry_msg() 
{
    // 设置时间戳
    vehicle_odometry_msg_.timestamp = this->now().nanoseconds() / 1000; // PX4 uses microseconds

    vehicle_odometry_msg_.timestamp_sample = vehicle_odometry_msg_.timestamp; // Assuming same timestamp for sample

    // 设置位置（注意坐标系转换）
    vehicle_odometry_msg_.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    vehicle_odometry_msg_.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;

    // 
    vehicle_odometry_msg_.position[0] = lio_odom_.pose.pose.position.x;
    vehicle_odometry_msg_.position[1] = -lio_odom_.pose.pose.position.y;
    vehicle_odometry_msg_.position[2] = -lio_odom_.pose.pose.position.z;

    Eigen::Vector3d mocap_att_ned;
    mocap_att_ned.x() = lio_att_rpy_.x();               // roll
    mocap_att_ned.y() = lio_att_rpy_.y();               // pitch
    mocap_att_ned.z() = -lio_att_rpy_.z();
    Eigen::Quaterniond q_ned = uav_control::frame_transforms::utils::quaternion::quaternion_from_rpy_321(mocap_att_ned);
    Eigen::Vector3d att_cal = uav_control::frame_transforms::utils::quaternion::quaternion_to_euler_321(q_ned);

    RCLCPP_INFO(this->get_logger(), "att_cal Yaw: %.2f degrees", att_cal.z() * 180.0 / M_PI);

    // 设置姿态四元数
    vehicle_odometry_msg_.q[0] = q_ned.w();
    vehicle_odometry_msg_.q[1] = q_ned.x();
    vehicle_odometry_msg_.q[2] = q_ned.y();
    vehicle_odometry_msg_.q[3] = q_ned.z();
    
    // 暂时设为 NaN
    for (int i = 0; i < 3; ++i) {
        vehicle_odometry_msg_.velocity[i] = std::numeric_limits<float>::quiet_NaN();
        vehicle_odometry_msg_.angular_velocity[i] = std::numeric_limits<float>::quiet_NaN();
    }

    // 设置方差，默认为 NaN 表示未知
    for (int i = 0; i < 3; ++i) {
        vehicle_odometry_msg_.position_variance[i] = std::numeric_limits<float>::quiet_NaN();
        vehicle_odometry_msg_.orientation_variance[i] = std::numeric_limits<float>::quiet_NaN();
        vehicle_odometry_msg_.velocity_variance[i] = std::numeric_limits<float>::quiet_NaN();
    }

    // 其他字段根据需要设置
    vehicle_odometry_msg_.reset_counter = 0; // 如果适用，重置计数器
    vehicle_odometry_msg_.quality = 0;       // 如果适用，质量等级

    publisher_->publish(vehicle_odometry_msg_);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Lio2Px4>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}