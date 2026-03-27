#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_imu.hpp>
#include <uav_control/frame_transforms.h>

#include <Eigen/Geometry>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <quadrotor_msgs/msg/corrections.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <quadrotor_msgs/msg/so3_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <cstdio>
#include <algorithm>
#include <array>

class Px4MsgBridge : public rclcpp::Node
{
public:
	explicit Px4MsgBridge()
	: Node("px4_msg_bridge")
	, has_vehicle_attitude_(false)
	, has_local_position_(false)
	, has_vehicle_status_(false)
	, quaternion_px4_ned_frd_(Eigen::Quaterniond::Identity())
	, quaternion_ros_enu_flu_(Eigen::Quaterniond::Identity())
	, mesh_orientation_correction_(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()))
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
		imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
		tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
		
		vehicle_attitude_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
			"/fmu/out/vehicle_attitude", qos,
			std::bind(&Px4MsgBridge::vehicle_attitude_callback, this, std::placeholders::_1));
		vehicle_local_position_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
			"/fmu/out/vehicle_local_position", qos,
			std::bind(&Px4MsgBridge::vehicle_local_position_callback, this, std::placeholders::_1));
		vehicle_status_subscription_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
			"/fmu/out/vehicle_status_v1", qos,
			std::bind(&Px4MsgBridge::vehicle_status_callback, this, std::placeholders::_1));		
		vehicle_imu_subscription_ = this->create_subscription<px4_msgs::msg::VehicleImu>(
			"/fmu/out/vehicle_imu", qos,
			std::bind(&Px4MsgBridge::vehicle_imu_callback, this, std::placeholders::_1));
		
			auto timer_callback = [this]() -> void {
			if(has_local_position_ && has_vehicle_attitude_)
			{
				rclcpp::Time now = this->get_clock()->now();

				const Eigen::Vector3d position_ned(
					vehicle_local_position_.x,
					vehicle_local_position_.y,
					vehicle_local_position_.z);
				const Eigen::Vector3d position_enu =
					uav_control::frame_transforms::ned_to_enu_local_frame(position_ned);

				const Eigen::Vector3d velocity_ned(
					vehicle_local_position_.vx,
					vehicle_local_position_.vy,
					vehicle_local_position_.vz);
				const Eigen::Vector3d velocity_enu =
					uav_control::frame_transforms::ned_to_enu_local_frame(velocity_ned);

				nav_msgs::msg::Odometry odom_msg;
				odom_msg.header.stamp = now;
				odom_msg.header.frame_id = "map";
				odom_msg.child_frame_id = "base_link";
				odom_msg.pose.pose.position.x = position_enu.x();
				odom_msg.pose.pose.position.y = position_enu.y();
				odom_msg.pose.pose.position.z = position_enu.z();
				odom_msg.pose.pose.orientation.w = quaternion_ros_enu_flu_.w();
				odom_msg.pose.pose.orientation.x = quaternion_ros_enu_flu_.x();
				odom_msg.pose.pose.orientation.y = quaternion_ros_enu_flu_.y();
				odom_msg.pose.pose.orientation.z = quaternion_ros_enu_flu_.z();
				odom_msg.twist.twist.linear.x = velocity_enu.x();
				odom_msg.twist.twist.linear.y = velocity_enu.y();
				odom_msg.twist.twist.linear.z = velocity_enu.z();
				// 发布里程计消息
				odom_publisher_->publish(odom_msg);

				// Send the transformation
				geometry_msgs::msg::TransformStamped t;
				t.header.stamp = now;
				t.header.frame_id = "map";
				t.child_frame_id = "base_link";
				t.transform.translation.x = position_enu.x();
				t.transform.translation.y = position_enu.y();
				t.transform.translation.z = position_enu.z();
				t.transform.rotation.w = quaternion_ros_enu_flu_.w();
				t.transform.rotation.x = quaternion_ros_enu_flu_.x();
				t.transform.rotation.y = quaternion_ros_enu_flu_.y();
				t.transform.rotation.z = quaternion_ros_enu_flu_.z();
				tf_broadcaster_->sendTransform(t);
			}
			else{
				// std::cout << "Waiting for all messages..." << std::endl;
			}
		};

		const int rate_hz = std::max(hz, 1); // avoid zero or negative rates
		auto period = std::chrono::duration<double>(1.0 / static_cast<double>(rate_hz));
		timer_ = this->create_wall_timer(period, timer_callback);
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;
	void vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
	void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
	void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
	void vehicle_imu_callback(const px4_msgs::msg::VehicleImu::SharedPtr msg);
	rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_subscription_;
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_subscription_;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscription_;
	rclcpp::Subscription<px4_msgs::msg::VehicleImu>::SharedPtr vehicle_imu_subscription_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
	// rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr imu_publisher_;
	px4_msgs::msg::VehicleAttitude vehicle_attitude_;
	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	px4_msgs::msg::VehicleLocalPosition vehicle_local_position_;
	px4_msgs::msg::VehicleStatus vehicle_status_;
	std::atomic<bool> has_vehicle_attitude_;
	std::atomic<bool> has_local_position_;
	std::atomic<bool> has_vehicle_status_;

	Eigen::Vector3d euler_angles_deg_;
	Eigen::Quaterniond quaternion_px4_ned_frd_;
	Eigen::Quaterniond quaternion_ros_enu_flu_;
	Eigen::Quaterniond mesh_orientation_correction_;
	std::string mesh_resource_uri_;
	int hz = 100; // output rate in Hz
};

void Px4MsgBridge::vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
	vehicle_attitude_ = *msg;
	has_vehicle_attitude_ = true;
	// Convert quaternion to Euler angles
	const Eigen::Quaterniond q_frd_to_ned(
		vehicle_attitude_.q[0],
		vehicle_attitude_.q[1],
		vehicle_attitude_.q[2],
		vehicle_attitude_.q[3]
	);
	quaternion_px4_ned_frd_ = q_frd_to_ned;
	quaternion_ros_enu_flu_ =
		uav_control::frame_transforms::px4_to_ros_orientation(quaternion_px4_ned_frd_);
	euler_angles_deg_ =
		uav_control::frame_transforms::utils::quaternion::quaternion_to_euler_321(quaternion_ros_enu_flu_);
	euler_angles_deg_ = euler_angles_deg_ * (180.0 / M_PI); // Convert to degrees
}

void Px4MsgBridge::vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
	vehicle_local_position_ = *msg;
	has_local_position_ = true;
}

void Px4MsgBridge::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
	vehicle_status_ = *msg;
	has_vehicle_status_ = true;
}

void Px4MsgBridge::vehicle_imu_callback(const px4_msgs::msg::VehicleImu::SharedPtr msg)
{
	const double dt_angle_s = static_cast<double>(msg->delta_angle_dt) * 1e-6;
	const double dt_vel_s = static_cast<double>(msg->delta_velocity_dt) * 1e-6;

	double wx = 0.0, wy = 0.0, wz = 0.0;
	double ax = 0.0, ay = 0.0, az = 0.0;
	if (dt_angle_s > 0.0) {
		wx = static_cast<double>(msg->delta_angle[0]) / dt_angle_s;
		wy = static_cast<double>(msg->delta_angle[1]) / dt_angle_s;
		wz = static_cast<double>(msg->delta_angle[2]) / dt_angle_s;
	}
	if (dt_vel_s > 0.0) {
		ax = static_cast<double>(msg->delta_velocity[0]) / dt_vel_s;
		ay = static_cast<double>(msg->delta_velocity[1]) / dt_vel_s;
		az = static_cast<double>(msg->delta_velocity[2]) / dt_vel_s + 9.81; // add gravity compensation
	}

	const Eigen::Vector3d gyro_frd(wx, wy, wz);
	const Eigen::Vector3d accel_frd(ax, ay, az);
	const Eigen::Vector3d gyro_flu =
		uav_control::frame_transforms::aircraft_to_baselink_body_frame(gyro_frd);
	const Eigen::Vector3d accel_flu =
		uav_control::frame_transforms::aircraft_to_baselink_body_frame(accel_frd);

	sensor_msgs::msg::Imu imu_msg;
	imu_msg.header.frame_id = "base_link";
	imu_msg.header.stamp = rclcpp::Time(msg->timestamp * 1000ULL);
	imu_msg.orientation.w = quaternion_ros_enu_flu_.w();
	imu_msg.orientation.x = quaternion_ros_enu_flu_.x();
	imu_msg.orientation.y = quaternion_ros_enu_flu_.y();
	imu_msg.orientation.z = quaternion_ros_enu_flu_.z();
	imu_msg.orientation_covariance[0] = -1.0; // orientation unknown
	imu_msg.angular_velocity.x = gyro_flu.x();
	imu_msg.angular_velocity.y = gyro_flu.y();
	imu_msg.angular_velocity.z = gyro_flu.z();
	imu_msg.linear_acceleration.x = accel_flu.x();
	imu_msg.linear_acceleration.y = accel_flu.y();
	imu_msg.linear_acceleration.z = accel_flu.z();
	imu_publisher_->publish(imu_msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting px4_msg_bridge node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Px4MsgBridge>());
	rclcpp::shutdown();
	return 0;
}
