#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <uav_control/frame_transforms.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <cstdio>
#include <algorithm>
using namespace px4_msgs::msg;

class Px4MsgListener : public rclcpp::Node
{
public:
	explicit Px4MsgListener()
	: Node("px4_msg_listener")
	, has_vehicle_attitude_(false)
	, has_battery_status_(false)
	, has_local_position_(false)
	, has_vehicle_status_(false)
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		
		vehicle_attitude_subscription_ = this->create_subscription<VehicleAttitude>(
			"/fmu/out/vehicle_attitude", qos,
			std::bind(&Px4MsgListener::vehicle_attitude_callback, this, std::placeholders::_1));
		battery_status_subscription_ = this->create_subscription<BatteryStatus>(
			"/fmu/out/battery_status", qos,
			std::bind(&Px4MsgListener::battery_status_callback, this, std::placeholders::_1));
		vehicle_local_position_subscription_ = this->create_subscription<VehicleLocalPosition>(
			"/fmu/out/vehicle_local_position", qos,
			std::bind(&Px4MsgListener::vehicle_local_position_callback, this, std::placeholders::_1));
		vehicle_status_subscription_ = this->create_subscription<VehicleStatus>(
			"/fmu/out/vehicle_status_v1", qos,
			std::bind(&Px4MsgListener::vehicle_status_callback, this, std::placeholders::_1));		
		
		auto timer_callback = [this]() -> void {
			if (has_vehicle_attitude_ && has_battery_status_ && has_local_position_ && has_vehicle_status_) {
				std::cout << "Vehicle Status Armed State: " << static_cast<int>(vehicle_status_.arming_state) << std::endl;
				std::cout << "Vehicle Mode: " << static_cast<int>(vehicle_status_.nav_state_user_intention) << std::endl;
				std::cout << "----------------------------------------" << std::endl;
				std::cout << "Vehicle Attitude (quat) qx/qy/qz/qw: " 
						  << vehicle_attitude_.q[0] << " "
						  << vehicle_attitude_.q[1] << " "
						  << vehicle_attitude_.q[2] << " "
						  << vehicle_attitude_.q[3] << std::endl;
				std::cout << "Vehicle Attitude (deg) roll/pitch/yaw: " << euler_angles_deg_.transpose() << std::endl;
				std::cout << "Battery Voltage: " << battery_status_.voltage_v << " V" << std::endl;
				std::cout << "Local Position X Y Z(m): " << vehicle_local_position_.x << " " << vehicle_local_position_.y << " " << vehicle_local_position_.z << std::endl;
				std::cout << "Velocity in NED frame X Y Z(m/s): " << vehicle_local_position_.vx << " " << vehicle_local_position_.vy << " " << vehicle_local_position_.vz << std::endl;
				std::cout << "----------------------------------------" << std::endl;
			} else {
				std::cout << "Waiting for all messages..." << std::endl;
			}
		};

		const int rate_hz = std::max(hz, 1); // avoid zero or negative rates
		auto period = std::chrono::duration<double>(1.0 / static_cast<double>(rate_hz));
		timer_ = this->create_wall_timer(period, timer_callback);
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;
	void vehicle_attitude_callback(const VehicleAttitude::SharedPtr msg);
	void battery_status_callback(const BatteryStatus::SharedPtr msg);
	void vehicle_local_position_callback(const VehicleLocalPosition::SharedPtr msg);
	void vehicle_status_callback(const VehicleStatus::SharedPtr msg);
	rclcpp::Subscription<VehicleAttitude>::SharedPtr vehicle_attitude_subscription_;
	rclcpp::Subscription<BatteryStatus>::SharedPtr battery_status_subscription_;
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_subscription_;
	rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_subscription_;
	VehicleAttitude vehicle_attitude_;
	BatteryStatus battery_status_;
	VehicleLocalPosition vehicle_local_position_;
	VehicleStatus vehicle_status_;
	std::atomic<bool> has_vehicle_attitude_;
	std::atomic<bool> has_battery_status_;
	std::atomic<bool> has_local_position_;
	std::atomic<bool> has_vehicle_status_;

	Eigen::Vector3d euler_angles_deg_;
	Eigen::Quaterniond quaternion_;
	int hz = 1; // output rate in Hz
};

void Px4MsgListener::vehicle_attitude_callback(const VehicleAttitude::SharedPtr msg)
{
	vehicle_attitude_ = *msg;
	has_vehicle_attitude_ = true;

	// Convert quaternion to Euler angles
	quaternion_ = Eigen::Quaterniond(
		vehicle_attitude_.q[0],
		vehicle_attitude_.q[1],
		vehicle_attitude_.q[2],
		vehicle_attitude_.q[3]
	);
	euler_angles_deg_ = uav_control::frame_transforms::utils::quaternion::quaternion_to_euler_321(quaternion_);
}

void Px4MsgListener::battery_status_callback(const BatteryStatus::SharedPtr msg)
{	
	battery_status_ = *msg;
	has_battery_status_ = true;
}

void Px4MsgListener::vehicle_local_position_callback(const VehicleLocalPosition::SharedPtr msg)
{
	vehicle_local_position_ = *msg;
	has_local_position_ = true;
}

void Px4MsgListener::vehicle_status_callback(const VehicleStatus::SharedPtr msg)
{
	vehicle_status_ = *msg;
	has_vehicle_status_ = true;
}

int main(int argc, char *argv[])
{
	std::cout << "Starting sensor_combined listener node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Px4MsgListener>());

	rclcpp::shutdown();
	return 0;
}
