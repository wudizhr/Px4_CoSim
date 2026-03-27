#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <quadrotor_msgs/msg/so3_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <iostream>
#include <cmath>
#include <functional>
#include <chrono>
#include <algorithm>
#include <Eigen/Geometry>

using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
	enum class ControlState {
		ARMING,  // 解锁中
		TESTING, //测试中
		LANDING,     // 降落中
		FINISHED     // 完成
	};
	OffboardControl() : Node("offboard_control")
	{
		std::cout << "OffboardControl Node Initialized" << std::endl;
		std::cout << "kMaxTotalThrust(N): " << kMaxTotalThrust << std::endl;
		attitude_setpoint_publisher_ = this->create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		so3_command_sub_ =	this->create_subscription<quadrotor_msgs::msg::SO3Command>(
			"so3_cmd", qos,
			std::bind(&OffboardControl::vehicle_attitude_setpoint_callback, this, std::placeholders::_1));
		control_state_ = ControlState::ARMING;
		offboard_setpoint_counter_ = 0;
		timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&OffboardControl::timer_callback, this));
	}
	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Subscription<quadrotor_msgs::msg::SO3Command>::SharedPtr so3_command_sub_;
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_publisher_;

	void publish_offboard_control_mode();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void send_land_command();
	void publish_attitude_setpoint(px4_msgs::msg::VehicleAttitudeSetpoint att_setpoint);
	void vehicle_attitude_setpoint_callback(const quadrotor_msgs::msg::SO3Command::SharedPtr msg);
	void timer_callback();
	void send_so3_command();
	void hover();

	std::atomic<bool> has_so3_command_;
	quadrotor_msgs::msg::SO3Command latest_so3_command_;
	ControlState control_state_;             //!< current control state
	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
	VehicleAttitudeSetpoint default_attitude_setpoint_;

	static constexpr double kMotorConstant = 8.54858e-06; // SDF: motorConstant
	static constexpr double kMaxRotVelocity = 1000.0;     // SDF: maxRotVelocity (rad/s)
	static constexpr int kMotorCount = 4;
	static constexpr double kMaxTotalThrust = kMotorCount * kMotorConstant * kMaxRotVelocity * kMaxRotVelocity;
};

void OffboardControl::hover()
{
	VehicleAttitudeSetpoint att_setpoint{};
	att_setpoint.thrust_body[2] = - 18.185 / kMaxTotalThrust; //测出来的无人机质量，这里直接写死
	publish_offboard_control_mode();
	publish_attitude_setpoint(att_setpoint);
}

void OffboardControl::timer_callback()
{
	if (offboard_setpoint_counter_ == 100 && control_state_ == ControlState::ARMING) {
		// Change to Offboard mode after 10 setpoints
		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

		// Arm the vehicle
		this->arm();
		control_state_ = ControlState::TESTING;
	}
	if(control_state_ == ControlState::ARMING)
	{
		publish_offboard_control_mode();
		publish_attitude_setpoint(default_attitude_setpoint_);
	}
	else if (control_state_ == ControlState::TESTING) {
		if (has_so3_command_) {
			send_so3_command();
		}
		else {
			hover();
		}
	}
	// stop the counter after reaching 11
	if (offboard_setpoint_counter_ < 101) {
		offboard_setpoint_counter_++;
	}
}

void OffboardControl::send_so3_command()
{
	VehicleAttitudeSetpoint att_setpoint{};

	// 直接使用 SO3Command 的姿态与推力（不做任何坐标系转换）
	const Eigen::Quaterniond q_frd_to_ned(
		latest_so3_command_.orientation.w,
		latest_so3_command_.orientation.x,
		latest_so3_command_.orientation.y,
		latest_so3_command_.orientation.z);
	att_setpoint.q_d[0] = static_cast<float>(q_frd_to_ned.w()); // w
	att_setpoint.q_d[1] = static_cast<float>(q_frd_to_ned.x()); // x
	att_setpoint.q_d[2] = static_cast<float>(q_frd_to_ned.y()); // y
	att_setpoint.q_d[3] = static_cast<float>(q_frd_to_ned.z()); // z

	const Eigen::Vector3d force_ned(latest_so3_command_.force.x, latest_so3_command_.force.y, latest_so3_command_.force.z);
	const Eigen::Vector3d thrust_body_frd = q_frd_to_ned.conjugate() * force_ned;
	const double thrust_body_frd_z = thrust_body_frd.z();
	// std::cout << "thrust_body_frd_z" << thrust_body_frd.z() << std::endl;
	const double thrust_body_frd_z_norm = std::min(0.0, std::max(thrust_body_frd_z / kMaxTotalThrust, -0.95));

	// 推力施加在机体 Z 轴方向（FRD 坐标系），正值表示向下推力
	att_setpoint.thrust_body[0] = 0.0f; // X 轴推力为 0（不控制）
	att_setpoint.thrust_body[1] = 0.0f; // Y 轴推力为 0（不控制）
	att_setpoint.thrust_body[2] = static_cast<float>(thrust_body_frd_z_norm);

	if(control_state_ == ControlState::TESTING)
	{
		publish_offboard_control_mode();
		publish_attitude_setpoint(att_setpoint);
	}
}

void OffboardControl::vehicle_attitude_setpoint_callback(const quadrotor_msgs::msg::SO3Command::SharedPtr msg)
{
	latest_so3_command_ = *msg;
	has_so3_command_ = true;
}

void OffboardControl::publish_attitude_setpoint(px4_msgs::msg::VehicleAttitudeSetpoint att_setpoint)
{
	attitude_setpoint_publisher_->publish(att_setpoint);
}

void OffboardControl::send_land_command()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
	RCLCPP_INFO(this->get_logger(), "Land command sent!");
}

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

  RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

  RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
  OffboardControlMode msg{};
  msg.position = false;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = true;
  msg.body_rate = false;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  offboard_control_mode_publisher_->publish(msg);
}
/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 * 作用：封装并发布一条 PX4 VehicleCommand，用于发送 MAVLink 风格的指令（如切换模式、解锁/上锁等）。见 offboard_control.cpp:164-177。
字段含义：command 对应 MAV_CMD/VehicleCommand 枚举；param1/param2 传入的参数；target_system/component 设为 1 指向主飞控；source_system/component 设为 1 表示本节点；from_external = true 标记来自外部控制；timestamp 用 ROS2 纳秒转微秒，符合 PX4 uORB 时间单位。
发布：通过 vehicle_command_publisher_ 推送到 /fmu/in/vehicle_command，PX4 接收后执行对应动作。
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}

