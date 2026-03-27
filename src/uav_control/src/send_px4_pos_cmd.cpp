#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <thread>
#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
	public:
	enum class ControlState {
		NAVIGATING,  // 导航到目标点
		LANDING,     // 降落中
		FINISHED     // 完成
	};

	OffboardControl() : Node("offboard_control")
	{

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		vehicle_local_position_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos,
		[this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
			vehicle_local_position_ = *msg;
			has_local_position_ = true;
		});
		land_detected_sub_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected", qos,
		[this](const px4_msgs::msg::VehicleLandDetected::UniquePtr msg) {
			bool landed_ = msg->landed;
			if (landed_ && !landing_complete_ && control_state_ == ControlState::LANDING) {
				RCLCPP_INFO(this->get_logger(), "Drone detected as LANDED!");
				landing_complete_ = true;
			}
		});

		target_position_ = {{0.0, 0.0, -3.0}, {3.0, 0.0, -3.0}, {3.0, -3.0, -3.0}, {0.0, -3.0, -3.0}, {0.0, 0.0, -3.0}}; // square pattern [x1, y1, x2, y2, x3, y3, x4, y4]

		offboard_setpoint_counter_ = 0;
		control_state_ = ControlState::NAVIGATING;

		auto timer_callback = [this]() -> void {

		if (offboard_setpoint_counter_ == 10) {
			// Change to Offboard mode after 10 setpoints
			this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

			// Arm the vehicle
			this->arm();
		}

		// offboard_control_mode needs to be paired with trajectory_setpoint
		//这样保证在切换模式和解锁前，PX4 已经连续收到足够的 setpoint，符合 Offboard 的安全要求
		if (control_state_ == ControlState::NAVIGATING) {
			is_arrive_dead_zone();
		} else if (control_state_ == ControlState::LANDING) {
			handle_landing();
		} else if (control_state_ == ControlState::FINISHED) {
			return;  // Stop publishing, node will exit
		}

		publish_offboard_control_mode();
		publish_trajectory_setpoint();

		// stop the counter after reaching 11
		if (offboard_setpoint_counter_ < 11) {
			offboard_setpoint_counter_++;
		}
		};
		timer_ = this->create_wall_timer(50ms, timer_callback);
	}

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;
	VehicleLocalPosition vehicle_local_position_;
	rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detected_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_subscription_;
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	std::atomic<bool> has_local_position_;
	std::vector<std::vector<double>> target_position_;   //!< target position [x, y, z]
	int target_position_index_ = 0;          //!< current target position index
	ControlState control_state_;             //!< current control state
	double landing_start_height_ = 0.0;      //!< height when landing starts
	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
	bool landing_complete_ = false;

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	bool is_arrive_dead_zone();
	void handle_landing();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void send_land_command();

  	const double DEAD_ZONE_TINY = 0.1; //!< small dead zone for joystick inputs
};

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
  msg.position = true;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};
	
	if (control_state_ == ControlState::NAVIGATING) {
		// Normal navigation to target position
		auto& target = target_position_[target_position_index_];
		msg.position[0] = static_cast<float>(target[0]);
		msg.position[1] = static_cast<float>(target[1]);
		msg.position[2] = static_cast<float>(target[2]);
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		trajectory_setpoint_publisher_->publish(msg);
	}  

}

/**
 * @brief Check if the vehicle has arrived within a dead zone of the target position
 */
bool OffboardControl::is_arrive_dead_zone()
{
  if (!has_local_position_) {
    std::cout << "Waiting for local position..." << std::endl;
    return false;
  }

  auto& target = target_position_[target_position_index_];
  double dx = vehicle_local_position_.x - target[0];
  double dy = vehicle_local_position_.y - target[1];
  double dz = vehicle_local_position_.z - target[2];

  if (std::abs(dx) < DEAD_ZONE_TINY &&
      std::abs(dy) < DEAD_ZONE_TINY &&
      std::abs(dz) < DEAD_ZONE_TINY
  ) {
    // Check if this is the last target position
    if (target_position_index_ == target_position_.size() - 1) {
      // This is the last point, switch to landing mode
      RCLCPP_INFO(this->get_logger(), "Arrived at final target position. Starting landing sequence...");
      control_state_ = ControlState::LANDING;
      landing_start_height_ = vehicle_local_position_.z;
	  send_land_command();
    } else {
      // Move to the next target
      target_position_index_++;
      RCLCPP_INFO(this->get_logger(), "Arrived at target position, moving to target %d", target_position_index_);
    }
    return true;
  }
  return false;
}

/**
 * @brief Handle landing sequence
 *        Gradually descends the vehicle and disarms when it touches the ground
 */
void OffboardControl::handle_landing()
{
	if (!has_local_position_) {
		return;
	}
	if (landing_complete_)
	{
		// Touched ground, disarm and finish
		RCLCPP_INFO(this->get_logger(), "Landing complete. Disarming vehicle...");
		this->disarm();
		
		// Schedule node shutdown after a short delay
		std::thread([this]() {
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
			RCLCPP_INFO(this->get_logger(), "Shutting down node...");
			rclcpp::shutdown();
		}).detach();
		control_state_ = ControlState::FINISHED;
	}
	return;
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

