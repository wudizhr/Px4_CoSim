#include <cstdio>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include "quadrotor_msgs/msg/position_command.hpp"
#include <quadrotor_msgs/msg/so3_command.hpp>
#include <std_msgs/msg/empty.hpp>

#include "ros_msg_utils.h"
#include "sunray_logger.h"
#include "uav_control/command_state.hpp"
#include "uav_control/tarj_generator.hpp"

using namespace std;
using namespace uav_control;

class TerminalNode : public rclcpp::Node
{
public:
	explicit TerminalNode()
	: Node("terminal_node")
	{
		default_target_z = this->declare_parameter<double>("default_target_z", -1.0);
		sample_hz = this->declare_parameter<double>("sample_hz", 50.0);
		kMaxTotalThrust = this->declare_parameter<double>("kMaxTotalThrust", 34.194320);
		uav_name_ = this->declare_parameter<string>("uav_name", "default_uav");
		tarj_generator_.set_sample_hz(sample_hz);
		// 初始化自定义日志，避免后续 Logger::print_* 抛出未初始化异常
		Logger::init_default();
		Logger::print_color(int(LogColor::green), "Loaded parameter default_target_z = " + std::to_string(default_target_z));
		Logger::print_color(int(LogColor::green), "Loaded parameter sample_hz = " + std::to_string(sample_hz));
		Logger::print_color(int(LogColor::green), "Loaded parameter kMaxTotalThrust = " + std::to_string(kMaxTotalThrust));
		Logger::print_color(int(LogColor::green), "Loaded parameter uav_name = " + uav_name_);
		tarj_generator_.init(this);
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		SO3_cmd_pub_ = this->create_publisher<quadrotor_msgs::msg::PositionCommand>("/position_cmd", 10);
		attitude_setpoint_publisher_ = this->create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", 10);
		traj_server_reset_pub_ = this->create_publisher<std_msgs::msg::Empty>("/traj_server/reset", 10);

		publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000/sample_hz)), [this]() {
			publish_command();
		});
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		land_detected_sub_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected", qos,
		[this](const px4_msgs::msg::VehicleLandDetected::UniquePtr msg) {
			if (msg->landed && !is_land && (cmd == CommandState::LANDING || cmd == CommandState::LAND)) {
				Logger::print_color(int(LogColor::magenta), "Drone detected as LANDED!");
				is_land = true;
				cmd = CommandState::OTHER; // Reset command after landing
			}
		});
		// 【订阅】
		vehicle_attitude_subscription_ = this->create_subscription<VehicleAttitude>(
			"/fmu/out/vehicle_attitude", qos,
			std::bind(&TerminalNode::vehicle_attitude_callback, this, std::placeholders::_1));
		so3_command_sub_ =	this->create_subscription<quadrotor_msgs::msg::SO3Command>(
			"so3_cmd", qos,
			std::bind(&TerminalNode::terminal_so3_cmd_callback, this, std::placeholders::_1));
		planner_so3_command_sub_ = this->create_subscription<quadrotor_msgs::msg::SO3Command>(
			"planner_so3_cmd", qos,
			std::bind(&TerminalNode::planner_so3_cmd_callback, this, std::placeholders::_1));
		vehicle_status_subscription_ = this->create_subscription<VehicleStatus>(
			"/fmu/out/vehicle_status_v1", qos,
			std::bind(&TerminalNode::vehicle_status_callback, this, std::placeholders::_1));
		vehicle_local_position_subscription_ = this->create_subscription<VehicleLocalPosition>(
			"/fmu/out/vehicle_local_position", qos,
			std::bind(&TerminalNode::vehicle_local_position_callback, this, std::placeholders::_1));	
        // Start input thread
        std::thread input_thread([this]() {
			int input = 0;
            while (rclcpp::ok()) {
                Logger::print_color(int(LogColor::cyan), ">>>>>>>>>>>>>>>> uav_control_terminal <<<<<<<<<<<<<<<<<");
				Logger::print_color(int(LogColor::green), "当前控制器：" + string(current_controller == ControlerKind::PX4 ? "PX4内置控制器" : "SO3控制器"));
				Logger::print_color(int(LogColor::cyan), "Please input command:");
				Logger::print_color(int(LogColor::cyan), "  0: unlock");
				Logger::print_color(int(LogColor::cyan), "  1: takeoff");
				Logger::print_color(int(LogColor::cyan), "  2: go_to_waypoint");
				Logger::print_color(int(LogColor::cyan), "  3: land");
				Logger::print_color(int(LogColor::cyan), "  4: hover");
				Logger::print_color(int(LogColor::cyan), "  5: trajectory");
				Logger::print_color(int(LogColor::cyan), "  6: change controller");
				Logger::print_color(int(LogColor::cyan), "  7: ego planner");
				if (!(cin >> input)) {
					cin.clear();
					string dummy; getline(cin, dummy);
					continue;
				}
				handle_input_command(input);
            }
        });
        input_thread.detach();    
    }

private:
	void publish_command();
	void arm();
	void disarm();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void send_land_command();
	void keep_armed();
	void hover();
	void vehicle_status_callback(const VehicleStatus::SharedPtr msg);
	void vehicle_local_position_callback(const VehicleLocalPosition::SharedPtr msg);
	void arm_function();
	void handle_input_command(int input);
	// 处理 SO3Command 消息
	void terminal_so3_cmd_callback(const quadrotor_msgs::msg::SO3Command::SharedPtr msg);
	void planner_so3_cmd_callback(const quadrotor_msgs::msg::SO3Command::SharedPtr msg);

	void publish_attitude_setpoint(px4_msgs::msg::VehicleAttitudeSetpoint att_setpoint);
	void send_so3_command();
	void uav_move_control();
	void vehicle_attitude_callback(const VehicleAttitude::SharedPtr msg);
	void planner_init();
	void vehicle_attitude_setpoint(const quadrotor_msgs::msg::SO3Command msg);

private:
	rclcpp::TimerBase::SharedPtr publish_timer_;
	CommandState cmd = CommandState::OTHER;
	TarjKind current_traj = TarjKind::OTHER;
	ControlerKind current_controller = ControlerKind::PX4;

	int arm_count = 0;
	int tarj_count = 0;
	int sample_hz = 50;

	bool is_land = true;
	bool is_armed = false;
	bool is_first_planner_command = false;

	double target_x = 0.0;
	double target_y = 0.0;
	double target_z = 0.0;
	double target_vx = 0.0;
	double target_vy = 0.0;
	double target_vz = 0.0;
	double target_ax = 0.0;
	double target_ay = 0.0;
	double target_az = 0.0;
	double target_yaw = 0.0;
	double default_target_z = -1.0; // 默认目标高度为 5 米
	double kMaxTotalThrust = 34.194320;
	string uav_name_;

	Eigen::Vector3d euler_angles_rad_;
	Eigen::Quaterniond quaternion_;
	VehicleLocalPosition vehicle_local_position_;
	VehicleAttitude vehicle_attitude_;
	std::atomic<bool> has_local_position_ = false;
	TarjGenerator tarj_generator_;

	rclcpp::Subscription<VehicleAttitude>::SharedPtr vehicle_attitude_subscription_;
	rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detected_sub_;
	rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_subscription_;
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_subscription_;
	rclcpp::Subscription<quadrotor_msgs::msg::SO3Command>::SharedPtr so3_command_sub_;
	rclcpp::Subscription<quadrotor_msgs::msg::SO3Command>::SharedPtr planner_so3_command_sub_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr SO3_cmd_pub_;
	rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr traj_server_reset_pub_;

	rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_publisher_;

};

void TerminalNode::planner_init()
{
	hover(); // 先悬停
	if (traj_server_reset_pub_)
	{
		traj_server_reset_pub_->publish(std_msgs::msg::Empty());
	}
	is_first_planner_command = false;
	cmd = CommandState::OTHER; // 重置命令状态
	Logger::print_color(int(LogColor::yellow), "Exiting planner mode and hovering...");
}

void TerminalNode::vehicle_attitude_callback(const VehicleAttitude::SharedPtr msg)
{
	// 处理车辆姿态消息
	vehicle_attitude_ = *msg;
	// Convert quaternion to Euler angles
	quaternion_ = Eigen::Quaterniond(
		vehicle_attitude_.q[0],
		vehicle_attitude_.q[1],
		vehicle_attitude_.q[2],
		vehicle_attitude_.q[3]
	);
	euler_angles_rad_ = uav_control::frame_transforms::utils::quaternion::quaternion_to_euler_321(quaternion_);
}

void TerminalNode::uav_move_control()
{
	if(current_controller == ControlerKind::SO3)
	{
		send_so3_command();
	}
	else if(current_controller == ControlerKind::PX4)
	{
		publish_offboard_control_mode();
		publish_trajectory_setpoint();
	}
}

void TerminalNode::send_so3_command()
{
	auto cmd = quadrotor_msgs::msg::PositionCommand();
	cmd.position.x = target_x;
	cmd.position.y = target_y;
	cmd.position.z = target_z;
	cmd.velocity.x = target_vx;
	cmd.velocity.y = target_vy;
	cmd.velocity.z = target_vz;
	cmd.acceleration.x = target_ax;
	cmd.acceleration.y = target_ay;
	cmd.acceleration.z = target_az;
	cmd.yaw = target_yaw;
	SO3_cmd_pub_->publish(cmd);
}

void TerminalNode::publish_attitude_setpoint(px4_msgs::msg::VehicleAttitudeSetpoint att_setpoint)
{
	attitude_setpoint_publisher_->publish(att_setpoint);
}

void TerminalNode::vehicle_attitude_setpoint(const quadrotor_msgs::msg::SO3Command msg)
{
	VehicleAttitudeSetpoint att_setpoint{};
	// 直接使用 SO3Command 的姿态与推力（不做任何坐标系转换）
	const Eigen::Quaterniond q(
		msg.orientation.w,
		msg.orientation.x,
		msg.orientation.y,
		msg.orientation.z);
	att_setpoint.q_d[0] = static_cast<float>(q.w()); // w
	att_setpoint.q_d[1] = static_cast<float>(q.x()); // x
	att_setpoint.q_d[2] = static_cast<float>(q.y()); // y
	att_setpoint.q_d[3] = static_cast<float>(q.z()); // z

	const Eigen::Vector3d force(msg.force.x, msg.force.y, msg.force.z);
	const Eigen::Vector3d thrust_body_frd = q.conjugate() * force; // 将力从世界坐标系转换到机体坐标系（FRD）
	const double thrust_body_frd_z = thrust_body_frd.z();
	const double thrust_body_frd_z_norm = std::min(0.0, std::max(thrust_body_frd_z / kMaxTotalThrust, -0.95));

	// 推力施加在机体 Z 轴方向（FRD 坐标系），正值表示向下推力
	att_setpoint.thrust_body[0] = 0.0f; // X 轴推力为 0（不控制）
	att_setpoint.thrust_body[1] = 0.0f; // Y 轴推力为 0（不控制）
	att_setpoint.thrust_body[2] = static_cast<float>(thrust_body_frd_z_norm);	

	publish_offboard_control_mode();
	publish_attitude_setpoint(att_setpoint);	
}

void TerminalNode::terminal_so3_cmd_callback(const quadrotor_msgs::msg::SO3Command::SharedPtr msg)
{
	if (current_controller == ControlerKind::SO3 && (cmd == CommandState::GO_TO_WAYPOINT || cmd == CommandState::TRAJ || (cmd == CommandState::PLANNER && !is_first_planner_command))) // 仅在使用 SO3 控制器且处于飞行状态时才发布姿态控制命令
	{
		vehicle_attitude_setpoint(*msg);
	}
}

void TerminalNode::planner_so3_cmd_callback(const quadrotor_msgs::msg::SO3Command::SharedPtr msg)
{
	if (current_controller == ControlerKind::SO3 && cmd == CommandState::PLANNER) //	仅在使用 SO3 控制器且处于规划状态时才发布姿态控制命令
	{
		if(is_first_planner_command)
		{
			vehicle_attitude_setpoint(*msg);
		}
		 else
		{
			Logger::print_color(int(LogColor::yellow), "Received first planner command, initializing trajectory generator...");
			is_first_planner_command = true;
		}
	}
}

void TerminalNode::arm_function()
{
	if(arm_count < 10)
	{
		publish_offboard_control_mode();
		publish_trajectory_setpoint();
		arm_count++;
	}
	else if(arm_count == 10)
	{
		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
		arm();
		Logger::print_color(int(LogColor::green), "Vehicle is already armed!");
		if(cmd == CommandState::UNLOCK)
		{
			cmd = CommandState::OTHER; // Reset command after arming
		}
	}
}

void TerminalNode::handle_input_command(int input)
{
	switch (static_cast<CommandState>(input)) {
	case CommandState::UNLOCK:
		Logger::print_color(int(LogColor::yellow),"Command: UNLOCK");
		// TODO: 在此处添加解锁逻辑
		if(is_armed)
		{
			Logger::print_color(int(LogColor::green), "Vehicle is already armed!");
		}
		else
		{
			Logger::print_color(int(LogColor::green), "Arming vehicle...");
			cmd = CommandState::UNLOCK; // Set command to UNLOCK to trigger arming in publish_command
		}
		break;
	case CommandState::GO_TO_WAYPOINT:
		if(!is_armed)
		{
			Logger::print_color(int(LogColor::red), "Vehicle is not armed! Please arm the vehicle before sending waypoints.");
			cmd = CommandState::OTHER; // Reset command if not armed
			return;
		}
		if(cmd == CommandState::LAND)
		{
			Logger::print_color(int(LogColor::red), "Vehicle is landing! Cannot send waypoints during landing.");
			cmd = CommandState::OTHER; // Reset command if landing
			return;
		}
		Logger::print_color(int(LogColor::yellow),"Command: GO_TO_WAYPOINT");
		Logger::print_color(int(LogColor::yellow),"Please input target position (x y z): ");
		// 使用 std::cin 读取目标位置并做基本校验（替换原来的 scanf）
		double input_x, input_y, input_z;
		if (!(std::cin >> input_x >> input_y >> input_z)) {
			std::cin.clear();
			std::string dummy; std::getline(std::cin, dummy);
			Logger::warning("Invalid waypoint input");
			return;
		}
		target_x = input_x;
		target_y = input_y;
		target_z = input_z;
		cmd = CommandState::GO_TO_WAYPOINT;
		// TODO: 在此处添加飞向航点逻辑
		break;
	case CommandState::LAND:
		Logger::print_color(int(LogColor::yellow),"Command: LAND");
		// TODO: 在此处添加降落逻辑
		if(is_land)
		{
			Logger::print_color(int(LogColor::green), "Vehicle is already landing!");
		}
		else
		{
			Logger::print_color(int(LogColor::green), "Initiating landing sequence...");
		}
		cmd = CommandState::LAND;
		break;
	case CommandState::HOVER:
		Logger::print_color(int(LogColor::yellow),"Command: HOVER");
		if(!is_armed)
		{
			Logger::print_color(int(LogColor::red), "Vehicle is not armed! Please arm the vehicle before sending hover commands.");
			cmd = CommandState::OTHER; // Reset command if not armed
			return;
		}
		if(cmd == CommandState::LAND)
		{
			Logger::print_color(int(LogColor::red), "Vehicle is landing! Cannot send hover commands during landing.");
			cmd = CommandState::OTHER; // Reset command if landing
			return;
		}
		Logger::print_color(int(LogColor::yellow),"Setting hover at current position...");
		hover();
		cmd = CommandState::GO_TO_WAYPOINT; // Reuse GO_TO_WAYPOINT command to trigger hover at current position
		break;
	case CommandState::TAKEOFF:
		Logger::print_color(int(LogColor::yellow),"Command: TAKEOFF");
		if(is_armed)
		{
			Logger::print_color(int(LogColor::green), "Vehicle is already armed! Publishing takeoff setpoint...");
		}
		else
		{
			Logger::print_color(int(LogColor::green), "Arming vehicle for takeoff...");
		}
		cmd = CommandState::TAKEOFF;
		break;
	case CommandState::TRAJ:
		Logger::print_color(int(LogColor::yellow),"Command: TRAJ");
		if(!is_armed)
		{
			Logger::print_color(int(LogColor::red), "Vehicle is not armed! Please arm the vehicle before sending trajectory commands.");
			cmd = CommandState::OTHER; // Reset command if not armed
			return;
		}
		if(cmd == CommandState::LANDING)
		{
			Logger::print_color(int(LogColor::red), "Vehicle is landing! Cannot send trajectory commands during landing.");
			cmd = CommandState::OTHER; // Reset command if landing
		}
		if (!has_local_position_) {
			Logger::print_color(int(LogColor::red), "No local position data! Cannot generate trajectory.");
			cmd = CommandState::OTHER; // Reset command if no local position
			return;
		}
		Logger::print_color(int(LogColor::yellow), ">>>>>>>>>>>>>>>> tarj_list <<<<<<<<<<<<<<<<<");
		Logger::print_color(int(LogColor::yellow), "Please input command:");
		Logger::print_color(int(LogColor::yellow), "  0: circle trajectory");
		Logger::print_color(int(LogColor::yellow), "  1: poly5 trajectory");
		Logger::print_color(int(LogColor::yellow), "  2: lissajous trajectory");
		if (!(std::cin >> input)) {
			std::cin.clear();
			std::string dummy; std::getline(std::cin, dummy);
			Logger::warning("Invalid trajectory command input");
			return;
		}
		tarj_count = 0; // Reset trajectory count
		if(input == 0)
		{
			Logger::print_color(int(LogColor::yellow),"Please input target position (x y z): ");
			tarj_generator_.circle_params_.circle_center = Eigen::Vector3f(
				static_cast<float>(vehicle_local_position_.x) - tarj_generator_.circle_params_.circle_radius,
				static_cast<float>(vehicle_local_position_.y),
				static_cast<float>(vehicle_local_position_.z)
			); // 圆心坐标
			tarj_generator_.generate_circle_trajectory(); // 生成圆形轨迹，频率为 50 Hz
			current_traj = TarjKind::CIRCLE;
		}
		else if(input == 1)
		{
			// 使用 std::cin 读取目标位置并做基本校验（替换原来的 scanf）
			double input_x, input_y, input_z;
			double input_vx, input_vy, input_vz;
			double input_ax, input_ay, input_az;
			Logger::print_color(int(LogColor::yellow),"Please input target position (x y z): ");
			if (!(std::cin >> input_x >> input_y >> input_z)) {
				std::cin.clear();
				std::string dummy; std::getline(std::cin, dummy);
				Logger::warning("Invalid waypoint input");
				return;
			}
			Logger::print_color(int(LogColor::yellow),"Please input target velocity (vx vy vz): ");
			if (!(std::cin >> input_vx >> input_vy >> input_vz)) {
				std::cin.clear();
				std::string dummy; std::getline(std::cin, dummy);
				Logger::warning("Invalid velocity input");
				return;
			}
			Logger::print_color(int(LogColor::yellow),"Please input target acceleration (ax ay az): ");
			if (!(std::cin >> input_ax >> input_ay >> input_az)) {
				std::cin.clear();		
				std::string dummy; std::getline(std::cin, dummy);
				Logger::warning("Invalid acceleration input");
				return;
			}
			tarj_generator_.set_current_state(
				Eigen::Vector3d(vehicle_local_position_.x, vehicle_local_position_.y, vehicle_local_position_.z),
				Eigen::Vector3d(vehicle_local_position_.vx, vehicle_local_position_.vy, vehicle_local_position_.vz),
				Eigen::Vector3d(vehicle_local_position_.ax, vehicle_local_position_.ay, vehicle_local_position_.az)
			);
			tarj_generator_.set_target_state(
				Eigen::Vector3d(input_x, input_y, input_z),
				Eigen::Vector3d(input_vx, input_vy, input_vz),
				Eigen::Vector3d(input_ax, input_ay, input_az)
			);
			tarj_generator_.generate_poly5_trajectory(); // 生成五次多项式轨迹，频率为 50 Hz
			current_traj = TarjKind::POLY5;
		}
		else if(input == 2)
		{
			// 生成李萨如曲线轨迹
			tarj_generator_.figure8_params_.center = Eigen::Vector3f(
				static_cast<float>(vehicle_local_position_.x),
				static_cast<float>(vehicle_local_position_.y),
				static_cast<float>(vehicle_local_position_.z)
			); // 圆心坐标
			tarj_generator_.generate_lissajous_trajectory();
			current_traj = TarjKind::LISSAJOUS;
		}
		cmd = CommandState::TRAJ;
		break;
	case CommandState::CHANGECONTROLLER:
		Logger::print_color(int(LogColor::yellow),"Command: CHANGE_CONTROLLER");
		Logger::print_color(int(LogColor::yellow),"Please input controller type:");
		Logger::print_color(int(LogColor::yellow),"  0: PX4 built-in controller");
		Logger::print_color(int(LogColor::yellow),"  1: SO3 controller");
		int controller_input;
		if (!(std::cin >> controller_input)) {
			std::cin.clear();
			std::string dummy; std::getline(std::cin, dummy);
			Logger::warning("Invalid controller command input");
			return;
		}
		if(controller_input == 0)
		{
			current_controller = ControlerKind::PX4;
			Logger::print_color(int(LogColor::green), "Switched to PX4 built-in controller.");
		}
		else if(controller_input == 1)
		{
			current_controller = ControlerKind::SO3;
			Logger::print_color(int(LogColor::green), "Switched to SO3 controller.");
		}
		else
		{
			Logger::warning("Unknown controller type: ", controller_input);
		}
		break;
	case CommandState::PLANNER:
		Logger::print_color(int(LogColor::yellow),"Command: EGO_PLANNER");
		if(current_controller != ControlerKind::SO3)
		{
			Logger::print_color(int(LogColor::red), "Ego planner requires SO3 controller! Please switch to SO3 controller first.");
			return;
		}
		planner_init();
		cmd = CommandState::PLANNER;
		break;
	default:
		Logger::warning("Unknown command: ", input);
		break;
	}
}

void TerminalNode::publish_command()
{
	switch (cmd) {
		case CommandState::UNLOCK:
			arm_function();
			break;
		case CommandState::GO_TO_WAYPOINT:
			uav_move_control();	
			break;
		case CommandState::LAND:
			send_land_command();
			cmd = CommandState::LANDING; // Set to LANDING to prevent re-sending land command
			break;
		case CommandState::TAKEOFF:
			if(!is_armed)
			{
				arm_function();
			}
			else
			{
				target_x = vehicle_local_position_.x;
				target_y = vehicle_local_position_.y;
				target_z = default_target_z;
				target_vx = 0.0;
				target_vy = 0.0;
				target_vz = 0.0;
				target_ax = 0.0;
				target_ay = 0.0;
				target_az = 0.0;
				target_yaw = euler_angles_rad_[2]; // 保持当前航向
				cmd = CommandState::GO_TO_WAYPOINT; // Reuse GO_TO_WAYPOINT command to trigger takeoff to target altitude
			}
			break;
		case CommandState::TRAJ:
			tarj_generator_.publish_trajectory_marker(); // 发布轨迹可视化 Marker
			target_x = tarj_generator_.traj_params_.traj_params[tarj_count].target_x;
			target_y = tarj_generator_.traj_params_.traj_params[tarj_count].target_y;
			target_z = tarj_generator_.traj_params_.traj_params[tarj_count].target_z;
			target_vx = tarj_generator_.traj_params_.traj_params[tarj_count].target_vx;
			target_vy = tarj_generator_.traj_params_.traj_params[tarj_count].target_vy;
			target_vz = tarj_generator_.traj_params_.traj_params[tarj_count].target_vz;
			target_ax = tarj_generator_.traj_params_.traj_params[tarj_count].target_ax;
			target_ay = tarj_generator_.traj_params_.traj_params[tarj_count].target_ay;
			target_az = tarj_generator_.traj_params_.traj_params[tarj_count].target_az;
			target_yaw = tarj_generator_.traj_params_.traj_params[tarj_count].desired_yaw;
			if(tarj_generator_.traj_params_.is_loop)
			{
				tarj_count++;
				tarj_count = tarj_count % tarj_generator_.traj_params_.traj_params.size(); // Loop trajectory index
			}
			else
			{
				if(static_cast<std::size_t>(tarj_count) + 1 < tarj_generator_.traj_params_.traj_params.size())
				{
					tarj_count++;
				}
				else{
					target_ax = 0.0;
					target_ay = 0.0;
					target_az = 0.0;
					target_vx = 0.0;
					target_vy = 0.0;
					target_vz = 0.0;
					cmd = CommandState::GO_TO_WAYPOINT; 
					current_traj = TarjKind::OTHER; // Reset trajectory type
					Logger::print_color(int(LogColor::green), "Trajectory completed, hovering at last point.");
				}
			}
			uav_move_control();
			break;
		case CommandState::PLANNER:
			if (!is_first_planner_command) {
				hover(); // 在接收到第一个规划命令之前先悬停
				uav_move_control();
			}
			break;
		default:
			break;
	}
}

void TerminalNode::hover()
{
	target_x = vehicle_local_position_.x;
	target_y = vehicle_local_position_.y;
	target_z = vehicle_local_position_.z;
	target_ax = 0.0;
	target_ay = 0.0;
	target_az = 0.0;
	target_vx = 0.0;
	target_vy = 0.0;
	target_vz = 0.0;
}

void TerminalNode::keep_armed()
{
	// 发送一个小的 offboard 控制命令来保持解锁状态
	publish_offboard_control_mode();
	TrajectorySetpoint msg{};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg); 
}

void TerminalNode::vehicle_local_position_callback(const VehicleLocalPosition::SharedPtr msg)
{
	vehicle_local_position_ = *msg;
	has_local_position_ = true;
}

void TerminalNode::send_land_command()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
	Logger::print_color(int(LogColor::magenta), "Land command sent!");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void TerminalNode::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	if(current_controller == ControlerKind::PX4)
	{
		msg.position = true;
		msg.velocity = true;
		msg.acceleration = true;
		msg.attitude = false;
		msg.body_rate = false;
	}
	else if(current_controller == ControlerKind::SO3)
	{
		msg.position = false;
		msg.velocity = false;
		msg.acceleration = false;
		msg.attitude = true;
		msg.body_rate = false;		
	}
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void TerminalNode::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};
	msg.position[0] = static_cast<float>(target_x);
	msg.position[1] = static_cast<float>(target_y);
	msg.position[2] = static_cast<float>(target_z);

	msg.velocity[0] = static_cast<float>(target_vx);
	msg.velocity[1] = static_cast<float>(target_vy);
	msg.velocity[2] = static_cast<float>(target_vz);

	msg.acceleration[0] = static_cast<float>(target_ax);
	msg.acceleration[1] = static_cast<float>(target_ay);
	msg.acceleration[2] = static_cast<float>(target_az);

	msg.yaw = static_cast<float>(target_yaw);

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg); 
}

/**
 * @brief Send a command to Arm the vehicle
 */
void TerminalNode::arm()
{
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
  Logger::print_color(int(LogColor::magenta), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void TerminalNode::disarm()
{
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

  Logger::print_color(int(LogColor::magenta), "Disarm command send");
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
void TerminalNode::publish_vehicle_command(uint16_t command, float param1, float param2)
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

void TerminalNode::vehicle_status_callback(const VehicleStatus::SharedPtr msg)
{
	if (msg->arming_state == 2)
	{
		if(!is_armed)
		{
			Logger::print_color(int(LogColor::green), "飞控状态: [ 已解锁 ]");
			is_armed = true;
		}
	}
	else if(is_armed)
	{
		Logger::print_color(int(LogColor::red), "飞控状态: [ 已上锁 ]");
		is_armed = false;
		arm_count = 0; // Reset arm count when disarmed
		cmd = CommandState::OTHER; // Reset command when disarmed
	}
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TerminalNode>());
	rclcpp::shutdown();
	return 0;
}
