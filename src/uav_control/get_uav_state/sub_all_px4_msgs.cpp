#include "ros_msg_utils.h"

class Px4MsgListener : public rclcpp::Node
{
public:
	explicit Px4MsgListener()
	: Node("sub_all_px4_msgs")
	, has_vehicle_attitude_(false)
	, has_battery_status_(false)
	, has_local_position_(false)
	, has_vehicle_status_(false)
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		
		// 【订阅】
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
		vehicle_land_detected_subscription_ = this->create_subscription<VehicleLandDetected>(
			"/fmu/out/vehicle_land_detected", qos,
			std::bind(&Px4MsgListener::vehicle_land_detected_callback, this, std::placeholders::_1));
		estimator_status_flag_subscription_ = this->create_subscription<EstimatorStatusFlags>(
			"/fmu/out/estimator_status_flags", qos,
			std::bind(&Px4MsgListener::estimator_status_flag_callback, this, std::placeholders::_1));
		failsafe_flags_subscription_ = this->create_subscription<FailsafeFlags>(
			"/fmu/out/failsafe_flags", qos,
			std::bind(&Px4MsgListener::failsafe_flags_callback, this, std::placeholders::_1));
		// 【定时器】 1000代表1000ms
		debug_timer = this->create_wall_timer(std::chrono::milliseconds(1000), [this](){this->timer_callback();});
	}

private:
	rclcpp::TimerBase::SharedPtr debug_timer;
	void timer_callback();
	void vehicle_attitude_callback(const VehicleAttitude::SharedPtr msg);
	void battery_status_callback(const BatteryStatus::SharedPtr msg);
	void vehicle_local_position_callback(const VehicleLocalPosition::SharedPtr msg);
	void vehicle_status_callback(const VehicleStatus::SharedPtr msg);
	void vehicle_land_detected_callback(const VehicleLandDetected::SharedPtr msg);
	void estimator_status_flag_callback(const EstimatorStatusFlags::SharedPtr msg);
	void failsafe_flags_callback(const FailsafeFlags::SharedPtr msg);
	std::string nav_state_to_string(uint8_t nav_state);
	std::string true_or_false(bool value);
	rclcpp::Subscription<VehicleAttitude>::SharedPtr vehicle_attitude_subscription_;
	rclcpp::Subscription<BatteryStatus>::SharedPtr battery_status_subscription_;
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_subscription_;
	rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_subscription_;
	rclcpp::Subscription<VehicleLandDetected>::SharedPtr vehicle_land_detected_subscription_;
	rclcpp::Subscription<EstimatorStatusFlags>::SharedPtr estimator_status_flag_subscription_;
	rclcpp::Subscription<FailsafeFlags>::SharedPtr failsafe_flags_subscription_;
	VehicleAttitude vehicle_attitude_;
	BatteryStatus battery_status_;
	VehicleLocalPosition vehicle_local_position_;
	VehicleStatus vehicle_status_;
	VehicleLandDetected vehicle_land_detected_;
	EstimatorStatusFlags estimator_status_flag_;
	FailsafeFlags failsafe_flags_;
	std::atomic<bool> has_vehicle_attitude_;
	std::atomic<bool> has_battery_status_;
	std::atomic<bool> has_local_position_;
	std::atomic<bool> has_vehicle_status_;

	Eigen::Vector3d euler_angles_rad_;
	Eigen::Quaterniond quaternion_;

	// vehicle_status_ frequency calculation
	uint64_t vehicle_status_last_timestamp_us_ = 0;
	uint32_t vehicle_status_callback_count_ = 0;
	std::chrono::steady_clock::time_point vehicle_status_last_callback_time_;
	double vehicle_status_frequency_ = 0.0;
};

void Px4MsgListener::timer_callback()
{
    Logger::print_color(int(LogColor::cyan), ">>>>>>>>>>>>>>>> sub_all_px4_msgs <<<<<<<<<<<<<<<<<");
    if (vehicle_status_.arming_state == 2)
    {
            Logger::print_color(int(LogColor::green), "飞控状态: [ 已解锁 ]");
    }
    else
    {
        Logger::print_color(int(LogColor::red), "飞控状态: [ 未解锁 ]");
    }

	if(vehicle_land_detected_.landed)
	{
		Logger::print_color(int(LogColor::red), "着陆状态: [ 未起飞 ]");
	}
	else
	{
		Logger::print_color(int(LogColor::green), "着陆状态: [ 起飞 ]");
	}
	Logger::print_color(int(LogColor::green), "飞行模式:", vehicle_status_.nav_state, "[", nav_state_to_string(vehicle_status_.nav_state), "]");
	Logger::print_color(int(LogColor::green), "电池电压:", battery_status_.voltage_v, "[V]", "电池余量:", battery_status_.remaining, "[%]");
	Logger::print_color(int(LogColor::green), "无人机位置[X Y Z]:",
						vehicle_local_position_.x,
						vehicle_local_position_.y,
						vehicle_local_position_.z,
						"[ m ]");
	Logger::print_color(int(LogColor::green), "无人机速度[X Y Z]:",
						vehicle_local_position_.vx,
						vehicle_local_position_.vy,
						vehicle_local_position_.vz,
						"[m/s]");
	Logger::print_color(int(LogColor::green), "无人机姿态[X Y Z]:",
						euler_angles_rad_(0) / M_PI * 180,
						euler_angles_rad_(1) / M_PI * 180,
						euler_angles_rad_(2) / M_PI * 180,
						"[deg]");

	Logger::print_color(int(LogColor::green), 	"EKF是否融合外部定位: EKF2.cs_ev_pos", true_or_false(estimator_status_flag_.cs_ev_pos),
												", EKF2.cs_ev_vel", true_or_false(estimator_status_flag_.cs_ev_vel),
												", EKF2.cs_ev_hgt", true_or_false(estimator_status_flag_.cs_ev_hgt),
												", EKF2.cs_ev_yaw", true_or_false(estimator_status_flag_.cs_ev_yaw));	
	Logger::print_color(int(LogColor::green), 	"EKF是否融合GNSS: EKF2.cs_gnss_pos", true_or_false(estimator_status_flag_.cs_gnss_pos),
												", EKF2.cs_gnss_vel", true_or_false(estimator_status_flag_.cs_gnss_vel),
												", EKF2.cs_gps_hgt", true_or_false(estimator_status_flag_.cs_gps_hgt),
												", EKF2.cs_gnss_yaw", true_or_false(estimator_status_flag_.cs_gnss_yaw));	

	if(failsafe_flags_.local_position_invalid)
	{
		Logger::print_color(int(LogColor::red), "Failsafe: 本地位置无效");
	}
	else
	{
		Logger::print_color(int(LogColor::green), "Failsafe: 本地位置有效");
	}

	if(has_vehicle_status_)
	{
		// Logger::print_color(int(LogColor::cyan), "vehicle_status frequency:", vehicle_status_frequency_, "[Hz]");
		// Logger::print_color(int(LogColor::green), "vehicle_status.timestamp:", vehicle_status_.timestamp, "[us]");
		// Logger::print_color(int(LogColor::green), "vehicle_status.armed_time:", vehicle_status_.armed_time, "[us]");
		// Logger::print_color(int(LogColor::green), "vehicle_status.takeoff_time:", vehicle_status_.takeoff_time, "[us]");
		// Logger::print_color(int(LogColor::green), "vehicle_status.arming_state:", vehicle_status_.arming_state);
		// Logger::print_color(int(LogColor::green), "vehicle_status.latest_arming_reason:", vehicle_status_.latest_arming_reason);
		// Logger::print_color(int(LogColor::green), "vehicle_status.latest_disarming_reason:", vehicle_status_.latest_disarming_reason);
		// Logger::print_color(int(LogColor::green), "vehicle_status.nav_state:", vehicle_status_.nav_state);
		// Logger::print_color(int(LogColor::green), "vehicle_status.vehicle_type:", vehicle_status_.vehicle_type);
		// Logger::print_color(int(LogColor::green), "vehicle_status.system_type:", vehicle_status_.system_type);
		// Logger::print_color(int(LogColor::green), "vehicle_status.system_id:", vehicle_status_.system_id);
		// Logger::print_color(int(LogColor::green), "vehicle_status.component_id:", vehicle_status_.component_id);
		// Logger::print_color(int(LogColor::green), "vehicle_status.usb_connected:", vehicle_status_.usb_connected);
		// Logger::print_color(int(LogColor::green), "vehicle_status.pre_flight_checks_pass:", vehicle_status_.pre_flight_checks_pass);
	}

}

void Px4MsgListener::estimator_status_flag_callback(const EstimatorStatusFlags::SharedPtr msg)
{
	estimator_status_flag_ = *msg;
}	

void Px4MsgListener::failsafe_flags_callback(const FailsafeFlags::SharedPtr msg)
{
	failsafe_flags_ = *msg;
}

void Px4MsgListener::vehicle_land_detected_callback(const VehicleLandDetected::SharedPtr msg)
{
	vehicle_land_detected_ = *msg;
}

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
	euler_angles_rad_ = uav_control::frame_transforms::utils::quaternion::quaternion_to_euler_321(quaternion_);
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

	// Calculate frequency
	auto current_time = std::chrono::steady_clock::now();
	if (vehicle_status_last_timestamp_us_ > 0) {
		auto time_diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
			current_time - vehicle_status_last_callback_time_).count();
		if (time_diff_ms > 0) {
			vehicle_status_frequency_ = 1000.0 / time_diff_ms;  // Convert to Hz
		}
	}
	vehicle_status_last_callback_time_ = current_time;
	vehicle_status_last_timestamp_us_ = vehicle_status_.timestamp;
	vehicle_status_callback_count_++;
}

std::string Px4MsgListener::true_or_false(bool value) 
{
	return value ? "[True]" : "[False]";
}

std::string Px4MsgListener::nav_state_to_string(uint8_t nav_state) 
{
    static const std::unordered_map<uint8_t, std::string> nav_state_map = {
        {0, "MANUAL"},               // 手动模式
        {1, "ALTCTL"},               // 高度控制模式
        {2, "POSCTL"},               // 位置控制模式
        {3, "AUTO_MISSION"},         // 自动任务模式
        {4, "AUTO_LOITER"},          // 自动悬停模式
        {5, "AUTO_RTL"},             // 自动返航模式
        {6, "POSITION_SLOW"},
        {7, "FREE5"},
        {8, "FREE4"},
        {9, "FREE3"},
        {10, "ACRO"},                // 特技模式
        {11, "FREE2"},
        {12, "DESCEND"},             // 下降模式（无位置控制）
        {13, "TERMINATION"},         // 终止模式
        {14, "OFFBOARD"},            // 外部控制模式
        {15, "STAB"},                // 稳定化模式
        {16, "FREE1"},
        {17, "AUTO_TAKEOFF"},        // 自动起飞
        {18, "AUTO_LAND"},           // 自动降落
        {19, "AUTO_FOLLOW_TARGET"},  // 自动跟随目标
        {20, "AUTO_PRECLAND"},       // 精密降落
        {21, "ORBIT"},               // 圆形轨道
        {22, "AUTO_VTOL_TAKEOFF"},   // VTOL 起飞
        {23, "EXTERNAL1"},
        {24, "EXTERNAL2"},
        {25, "EXTERNAL3"},
        {26, "EXTERNAL4"},
        {27, "EXTERNAL5"},
        {28, "EXTERNAL6"},
        {29, "EXTERNAL7"},
        {30, "EXTERNAL8"},
        {31, "MAX"}
    };

    auto it = nav_state_map.find(nav_state);
    if (it != nav_state_map.end()) {
        return it->second;
    } else {
        return "UNKNOWN"; // 如果找不到对应值，则返回未知
    }
}


int main(int argc, char *argv[])
{
    // 设置日志
    Logger::init_default();
    Logger::setPrintToFile(false);
    Logger::setFilename("~/Documents/Sunray_log.txt");

	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Px4MsgListener>());

	rclcpp::shutdown();
	return 0;
}
