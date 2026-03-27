#include <cstdio>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <uav_control/poly_solver.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

class PolyTarjPublishDemo : public rclcpp::Node
{
public:
	explicit PolyTarjPublishDemo()
	: Node("poly_tarj_publish_demo")
	{
		frame_id_ = this->declare_parameter<std::string>("frame_id", "odom");
		default_tf_ = this->declare_parameter<double>("default_tf", 5.0);
		sample_hz_ = this->declare_parameter<double>("sample_hz", 50.0);
		traj_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/poly5_trajectory", 1);
		publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(20), [this]() {
			publish_marker();
		});
        // Start input thread
        std::thread input_thread([this]() {
            while (rclcpp::ok()) {
                std::printf("Enter target position (x y z), velocity (vx vy vz), acceleration (ax ay az) and time to reach (tf): ");
                std::scanf("%lf %lf %lf %lf %lf %lf %lf %lf %lf",
                    &x, &y, &z, &vx, &vy, &vz, &ax, &ay, &az);
                build_trajectory_marker(
                    Eigen::Vector3d(x, y, z),
                    Eigen::Vector3d(vx, vy, vz),        
                    Eigen::Vector3d(ax, ay, az),
                    default_tf_);
            }
        });
            input_thread.detach();    
        }

private:
	void build_trajectory_marker(
		const Eigen::Vector3d &target_pos,
		const Eigen::Vector3d &target_vel,
		const Eigen::Vector3d &target_acc,
		double tf);

	void publish_marker()
	{
		std::lock_guard<std::mutex> lock(marker_mutex_);
		if (!has_marker_) {
			return;
		}
		traj_marker_.header.stamp = this->get_clock()->now();
		traj_marker_pub_->publish(traj_marker_);
	}

private:
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_marker_pub_;
	rclcpp::TimerBase::SharedPtr publish_timer_;

	std::mutex marker_mutex_;
	visualization_msgs::msg::Marker traj_marker_;
	bool has_marker_{false};

	Eigen::Vector3d cur_pos_ = Eigen::Vector3d::Zero();
	Eigen::Vector3d cur_vel_ = Eigen::Vector3d::Zero();
	Eigen::Vector3d cur_acc_ = Eigen::Vector3d::Zero();

    double x, y, z, vx, vy, vz, ax, ay, az;

	std::string frame_id_;
	double default_tf_{5.0};
	double sample_hz_{50.0};
};

void PolyTarjPublishDemo::build_trajectory_marker(
    const Eigen::Vector3d &target_pos,
    const Eigen::Vector3d &target_vel,
    const Eigen::Vector3d &target_acc,
    double tf)
{
    uav_control::Poly5Solver sx(cur_pos_.x(), cur_vel_.x(), cur_acc_.x(),
        target_pos.x(), target_vel.x(), target_acc.x(), tf);
    uav_control::Poly5Solver sy(cur_pos_.y(), cur_vel_.y(), cur_acc_.y(),
        target_pos.y(), target_vel.y(), target_acc.y(), tf);
    uav_control::Poly5Solver sz(cur_pos_.z(), cur_vel_.z(), cur_acc_.z(),
        target_pos.z(), target_vel.z(), target_acc.z(), tf);

    const int samples = std::max(2, static_cast<int>(tf * sample_hz_) + 1);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.ns = "poly5";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.06;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.3;
    marker.color.b = 0.2;
    marker.points.reserve(static_cast<std::size_t>(samples));

    std::vector<double> speed_samples;
    speed_samples.reserve(static_cast<std::size_t>(samples));
    marker.points.reserve(static_cast<std::size_t>(samples));
    for (int i = 0; i < samples; ++i) {
        const double t = tf * static_cast<double>(i) / static_cast<double>(samples - 1);
        geometry_msgs::msg::Point p;
        p.x = sx.get_position(t);
        p.y = sy.get_position(t);
        p.z = sz.get_position(t);
        marker.points.push_back(p);

        const double vx = sx.get_velocity(t);
        const double vy = sy.get_velocity(t);
        const double vz = sz.get_velocity(t);
        speed_samples.push_back(std::sqrt(vx * vx + vy * vy + vz * vz));
    }

    const auto [min_it, max_it] = std::minmax_element(speed_samples.begin(), speed_samples.end());
    const double min_speed = (min_it != speed_samples.end()) ? *min_it : 0.0;
    const double max_speed = (max_it != speed_samples.end()) ? *max_it : 0.0;
    const double speed_range = std::max(1e-6, max_speed - min_speed);

    constexpr std::array<double, 3> slow_color{0.2, 0.6, 1.0};
    constexpr std::array<double, 3> fast_color{1.0, 0.2, 0.2};
    marker.colors.reserve(marker.points.size());
    for (double speed : speed_samples) {
        const double normalized = (speed - min_speed) / speed_range;
        std_msgs::msg::ColorRGBA color;
        color.a = 1.0;
        color.r = slow_color[0] + (fast_color[0] - slow_color[0]) * normalized;
        color.g = slow_color[1] + (fast_color[1] - slow_color[1]) * normalized;
        color.b = slow_color[2] + (fast_color[2] - slow_color[2]) * normalized;
        marker.colors.push_back(color);
    }

    {
        std::lock_guard<std::mutex> lock(marker_mutex_);
        traj_marker_ = marker;
        has_marker_ = true;
    }

    cur_pos_ = target_pos;
    cur_vel_ = target_vel;
    cur_acc_ = target_acc;
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PolyTarjPublishDemo>());
	rclcpp::shutdown();
	return 0;
}
