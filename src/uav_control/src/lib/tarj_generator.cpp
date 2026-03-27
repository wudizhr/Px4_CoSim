#include "uav_control/tarj_generator.hpp"
#include "cmath"

#include <algorithm>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <uav_control/frame_transforms.h>
#include <vector>

namespace uav_control
{
    void TarjGenerator::init(rclcpp::Node* nh)
    {
        // 使用 ROS2 参数声明/读取轨迹参数（带默认值）
        traj_marker_pub_ = nh->create_publisher<visualization_msgs::msg::Marker>("trajectory_marker", 10);
        default_tf_ = nh->declare_parameter<double>("default_tf", 5.0);

        const std::string prefix = "trajectory.circle";
        const std::string center_name = prefix + ".center";
        const std::string radius_name = prefix + ".radius";
        const std::string linear_vel_name = prefix + ".linear_vel";
        const std::string direction_name = prefix + ".direction";
        const std::string fixed_yaw_name = prefix + ".fixed_yaw";

        const std::vector<double> center_default{0.0, 0.0, -1.0};
        const double radius_default = 0.5;
        const double linear_vel_default = 0.5;
        const double direction_default = 1.0;
        const bool fixed_yaw_default = false;

        std::vector<double> center = center_default;
        double radius = radius_default;
        double linear_vel = linear_vel_default;
        double direction = direction_default;
        bool fixed_yaw = fixed_yaw_default;

        if (nh->has_parameter(center_name)) {
            (void)nh->get_parameter(center_name, center);
        } else {
            center = nh->declare_parameter<std::vector<double>>(center_name, center_default);
        }

        if (nh->has_parameter(radius_name)) {
            (void)nh->get_parameter(radius_name, radius);
        } else {
            radius = nh->declare_parameter<double>(radius_name, radius_default);
        }

        if (nh->has_parameter(linear_vel_name)) {
            (void)nh->get_parameter(linear_vel_name, linear_vel);
        } else {
            linear_vel = nh->declare_parameter<double>(linear_vel_name, linear_vel_default);
        }

        if (nh->has_parameter(direction_name)) {
            (void)nh->get_parameter(direction_name, direction);
        } else {
            direction = nh->declare_parameter<double>(direction_name, direction_default);
        }

        if (nh->has_parameter(fixed_yaw_name)) {
            (void)nh->get_parameter(fixed_yaw_name, fixed_yaw);
        } else {
            fixed_yaw = nh->declare_parameter<bool>(fixed_yaw_name, fixed_yaw_default);
        }

        if (center.size() != 3) {
            RCLCPP_WARN(
                nh->get_logger(),
                "%s should be a 3-element array [x,y,z]. Got size=%zu. Using default.",
                center_name.c_str(),
                center.size());
            center = center_default;
        }

        if (radius <= 1e-6) {
            RCLCPP_WARN(
                nh->get_logger(),
                "%s must be > 0. Got %.6f. Using default %.6f.",
                radius_name.c_str(),
                radius,
                radius_default);
            radius = radius_default;
        }

        if (linear_vel <= 1e-6) {
            RCLCPP_WARN(
                nh->get_logger(),
                "%s must be > 0. Got %.6f. Using default %.6f.",
                linear_vel_name.c_str(),
                linear_vel,
                linear_vel_default);
            linear_vel = linear_vel_default;
        }

        // direction: +1 CCW, -1 CW
        direction = (direction >= 0.0) ? 1.0 : -1.0;

        circle_params_.circle_center = Eigen::Vector3f(
            static_cast<float>(center[0]),
            static_cast<float>(center[1]),
            static_cast<float>(center[2]));
        circle_params_.circle_radius = static_cast<float>(radius);
        circle_params_.linear_vel = static_cast<float>(linear_vel);
        circle_params_.direction = static_cast<float>(direction);
        circle_params_.fixed_yaw = fixed_yaw;

        // figure-8(lissajous) 独立参数
        const std::string fig8_prefix = "trajectory.figure8";
        const std::string fig8_center_name = fig8_prefix + ".center";
        const std::string fig8_amp_x_name = fig8_prefix + ".amp_x";
        const std::string fig8_amp_y_name = fig8_prefix + ".amp_y";
        const std::string fig8_linear_vel_name = fig8_prefix + ".linear_vel";
        const std::string fig8_direction_name = fig8_prefix + ".direction";
        const std::string fig8_fixed_yaw_name = fig8_prefix + ".fixed_yaw";
        const std::string fig8_phase_name = fig8_prefix + ".phase";
        const std::string fig8_ratio_x_name = fig8_prefix + ".ratio_x";
        const std::string fig8_ratio_y_name = fig8_prefix + ".ratio_y";

        const std::vector<double> fig8_center_default{0.0, 0.0, -1.0};
        const double fig8_amp_x_default = 0.8;
        const double fig8_amp_y_default = 0.5;
        const double fig8_linear_vel_default = 0.5;
        const double fig8_direction_default = 1.0;
        const bool fig8_fixed_yaw_default = false;
        const double fig8_phase_default = 0.0;
        const int64_t fig8_ratio_x_default = 1;
        const int64_t fig8_ratio_y_default = 2;

        std::vector<double> fig8_center = fig8_center_default;
        double fig8_amp_x = fig8_amp_x_default;
        double fig8_amp_y = fig8_amp_y_default;
        double fig8_linear_vel = fig8_linear_vel_default;
        double fig8_direction = fig8_direction_default;
        bool fig8_fixed_yaw = fig8_fixed_yaw_default;
        double fig8_phase = fig8_phase_default;
        int64_t fig8_ratio_x = fig8_ratio_x_default;
        int64_t fig8_ratio_y = fig8_ratio_y_default;

        if (nh->has_parameter(fig8_center_name)) {
            (void)nh->get_parameter(fig8_center_name, fig8_center);
        } else {
            fig8_center = nh->declare_parameter<std::vector<double>>(fig8_center_name, fig8_center_default);
        }
        if (nh->has_parameter(fig8_amp_x_name)) {
            (void)nh->get_parameter(fig8_amp_x_name, fig8_amp_x);
        } else {
            fig8_amp_x = nh->declare_parameter<double>(fig8_amp_x_name, fig8_amp_x_default);
        }
        if (nh->has_parameter(fig8_amp_y_name)) {
            (void)nh->get_parameter(fig8_amp_y_name, fig8_amp_y);
        } else {
            fig8_amp_y = nh->declare_parameter<double>(fig8_amp_y_name, fig8_amp_y_default);
        }
        if (nh->has_parameter(fig8_linear_vel_name)) {
            (void)nh->get_parameter(fig8_linear_vel_name, fig8_linear_vel);
        } else {
            fig8_linear_vel = nh->declare_parameter<double>(fig8_linear_vel_name, fig8_linear_vel_default);
        }
        if (nh->has_parameter(fig8_direction_name)) {
            (void)nh->get_parameter(fig8_direction_name, fig8_direction);
        } else {
            fig8_direction = nh->declare_parameter<double>(fig8_direction_name, fig8_direction_default);
        }
        if (nh->has_parameter(fig8_fixed_yaw_name)) {
            (void)nh->get_parameter(fig8_fixed_yaw_name, fig8_fixed_yaw);
        } else {
            fig8_fixed_yaw = nh->declare_parameter<bool>(fig8_fixed_yaw_name, fig8_fixed_yaw_default);
        }
        if (nh->has_parameter(fig8_phase_name)) {
            (void)nh->get_parameter(fig8_phase_name, fig8_phase);
        } else {
            fig8_phase = nh->declare_parameter<double>(fig8_phase_name, fig8_phase_default);
        }
        if (nh->has_parameter(fig8_ratio_x_name)) {
            (void)nh->get_parameter(fig8_ratio_x_name, fig8_ratio_x);
        } else {
            fig8_ratio_x = nh->declare_parameter<int64_t>(fig8_ratio_x_name, fig8_ratio_x_default);
        }
        if (nh->has_parameter(fig8_ratio_y_name)) {
            (void)nh->get_parameter(fig8_ratio_y_name, fig8_ratio_y);
        } else {
            fig8_ratio_y = nh->declare_parameter<int64_t>(fig8_ratio_y_name, fig8_ratio_y_default);
        }

        if (fig8_center.size() != 3) {
            RCLCPP_WARN(
                nh->get_logger(),
                "%s should be a 3-element array [x,y,z]. Got size=%zu. Using default.",
                fig8_center_name.c_str(),
                fig8_center.size());
            fig8_center = fig8_center_default;
        }
        if (fig8_amp_x <= 1e-6 || fig8_amp_y <= 1e-6) {
            RCLCPP_WARN(
                nh->get_logger(),
                "%s and %s must be > 0. Got (%.6f, %.6f). Using defaults (%.6f, %.6f).",
                fig8_amp_x_name.c_str(),
                fig8_amp_y_name.c_str(),
                fig8_amp_x,
                fig8_amp_y,
                fig8_amp_x_default,
                fig8_amp_y_default);
            fig8_amp_x = fig8_amp_x_default;
            fig8_amp_y = fig8_amp_y_default;
        }
        if (fig8_linear_vel <= 1e-6) {
            RCLCPP_WARN(
                nh->get_logger(),
                "%s must be > 0. Got %.6f. Using default %.6f.",
                fig8_linear_vel_name.c_str(),
                fig8_linear_vel,
                fig8_linear_vel_default);
            fig8_linear_vel = fig8_linear_vel_default;
        }
        fig8_direction = (fig8_direction >= 0.0) ? 1.0 : -1.0;
        if (fig8_ratio_x <= 0 || fig8_ratio_y <= 0) {
            RCLCPP_WARN(
                nh->get_logger(),
                "%s and %s must be positive integers. Got (%ld, %ld). Using defaults (%ld, %ld).",
                fig8_ratio_x_name.c_str(),
                fig8_ratio_y_name.c_str(),
                static_cast<long>(fig8_ratio_x),
                static_cast<long>(fig8_ratio_y),
                static_cast<long>(fig8_ratio_x_default),
                static_cast<long>(fig8_ratio_y_default));
            fig8_ratio_x = fig8_ratio_x_default;
            fig8_ratio_y = fig8_ratio_y_default;
        }

        figure8_params_.center = Eigen::Vector3f(
            static_cast<float>(fig8_center[0]),
            static_cast<float>(fig8_center[1]),
            static_cast<float>(fig8_center[2]));
        figure8_params_.amp_x = static_cast<float>(fig8_amp_x);
        figure8_params_.amp_y = static_cast<float>(fig8_amp_y);
        figure8_params_.linear_vel = static_cast<float>(fig8_linear_vel);
        figure8_params_.direction = static_cast<float>(fig8_direction);
        figure8_params_.fixed_yaw = fig8_fixed_yaw;
        figure8_params_.phase = static_cast<float>(fig8_phase);
        figure8_params_.ratio_x = static_cast<int>(fig8_ratio_x);
        figure8_params_.ratio_y = static_cast<int>(fig8_ratio_y);
    }

    void TarjGenerator::generate_circle_trajectory()
    {
        traj_params_.traj_params.clear(); // 清空之前的轨迹参数
        double time_from_start = 0.0;
        double time_step = 1.0 / static_cast<double>(sample_hz_);
        double total_time = 2.0 * M_PI * circle_params_.circle_radius / circle_params_.linear_vel; // 完成一圈的时间
        int total_steps = static_cast<int>(total_time / time_step);
        for (int i = 0; i < total_steps; ++i) {
            uav_control_params params;
            float angular_vel = circle_params_.linear_vel / circle_params_.circle_radius; // 角速度 = 线速度 / 半径
            float angle = circle_params_.direction * angular_vel * time_from_start; // 当前角度 = 方向 * 角速度 * 时间

            // 计算位置
            params.target_x = circle_params_.circle_center[0] + circle_params_.circle_radius * cos(angle);
            params.target_y = circle_params_.circle_center[1] + circle_params_.circle_radius * sin(angle);
            params.target_z = circle_params_.circle_center[2];

            // 计算速度
            params.target_vx = -circle_params_.direction * circle_params_.linear_vel * sin(angle);
            params.target_vy = circle_params_.direction * circle_params_.linear_vel * cos(angle);
            params.target_vz = 0.0f;

            // 计算加速度
            params.target_ax = -circle_params_.direction * (angular_vel * circle_params_.linear_vel) * cos(angle);
            params.target_ay = -circle_params_.direction * (angular_vel * circle_params_.linear_vel) * sin(angle);
            params.target_az = 0.0f;

            if(circle_params_.fixed_yaw)
            {
                params.desired_yaw = 0;
            }else
            {
                // 让 yaw 随轨迹运动方向（切线方向）变化，而不是指向(0,0)原点。
                // 对圆轨迹来说，速度方向就是期望机头朝向：yaw = atan2(vy, vx)。
                params.desired_yaw = std::atan2(params.target_vy, params.target_vx);
            }        
            traj_params_.traj_params.push_back(params);
            time_from_start += time_step;
        }
        traj_params_.is_loop = true; // 圆轨迹需要循环执行
    }

    void TarjGenerator::generate_poly5_trajectory()
    {
        traj_params_.traj_params.clear(); // 清空之前的轨迹参数
        uav_control::Poly5Solver sx(cur_pos_.x(), cur_vel_.x(), cur_acc_.x(),
            target_pos_.x(), target_vel_.x(), target_acc_.x(), default_tf_);
        uav_control::Poly5Solver sy(cur_pos_.y(), cur_vel_.y(), cur_acc_.y(),
            target_pos_.y(), target_vel_.y(), target_acc_.y(), default_tf_);
        uav_control::Poly5Solver sz(cur_pos_.z(), cur_vel_.z(), cur_acc_.z(),
            target_pos_.z(), target_vel_.z(), target_acc_.z(), default_tf_);
        const int samples = std::max(2, static_cast<int>(default_tf_ * sample_hz_) + 1);
        cout << "Generating poly5 trajectory with " << samples << " samples over " << default_tf_ << " seconds." << endl;
        for (int i = 0; i < samples; ++i) {
            const double t = default_tf_ * static_cast<double>(i) / static_cast<double>(samples - 1);
            uav_control_params params;
            params.target_x = sx.get_position(t);
            params.target_y = sy.get_position(t);
            params.target_z = sz.get_position(t);
            params.target_vx = sx.get_velocity(t);
            params.target_vy = sy.get_velocity(t);
            params.target_vz = sz.get_velocity(t);
            params.target_ax = sx.get_acceleration(t);
            params.target_ay = sy.get_acceleration(t);
            params.target_az = sz.get_acceleration(t);
            // 让 yaw 随轨迹运动方向（切线方向）变化，而不是指向(0,0)原点。
            // 对一般轨迹来说，速度方向就是期望机头朝向：yaw = atan2(vy, vx)。
            params.desired_yaw = std::atan2(params.target_vy, params.target_vx);
            traj_params_.traj_params.push_back(params);
        }        
        traj_params_.is_loop = false; // 多项式轨迹通常不需要循环执行
    }

    void TarjGenerator::generate_lissajous_trajectory()
    {
        traj_params_.traj_params.clear(); // 清空之前的轨迹参数

        const double hz = std::max(1.0, sample_hz_);

        // 使用 figure-8 独立参数
        const double cx = static_cast<double>(figure8_params_.center.x());
        const double cy = static_cast<double>(figure8_params_.center.y());
        const double cz = static_cast<double>(figure8_params_.center.z());
        const double A = std::max(1e-3, static_cast<double>(figure8_params_.amp_x));
        const double B = std::max(1e-3, static_cast<double>(figure8_params_.amp_y));
        const double v = std::max(1e-3, static_cast<double>(figure8_params_.linear_vel));
        const double dir = (figure8_params_.direction >= 0.0f) ? 1.0 : -1.0;

        // 李萨如八字参数:
        // x = cx + A*sin(w*t + delta)
        // y = cy + B*sin(2*w*t)
        // 取 a:b = 1:2 可得到典型“8”字形
        const int a = std::max(1, figure8_params_.ratio_x);
        const int b = std::max(1, figure8_params_.ratio_y);
        const double delta = static_cast<double>(figure8_params_.phase);
        const double base_scale = std::max(A, B);
        const double omega = dir * v / std::max(1e-3, base_scale);

        // 对整数频比(a,b)，一个完整闭合周期可取 T = 2*pi/|omega|
        const double total_time = 2.0 * M_PI / std::max(1e-6, std::abs(omega));
        // 为保证曲线严格闭合：显式包含 t=0 与 t=T 两个端点
        const int total_steps = std::max(2, static_cast<int>(std::llround(total_time * hz)) + 1);

        float last_yaw = 0.0f;
        for (int i = 0; i < total_steps; ++i) {
            uav_control_params params;

            const double t = total_time * static_cast<double>(i) /
                             static_cast<double>(total_steps - 1);

            const double awt = static_cast<double>(a) * omega * t + delta;
            const double bwt = static_cast<double>(b) * omega * t;

            // 位置
            params.target_x = static_cast<float>(cx + A * std::sin(awt));
            params.target_y = static_cast<float>(cy + B * std::sin(bwt));
            params.target_z = static_cast<float>(cz);

            // 速度
            const double vx = A * static_cast<double>(a) * omega * std::cos(awt);
            const double vy = B * static_cast<double>(b) * omega * std::cos(bwt);
            params.target_vx = static_cast<float>(vx);
            params.target_vy = static_cast<float>(vy);
            params.target_vz = 0.0f;

            // 加速度
            params.target_ax = static_cast<float>(-A * std::pow(static_cast<double>(a) * omega, 2) * std::sin(awt));
            params.target_ay = static_cast<float>(-B * std::pow(static_cast<double>(b) * omega, 2) * std::sin(bwt));
            params.target_az = 0.0f;

            if (figure8_params_.fixed_yaw) {
                params.desired_yaw = 0.0f;
            } else {
                const double speed_xy = std::hypot(vx, vy);
                if (speed_xy > 1e-4) {
                    last_yaw = static_cast<float>(std::atan2(vy, vx));
                }
                params.desired_yaw = last_yaw;
            }

            traj_params_.traj_params.push_back(params);
        }

        traj_params_.is_loop = true; // 李萨如曲线通常按周期循环执行
    }

    void TarjGenerator::publish_trajectory_marker()
    {
        if (traj_params_.traj_params.empty() || !traj_marker_pub_) {
            return;
        }

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
        marker.ns = "trajectory";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;

        std::vector<double> speeds;
        speeds.reserve(traj_params_.traj_params.size());
        for (const auto &p : traj_params_.traj_params) {
            const double speed = std::sqrt(
                static_cast<double>(p.target_vx) * static_cast<double>(p.target_vx) +
                static_cast<double>(p.target_vy) * static_cast<double>(p.target_vy) +
                static_cast<double>(p.target_vz) * static_cast<double>(p.target_vz));
            speeds.push_back(speed);
        }

        const auto [min_it, max_it] = std::minmax_element(speeds.begin(), speeds.end());
        const double min_speed = *min_it;
        const double max_speed = *max_it;
        const double speed_range = std::max(1e-6, max_speed - min_speed);

        marker.points.reserve(traj_params_.traj_params.size());
        marker.colors.reserve(traj_params_.traj_params.size());
        for (size_t i = 0; i < traj_params_.traj_params.size(); ++i) {
            const auto &p = traj_params_.traj_params[i];

            const Eigen::Vector3d point_ned(
                static_cast<double>(p.target_x),
                static_cast<double>(p.target_y),
                static_cast<double>(p.target_z));
            const Eigen::Vector3d point_enu =
                uav_control::frame_transforms::ned_to_enu_local_frame(point_ned);

            geometry_msgs::msg::Point point;
            point.x = point_enu.x();
            point.y = point_enu.y();
            point.z = point_enu.z();
            marker.points.push_back(point);

            const double normalized = (speeds[i] - min_speed) / speed_range;
            std_msgs::msg::ColorRGBA color;
            color.a = 1.0f;
            color.r = static_cast<float>(normalized);
            color.g = 0.2f;
            color.b = static_cast<float>(1.0 - normalized);
            marker.colors.push_back(color);
        }

        traj_marker_pub_->publish(marker);
    }
}