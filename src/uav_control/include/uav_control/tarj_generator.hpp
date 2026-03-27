#pragma once

#include "ros_msg_utils.h"
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <uav_control/poly_solver.hpp>

namespace uav_control {
    struct Circle_Params
    {
        Eigen::Vector3f circle_center;  // 圆心坐标
        float circle_radius;            // 半径
        float linear_vel;               // 线速度
        float direction;                // direction = 1 for CCW 逆时针, direction = -1 for CW 顺时针
        bool fixed_yaw;                 // 是否固定yaw角：true代表yaw角恒定为0，false代表yaw角跟随轨迹移动
    };
    struct uav_control_params
    {
        float target_x = 0.0f;
        float target_y = 0.0f;
        float target_z = 0.0f;
        float target_vx = 0.0f;
        float target_vy = 0.0f;
        float target_vz = 0.0f;
        float target_ax = 0.0f;
        float target_ay = 0.0f;
        float target_az = 0.0f;

        float desired_yaw = 0.0f; // 期望的yaw角，单位为弧度
    };
    struct TARJ_PARAMS
    {
        std::vector<uav_control_params> traj_params; // 预生成的轨迹参数列表，可以根据需要添加更多轨迹类型和参数
        bool is_loop = false; // 是否循环执行轨迹
    };
    struct FigureEight_Params
    {
        Eigen::Vector3f center;        // 8字形中心 [x,y,z]；修改后会整体平移轨迹
        float amp_x;                   // X轴振幅；增大后“8”在左右方向更宽
        float amp_y;                   // Y轴振幅；增大后“8”在前后方向更高
        float linear_vel;              // 基准线速度；增大后整条轨迹飞行更快（角频率增大）
        float direction;               // 方向：+1 逆时针时间推进，-1 顺时针时间推进（轨迹遍历方向反转）
        bool fixed_yaw;                // true: yaw 固定为0；false: yaw 跟随速度切线方向
        float phase;                   // X通道相位偏移(弧度)；改变“8”的起始点和交叉形态
        int ratio_x;                   // X通道频比 a；与 ratio_y 的比例决定曲线拓扑
        int ratio_y;                   // Y通道频比 b；典型8字建议 a:b = 1:2
    };
    class TarjGenerator
    {
        public:
            TarjGenerator() = default;
            ~TarjGenerator() = default;
            void init(rclcpp::Node* nh);
            void set_current_state(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Vector3d& acc)
            {
                cur_pos_ = pos;
                cur_vel_ = vel;
                cur_acc_ = acc;
            }
            void set_target_state(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Vector3d& acc)
            {
                target_pos_ = pos;
                target_vel_ = vel;
                target_acc_ = acc;
            }
            void generate_circle_trajectory();
            void generate_poly5_trajectory(); 
            void generate_lissajous_trajectory(); // 可以根据需要添加更多轨迹生成函数
            void publish_trajectory_marker();
            void set_sample_hz(double hz)
            {
                sample_hz_ = std::max(1.0, hz);
            }

        public:
            double default_tf_{5.0};
	        double sample_hz_{50.0};

            Eigen::Vector3d cur_pos_ = Eigen::Vector3d::Zero();
            Eigen::Vector3d cur_vel_ = Eigen::Vector3d::Zero();
            Eigen::Vector3d cur_acc_ = Eigen::Vector3d::Zero();
            Eigen::Vector3d target_pos_ = Eigen::Vector3d::Zero();
            Eigen::Vector3d target_vel_ = Eigen::Vector3d::Zero();
            Eigen::Vector3d target_acc_ = Eigen::Vector3d::Zero();

            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_marker_pub_;
            Circle_Params circle_params_;
            FigureEight_Params figure8_params_;
            TARJ_PARAMS traj_params_; // 预生成的轨迹参数列表，可以根据需要添加更多轨迹类型和参数
    };
}

