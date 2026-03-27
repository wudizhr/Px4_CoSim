#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <quadrotor_msgs/msg/corrections.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <quadrotor_msgs/msg/so3_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <so3_control/SO3Control.hpp>
#include <std_msgs/msg/bool.hpp>
#include <uav_control/frame_transforms.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class SO3ControlComponent : public rclcpp::Node
{
public:
    SO3ControlComponent(const rclcpp::NodeOptions &options)
        : Node("SO3ControlComponent", options), position_cmd_updated_(false), position_cmd_init_(false), des_yaw_(0), des_yaw_dot_(0), current_yaw_(0), enable_motors_(true), // FIXME
          use_external_yaw_(false)
    {
        onInit();
    }

    void onInit(void);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    void publishSO3Command(void);
    void position_cmd_callback(const quadrotor_msgs::msg::PositionCommand::ConstPtr &cmd);
    void odom_callback(const nav_msgs::msg::Odometry::ConstPtr &odom);
    void enable_motors_callback(const std_msgs::msg::Bool::ConstPtr &msg);
    void corrections_callback(const quadrotor_msgs::msg::Corrections::ConstPtr &msg);
    void imu_callback(const sensor_msgs::msg::Imu &imu);

    SO3Control controller_;
    rclcpp::Publisher<quadrotor_msgs::msg::SO3Command>::SharedPtr so3_command_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr position_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_motors_sub_;
    rclcpp::Subscription<quadrotor_msgs::msg::Corrections>::SharedPtr corrections_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    bool position_cmd_updated_, position_cmd_init_;
    std::string frame_id_;

    Eigen::Vector3d des_pos_, des_vel_, des_acc_, kx_, kv_;
    double des_yaw_, des_yaw_dot_;
    double current_yaw_;
    bool enable_motors_;
    bool use_external_yaw_;
    double kR_[3], kOm_[3], corrections_[3];
    double init_x_, init_y_, init_z_;
};

void SO3ControlComponent::publishSO3Command(void)
{   
    // std::cout<< "pub so3 cmd!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    controller_.calculateControl(des_pos_, des_vel_, des_acc_, des_yaw_,
                                 des_yaw_dot_, kx_, kv_);

    const Eigen::Vector3d &force = controller_.getComputedForce();
    const Eigen::Quaterniond &orientation = controller_.getComputedOrientation();

    auto so3_command = std::make_shared<quadrotor_msgs::msg::SO3Command>(); // 使用智能指针

    // 设置消息内容
    so3_command->header.stamp = this->get_clock()->now(); // ROS 2 使用 get_clock() 获取当前时间
    so3_command->header.frame_id = frame_id_;
    so3_command->force.x = force(0);
    so3_command->force.y = force(1);
    so3_command->force.z = force(2);
    so3_command->orientation.x = orientation.x();
    so3_command->orientation.y = orientation.y();
    so3_command->orientation.z = orientation.z();
    so3_command->orientation.w = orientation.w();

    // 填充 kR 和 kOm
    for (int i = 0; i < 3; i++)
    {
        so3_command->k_r[i] = kR_[i];
        so3_command->k_om[i] = kOm_[i];
    }

    // 填充辅助信息
    so3_command->aux.current_yaw = current_yaw_;
    so3_command->aux.kf_correction = corrections_[0];
    so3_command->aux.angle_corrections[0] = corrections_[1];
    so3_command->aux.angle_corrections[1] = corrections_[2];
    so3_command->aux.enable_motors = enable_motors_;
    so3_command->aux.use_external_yaw = use_external_yaw_;

    // 发布消息
    so3_command_pub_->publish(*so3_command);
}

void SO3ControlComponent::position_cmd_callback(const quadrotor_msgs::msg::PositionCommand::ConstPtr &cmd)
{
    // std::cout<< "SO3ControlComponent::position_cmd_callback!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    des_pos_ = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
    des_vel_ = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
    des_acc_ = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z);

    if (cmd->kx[0] > 1e-5 || cmd->kx[1] > 1e-5 || cmd->kx[2] > 1e-5)
    {
        kx_ = Eigen::Vector3d(cmd->kx[0], cmd->kx[1], cmd->kx[2]);
    }
    if (cmd->kv[0] > 1e-5 || cmd->kv[1] > 1e-5 || cmd->kv[2] > 1e-5)
    {
        kv_ = Eigen::Vector3d(cmd->kv[0], cmd->kv[1], cmd->kv[2]);
    }

    des_yaw_ = cmd->yaw;
    des_yaw_dot_ = cmd->yaw_dot;
    position_cmd_updated_ = true;
    position_cmd_init_ = true;

    publishSO3Command();
}

void SO3ControlComponent::odom_callback(const nav_msgs::msg::Odometry::ConstPtr &odom)
{   
    // std::cout<< "SO3ControlComponent::odom_callback!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    // std::cout<< "position_cmd_init_!!!!!!!!!!!!!!!!!!!!!!!" << position_cmd_init_ << std::endl;
    // std::cout<< "position_cmd_updated_!!!!!!!!!!!!!!!!!!!!!!!" << position_cmd_updated_ << std::endl;
    const Eigen::Vector3d position_enu(odom->pose.pose.position.x,
                                       odom->pose.pose.position.y,
                                       odom->pose.pose.position.z);
    const Eigen::Vector3d velocity_enu(odom->twist.twist.linear.x,
                                       odom->twist.twist.linear.y,
                                       odom->twist.twist.linear.z);

    const Eigen::Vector3d position_ned =
        uav_control::frame_transforms::enu_to_ned_local_frame(position_enu);
    const Eigen::Vector3d velocity_ned =
        uav_control::frame_transforms::enu_to_ned_local_frame(velocity_enu);

    // 使用tf2提取yaw
    // 从消息转换为tf2四元数
    tf2::Quaternion quat;
    tf2::fromMsg(odom->pose.pose.orientation, quat);  

    const Eigen::Quaterniond q_ros_enu_flu(quat.w(), quat.x(), quat.y(), quat.z());
    const Eigen::Quaterniond q_px4_ned_frd =
        uav_control::frame_transforms::ros_to_px4_orientation(q_ros_enu_flu);
    const Eigen::Vector3d euler_angles =
        uav_control::frame_transforms::utils::quaternion::quaternion_to_euler_321(q_px4_ned_frd);
    // std::cout << "Current Yaw from Odom(deg): " << euler_angles[2] * 180.0 / M_PI << std::endl;
    current_yaw_ = euler_angles[2];
    controller_.setPosition(position_ned);
    controller_.setVelocity(velocity_ned);

    if (position_cmd_init_)
    {
        // We set position_cmd_updated_ = false and expect that the
        // position_cmd_callback would set it to true since typically a position_cmd
        // message would follow an odom message. If not, the position_cmd_callback
        // hasn't been called and we publish the so3 command ourselves
        // TODO: Fallback to hover if position_cmd hasn't been received for some
        // time
        if (!position_cmd_updated_)
            publishSO3Command();
        position_cmd_updated_ = false;
    }
    else if (init_z_ > -9999.0)
    {
        des_pos_ = Eigen::Vector3d(init_x_, init_y_, init_z_);
        des_vel_ = Eigen::Vector3d(0, 0, 0);
        des_acc_ = Eigen::Vector3d(0, 0, 0);
        publishSO3Command();
    }
}

void SO3ControlComponent::enable_motors_callback(const std_msgs::msg::Bool::ConstPtr &msg)
{
    if (msg->data)
        RCLCPP_INFO(this->get_logger(), "Enabling motors");
    else
        RCLCPP_INFO(this->get_logger(), "Disabling motors");

    enable_motors_ = msg->data;
}

void SO3ControlComponent::corrections_callback(
    const quadrotor_msgs::msg::Corrections::ConstPtr &msg)
{
    corrections_[0] = msg->kf_correction;
    corrections_[1] = msg->angle_corrections[0];
    corrections_[2] = msg->angle_corrections[1];
}

void SO3ControlComponent::imu_callback(const sensor_msgs::msg::Imu &imu)
{
    const Eigen::Vector3d acc_flu(imu.linear_acceleration.x,
                                  imu.linear_acceleration.y,
                                  imu.linear_acceleration.z);
    const Eigen::Vector3d acc_frd =
        uav_control::frame_transforms::baselink_to_aircraft_body_frame(acc_flu);
    controller_.setAcc(acc_frd);
}

void SO3ControlComponent::onInit(void)
{
    // rclcpp::Node::SharedPtr node = this->shared_from_this();
    RCLCPP_INFO(get_logger(), "start SO3ControlComponent");

    declare_parameter("quadrotor_name", "quadrotor");
    declare_parameter("mass", 2.49);

    declare_parameter("use_external_yaw", true);

    declare_parameter("gains/rot/x", 1.0);
    declare_parameter("gains/rot/y", 1.0);
    declare_parameter("gains/rot/z", 1.0);
    declare_parameter("gains/ang/x", 0.13);
    declare_parameter("gains/ang/y", 0.13);
    declare_parameter("gains/ang/z", 0.1);

    declare_parameter("gains/kx/x", 1.5);
    declare_parameter("gains/kx/y", 1.5);
    declare_parameter("gains/kx/z", 3.5);
    declare_parameter("gains/kv/x", 1.8);
    declare_parameter("gains/kv/y", 1.8);
    declare_parameter("gains/kv/z", 2.0);

    declare_parameter("corrections/z", 0.0);
    declare_parameter("corrections/r", 0.0);
    declare_parameter("corrections/p", 0.0);
    declare_parameter("so3_control/init_state_x", 0.0);
    declare_parameter("so3_control/init_state_y", 0.0);
    declare_parameter("so3_control/init_state_z", 0.0);

    std::string quadrotor_name;
    get_parameter("quadrotor_name", quadrotor_name);
    frame_id_ = "/" + quadrotor_name;

    double mass;
    get_parameter("mass", mass);
    controller_.setMass(mass);

    get_parameter("use_external_yaw", use_external_yaw_);

    get_parameter("gains/rot/x", kR_[0]);
    get_parameter("gains/rot/y", kR_[1]);
    get_parameter("gains/rot/z", kR_[2]);
    get_parameter("gains/ang/x", kOm_[0]);
    get_parameter("gains/ang/y", kOm_[1]);
    get_parameter("gains/ang/z", kOm_[2]);
    get_parameter("gains/kx/x", kx_[0]);
    get_parameter("gains/kx/y", kx_[1]);
    get_parameter("gains/kx/z", kx_[2]);
    get_parameter("gains/kv/x", kv_[0]);
    get_parameter("gains/kv/y", kv_[1]);
    get_parameter("gains/kv/z", kv_[2]);

    get_parameter("corrections/z", corrections_[0]);
    get_parameter("corrections/r", corrections_[1]);
    get_parameter("corrections/p", corrections_[2]);

    get_parameter("so3_control/init_state_x", init_x_);
    get_parameter("so3_control/init_state_y", init_y_);
    get_parameter("so3_control/init_state_z", init_z_);

    so3_command_pub_ = create_publisher<quadrotor_msgs::msg::SO3Command>("so3_cmd", 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&SO3ControlComponent::odom_callback, this, std::placeholders::_1));

    position_cmd_sub_ = create_subscription<quadrotor_msgs::msg::PositionCommand>(
        "position_cmd", 10, std::bind(&SO3ControlComponent::position_cmd_callback, this, std::placeholders::_1));

    enable_motors_sub_ = create_subscription<std_msgs::msg::Bool>(
        "motors", 2, std::bind(&SO3ControlComponent::enable_motors_callback, this, std::placeholders::_1));

    corrections_sub_ = create_subscription<quadrotor_msgs::msg::Corrections>(
        "corrections", 10, std::bind(&SO3ControlComponent::corrections_callback, this, std::placeholders::_1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu", 10, std::bind(&SO3ControlComponent::imu_callback, this, std::placeholders::_1));
}

RCLCPP_COMPONENTS_REGISTER_NODE(SO3ControlComponent)