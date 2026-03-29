#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <cmath>

class OdomVisualizationNode : public rclcpp::Node
{
public:
    OdomVisualizationNode() : Node("odom_visualization")
    {
        // Declare parameters
        this->declare_parameter("mesh_resource", "package://odom_visualization/meshes/yunque.dae");
        this->declare_parameter("color_r", 1.0);
        this->declare_parameter("color_g", 0.0);
        this->declare_parameter("color_b", 0.0);
        this->declare_parameter("color_a", 1.0);
        this->declare_parameter("robot_scale", 2.0);
        this->declare_parameter("frame_id", "world");
        this->declare_parameter("drone_id", 0);
        this->declare_parameter("quadrotor_name", "quadrotor");
        this->declare_parameter("is_frd", false);
        this->declare_parameter("trajectory_window_size", 300);

        // Get parameters
        mesh_resource_ = this->get_parameter("mesh_resource").as_string();
        color_r_ = this->get_parameter("color_r").as_double();
        color_g_ = this->get_parameter("color_g").as_double();
        color_b_ = this->get_parameter("color_b").as_double();
        color_a_ = this->get_parameter("color_a").as_double();
        scale_ = this->get_parameter("robot_scale").as_double();
        frame_id_ = this->get_parameter("frame_id").as_string();
        drone_id_ = this->get_parameter("drone_id").as_int();
        quad_name_ = this->get_parameter("quadrotor_name").as_string();
        is_frd_ = this->get_parameter("is_frd").as_bool();
        trajectory_window_size_ = static_cast<size_t>(
            this->get_parameter("trajectory_window_size").as_int());
        if (trajectory_window_size_ == 0) {
            trajectory_window_size_ = 1;
        }

        // Publishers
        mesh_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("robot", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
        traj_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("trajectory", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
        marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);

        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&OdomVisualizationNode::odom_callback, this, std::placeholders::_1));
        
        cmd_sub_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
            "cmd", 10, std::bind(&OdomVisualizationNode::cmd_callback, this, std::placeholders::_1));

        // TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Initialize path
        path_.header.frame_id = frame_id_;

        // Initialize trajectory marker for fixed-size sliding window publishing
        traj_marker_.header.frame_id = frame_id_;
        traj_marker_.ns = "trajectory";
        traj_marker_.id = drone_id_;
        traj_marker_.type = visualization_msgs::msg::Marker::LINE_LIST;
        traj_marker_.action = visualization_msgs::msg::Marker::ADD;
        traj_marker_.pose.orientation.w = 1.0;
        traj_marker_.scale.x = 0.05;
        
        RCLCPP_INFO(this->get_logger(), "Odom visualization node started for drone %d", drone_id_);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        nav_msgs::msg::Odometry odom = *msg;
        // if (is_frd_) {
        //     convert_ned_frd_to_enu_flu(odom);
        // }

        // Update path
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = odom.header;
        pose_stamped.pose = odom.pose.pose;
        path_.poses.push_back(pose_stamped);
        
        // Limit path length
        if (path_.poses.size() > 1000) {
            path_.poses.erase(path_.poses.begin());
        }
        
        path_.header.stamp = odom.header.stamp;
        path_pub_->publish(path_);

        // Publish pose
        pose_pub_->publish(pose_stamped);

        // Publish 3D mesh model
        publish_mesh(odom);

        // Publish trajectory line
        publish_trajectory(odom);

        // Publish TF
        if (is_frd_) {
            publish_tf(odom);
        }
    }

    void convert_ned_frd_to_enu_flu(nav_msgs::msg::Odometry & odom)
    {
        // Position: NED -> ENU
        const double x_ned = odom.pose.pose.position.x;
        const double y_ned = odom.pose.pose.position.y;
        const double z_ned = odom.pose.pose.position.z;
        odom.pose.pose.position.x = y_ned;
        odom.pose.pose.position.y = x_ned;
        odom.pose.pose.position.z = -z_ned;

        // Orientation: R_enu_flu = R_enu_ned * R_ned_frd * R_frd_flu
        tf2::Quaternion q_ned_frd;
        tf2::fromMsg(odom.pose.pose.orientation, q_ned_frd);

        tf2::Quaternion q_enu_ned(
            std::sqrt(0.5), std::sqrt(0.5), 0.0, 0.0);  // swap x/y, flip z
        tf2::Quaternion q_frd_flu(1.0, 0.0, 0.0, 0.0);  // 180 deg about x
        tf2::Quaternion q_enu_flu = q_enu_ned * q_ned_frd * q_frd_flu;
        q_enu_flu.normalize();
        odom.pose.pose.orientation = tf2::toMsg(q_enu_flu);

        // Twist (usually in body frame): FRD -> FLU
        odom.twist.twist.linear.y = -odom.twist.twist.linear.y;
        odom.twist.twist.linear.z = -odom.twist.twist.linear.z;
        odom.twist.twist.angular.y = -odom.twist.twist.angular.y;
        odom.twist.twist.angular.z = -odom.twist.twist.angular.z;
    }

    void cmd_callback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg)
    {
        // Store command for visualization
        last_cmd_ = *msg;
        has_cmd_ = true;
    }

    void publish_mesh(const nav_msgs::msg::Odometry & msg)
    {
        visualization_msgs::msg::Marker mesh;
        mesh.header.frame_id = frame_id_;
        mesh.header.stamp = msg.header.stamp;
        mesh.ns = "mesh";
        mesh.id = drone_id_;
        mesh.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        mesh.action = visualization_msgs::msg::Marker::ADD;
        
        mesh.pose = msg.pose.pose;
        
        mesh.scale.x = scale_;
        mesh.scale.y = scale_;
        mesh.scale.z = scale_;
        
        mesh.color.r = color_r_;
        mesh.color.g = color_g_;
        mesh.color.b = color_b_;
        mesh.color.a = color_a_;
        
        mesh.mesh_resource = mesh_resource_;
        
        mesh_pub_->publish(mesh);
    }

    void publish_trajectory(const nav_msgs::msg::Odometry & msg)
    {
        geometry_msgs::msg::Point current_point;
        current_point.x = msg.pose.pose.position.x;
        current_point.y = msg.pose.pose.position.y;
        current_point.z = msg.pose.pose.position.z;

        if (!has_last_traj_point_) {
            last_traj_point_ = current_point;
            has_last_traj_point_ = true;
            return;
        }

        // Add a new segment [last_point, current_point]
        traj_marker_.header.stamp = msg.header.stamp;
        traj_marker_.points.push_back(last_traj_point_);
        traj_marker_.points.push_back(current_point);

        std_msgs::msg::ColorRGBA color;
        color.r = 0.0; color.g = 1.0; color.b = 0.0; color.a = 0.8;
        traj_marker_.colors.push_back(color);
        traj_marker_.colors.push_back(color);

        // Keep only the latest fixed-size window of segments.
        // One segment in LINE_LIST uses 2 points and 2 colors.
        const size_t max_points = trajectory_window_size_ * 2;
        while (traj_marker_.points.size() > max_points) {
            traj_marker_.points.erase(traj_marker_.points.begin(), traj_marker_.points.begin() + 2);
            if (traj_marker_.colors.size() >= 2) {
                traj_marker_.colors.erase(traj_marker_.colors.begin(), traj_marker_.colors.begin() + 2);
            }
        }

        traj_pub_->publish(traj_marker_);
        last_traj_point_ = current_point;
    }

    void publish_tf(const nav_msgs::msg::Odometry & msg)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = msg.header.stamp;
        transform.header.frame_id = frame_id_;
        transform.child_frame_id = quad_name_ + "/base_link";
        // std::cout << "Publishing TF: " << transform.header.frame_id << " -> " << transform.child_frame_id << std::endl;
        transform.transform.translation.x = msg.pose.pose.position.x;
        transform.transform.translation.y = msg.pose.pose.position.y;
        transform.transform.translation.z = msg.pose.pose.position.z;
        transform.transform.rotation = msg.pose.pose.orientation;
        
        tf_broadcaster_->sendTransform(transform);
    }

    // Member variables
    std::string mesh_resource_;
    double color_r_, color_g_, color_b_, color_a_;
    double scale_;
    std::string frame_id_;
    int drone_id_;
    std::string quad_name_;
    bool is_frd_ = false;
    size_t trajectory_window_size_ = 300;
    
    nav_msgs::msg::Path path_;
    visualization_msgs::msg::Marker traj_marker_;
    geometry_msgs::msg::Point last_traj_point_;
    bool has_last_traj_point_ = false;
    quadrotor_msgs::msg::PositionCommand last_cmd_;
    bool has_cmd_ = false;

    // ROS 2 objects
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mesh_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_sub_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomVisualizationNode>());
    rclcpp::shutdown();
    return 0;
}
