#include "control_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

ControlNode::ControlNode() 
: Node("control"), control_(robot::ControlCore(this->get_logger()))
{
    // Declare and load parameters
    this->declare_parameter("lookahead_distance", 1.0);
    this->declare_parameter("goal_tolerance", 0.1);
    this->declare_parameter("linear_speed", 0.5);
    this->declare_parameter("max_angular_speed", 1.0);

    lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    linear_speed_ = this->get_parameter("linear_speed").as_double();
    max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();

    RCLCPP_INFO(this->get_logger(), "Control parameters loaded: lookahead=%.2f, tolerance=%.2f, linear_speed=%.2f, angular_speed=%.2f",
                lookahead_distance_, goal_tolerance_, linear_speed_, max_angular_speed_);

    // Initialize subscribers
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

    // Initialize publisher
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Initialize timer for control loop (10Hz)
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "Control node initialized successfully");
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    current_path_ = msg;
    RCLCPP_INFO(this->get_logger(), "Received new path with %zu waypoints", msg->poses.size());
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_odom_ = msg;
    RCLCPP_DEBUG(this->get_logger(), "Received odometry update: x=%.2f, y=%.2f",
                msg->pose.pose.position.x, msg->pose.pose.position.y);
}

void ControlNode::controlLoop() {
    if (!current_path_ || !robot_odom_) {
        RCLCPP_DEBUG(this->get_logger(), "Waiting for path and odometry data...");
        return;  // No path or odometry data available
    }

    // Find the lookahead point
    auto lookahead_point = findLookaheadPoint();
    if (!lookahead_point) {
        // No valid lookahead point found
        RCLCPP_DEBUG(this->get_logger(), "No valid lookahead point found, stopping robot");
        geometry_msgs::msg::Twist stop_cmd;
        cmd_vel_pub_->publish(stop_cmd);
        return;
    }

    // Compute and publish velocity command
    auto cmd_vel = computeVelocity(*lookahead_point);
    RCLCPP_INFO(this->get_logger(), "Publishing velocity command: linear=%.2f, angular=%.2f",
                cmd_vel.linear.x, cmd_vel.angular.z);
    cmd_vel_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
    if (current_path_->poses.empty()) {
        return std::nullopt;
    }

    const auto& robot_pos = robot_odom_->pose.pose.position;
    
    // Find the first point that is at least lookahead_distance_ away
    for (const auto& pose : current_path_->poses) {
        double distance = computeDistance(robot_pos, pose.pose.position);
        if (distance >= lookahead_distance_) {
            return pose;
        }
    }

    // If no point is far enough, return the last point if we're not too close
    if (computeDistance(robot_pos, current_path_->poses.back().pose.position) > goal_tolerance_) {
        return current_path_->poses.back();
    }

    return std::nullopt;
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
    geometry_msgs::msg::Twist cmd_vel;
    
    // Get robot's current position and orientation
    const auto& robot_pos = robot_odom_->pose.pose.position;
    double robot_yaw = extractYaw(robot_odom_->pose.pose.orientation);
    
    // Calculate angle to target point in robot's frame
    double dx = target.pose.position.x - robot_pos.x;
    double dy = target.pose.position.y - robot_pos.y;
    double target_angle = std::atan2(dy, dx);
    
    // Calculate steering angle (error between current heading and target)
    double steering_angle = target_angle - robot_yaw;
    
    // Normalize steering angle to [-pi, pi]
    while (steering_angle > M_PI) steering_angle -= 2 * M_PI;
    while (steering_angle < -M_PI) steering_angle += 2 * M_PI;
    
    // Set constant linear velocity and proportional angular velocity
    cmd_vel.linear.x = linear_speed_;
    cmd_vel.angular.z = std::clamp(steering_angle, -max_angular_speed_, max_angular_speed_);
    
    return cmd_vel;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
