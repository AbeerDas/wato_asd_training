#ifndef ODOMETRY_SPOOF_HPP_
#define ODOMETRY_SPOOF_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Quaternion.h"

class OdometrySpoofNode : public rclcpp::Node {
  public:
    OdometrySpoofNode();

  private:
    void timerCallback();

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool has_last_transform_ = false;
    rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
    tf2::Vector3 last_position_;
    tf2::Quaternion last_orientation_;
};

#endif 
