#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "map_memory_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    robot::MapMemoryCore map_memory_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::OccupancyGrid global_map_;


    double last_x, last_y;
    double robot_x;  
    double robot_y;   
    double robot_theta_;
    const double distance_threshold;
    bool costmap_updated_ = false;

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    void updateMap();

    void integrateCostmap(nav_msgs::msg::OccupancyGrid cMap, double x, double y, double theta);

    double quaternionToYaw(double x, double y, double z, double w);
    bool robotFrameToMapFrame(double rx, double ry, int& mx, int& my);

    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool should_update_map_ = false;

};

#endif 