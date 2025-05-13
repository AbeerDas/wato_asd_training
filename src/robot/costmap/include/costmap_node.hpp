#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_core.hpp"

 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void publishMessage();
 
  private:
    void initializeCostmap();
    void convertToGrid(double range, double angle, int &x_grid, int &y_grid);
    void markObstacle(int x_grid, int y_grid);
    void inflateObstacles();
    void publishCostmap(const std_msgs::msg::Header &header);
    void clearLaserCoveredArea();

    robot::CostmapCore costmap_;
    
    // Member variables moved inside private section
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

    double grid_res;
    int grid_w;
    int grid_h;   
    std::vector<std::vector<int>> grid_;
};
 
#endif 