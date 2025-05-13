#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "planner_core.hpp"

// Supporting structures for A* implementation
struct CellIndex {
    int x;
    int y;
    
    CellIndex(int xx, int yy) : x(xx), y(yy) {}
    CellIndex() : x(0), y(0) {}
    
    bool operator==(const CellIndex &other) const {
        return (x == other.x && y == other.y);
    }
};

struct CellIndexHash {
    std::size_t operator()(const CellIndex &idx) const {
        return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
    }
};

struct AStarNode {
    CellIndex index;
    double f_score;
    
    AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF {
    bool operator()(const AStarNode &a, const AStarNode &b) {
        return a.f_score > b.f_score;
    }
};

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    void goalPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void planPath();
    
    std::vector<CellIndex> findPathAStar(const CellIndex &start, const CellIndex &goal);
    double heuristic(const CellIndex &a, const CellIndex &b);
    bool isValidCell(const CellIndex &cell);
    std::vector<CellIndex> getNeighbors(const CellIndex &cell);
    bool worldToGrid(double x, double y, int &grid_x, int &grid_y);
    void gridToWorld(int grid_x, int grid_y, double &x, double &y);
    
    robot::PlannerCore planner_;
    
    // ROS subscribers and publishers
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    
    // Store latest messages
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    nav_msgs::msg::Odometry::SharedPtr current_odom_;
    geometry_msgs::msg::PointStamped::SharedPtr current_goal_;
    
    // Parameters
    double obstacle_threshold_;
    double planning_timeout_;
};

#endif 
