#include "map_memory_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"


MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())), 
  last_x(0.0), last_y(0.0), distance_threshold(1.0)

{


  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));


  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);


  timer_ = this->create_wall_timer(
            std::chrono::milliseconds(3000), std::bind(&MapMemoryNode::updateMap, this));



  global_map_.info.resolution = 0.1;  // Example resolution
  global_map_.info.width = 100;     // Example width
  global_map_.info.height = 100;    // Example height
  global_map_.info.origin.position.x = -15.0;  // Centered at (-50, -50)
  global_map_.info.origin.position.y = -15.0;
  global_map_.info.origin.orientation.w = 1.0;

  global_map_.data.resize(global_map_.info.width * global_map_.info.height, 0); //unknown


}


// Callback for costmap updates
void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    // Store the latest costmap
    latest_costmap_ = *msg;
    costmap_updated_ = true;
}

// Callback for odometry updates
void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_x = msg->pose.pose.position.x;
    robot_y = msg->pose.pose.position.y;


    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    robot_theta_ = quaternionToYaw(qx, qy, qz, qw);

    // Compute distance traveled
    double distance = std::sqrt(std::pow(robot_x - last_x, 2) + std::pow(robot_y - last_y, 2));
    if (distance >= distance_threshold) {
        last_x = robot_x;
        last_y = robot_y;
        should_update_map_ = true;
    }
}

double MapMemoryNode::quaternionToYaw(double x, double y, double z, double w)
{
  tf2::Quaternion q(x, y, z, w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}


// Timer-based map update
void MapMemoryNode::updateMap() {
    if (should_update_map_ && costmap_updated_) {
        integrateCostmap(latest_costmap_, robot_x, robot_y, robot_theta_);
        global_map_.header.stamp = this->now();
        global_map_.header.frame_id = "sim_world";
        map_pub_->publish(global_map_);
        should_update_map_ = false;
    }
}

// Integrate the latest costmap into the global map
void MapMemoryNode::integrateCostmap(nav_msgs::msg::OccupancyGrid cMap, double rx, double ry, double theta) {
    // Transform and merge the latest costmap into the global map
    // (Implementation would handle grid alignment and merging logic)

    if (cMap.data.empty()) { 
        RCLCPP_WARN(this->get_logger(), "Latest costmap is empty. Skipping integration.");
        return;
    }

    int costmap_width = cMap.info.width;
    int costmap_height = cMap.info.height;
    double costmap_res = cMap.info.resolution;
    double cmap_origin_x = cMap.info.origin.position.x;
    double cmap_origin_y = cMap.info.origin.position.y;


    // Iterate over the costmap cells
    for (int y = 0; y < costmap_height; ++y) {
        for (int x = 0; x < costmap_width; ++x) {
              int costmap_index = y * costmap_width + x;
              int8_t cmapVal = cMap.data[costmap_index];
              if(cmapVal < 0){
                continue;
              }
              double lx = cmap_origin_x + (x + 0.5) * costmap_res;
              double ly = cmap_origin_y + (y + 0.5) * costmap_res;

              double cos_t = std::cos(theta);
              double sin_t = std::sin(theta);
              double wx = rx + (lx * cos_t - ly * sin_t);
              double wy = ry + (lx * sin_t + ly * cos_t);

              int gx, gy;
              if (!robotFrameToMapFrame(wx, wy, gx, gy)) {
                continue;
              }

              int8_t &global_val = global_map_.data[gy * global_map_.info.width + gx];

              int global_cost = (global_val < 0) ? 0 : global_val; 
              int local_cost = static_cast<int>(cmapVal);

              int max_cost = std::max(global_cost, local_cost);

              global_val = static_cast<int8_t>(max_cost);


            int global_x = static_cast<int>(cMap.info.origin.position.x / global_map_.info.resolution) + x;
            int global_y = static_cast<int>(cMap.info.origin.position.y / global_map_.info.resolution) + y;

            if (global_x < 0 || global_y < 0 || global_x >= global_map_.info.width || global_y >= global_map_.info.height) {
                continue;  // Skip out-of-bounds cells
            }

            int global_index = global_y * global_map_.info.width + global_x;

            // Merge the costmap value into the global map
            if (cMap.data[costmap_index] > 0) {  // Ignore unknown cells
                global_map_.data[global_index] = cMap.data[costmap_index];
                RCLCPP_INFO(this->get_logger(), "[%f]", cMap.data[costmap_index]);

            }
        }
    }

    costmap_updated_ = false;  // Reset costmap update flag
    RCLCPP_INFO(this->get_logger(), "Integrated latest costmap into the global map.");

}

bool MapMemoryNode::robotFrameToMapFrame(double rx, double ry, int& mx, int& my) {
  double origin_x = global_map_.info.origin.position.x;
  double origin_y = global_map_.info.origin.position.y;

  // offset from origin
  if (rx < origin_x || ry < origin_y) {
    return false;
  }

  my = static_cast<int>((ry - origin_y) / global_map_.info.resolution);
  mx = static_cast<int>((rx - origin_x) / global_map_.info.resolution);

  if (mx < 0 || mx >= static_cast<int>(global_map_.info.width) ||
      my < 0 || my >= static_cast<int>(global_map_.info.height))
  {
    return false;
  }
  return true;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}