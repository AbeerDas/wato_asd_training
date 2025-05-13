// #include <chrono>
// #include <memory>

// #include "costmap_node.hpp"

// CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {}

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<CostmapNode>());
//   rclcpp::shutdown();
//   return 0;
// }


#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));

  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar",  // Replace with the correct topic name
        10,       // Queue size
        std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  initializeCostmap();

}
 
// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

void CostmapNode::initializeCostmap(){
  grid_w = 100;
  grid_h = 100;
  grid_res = 0.1;

  grid_.resize(grid_h, std::vector<int>(grid_w, 0));

}

void CostmapNode::convertToGrid(double range, double angle, int &x_grid, int &y_grid){
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);

    x_grid = static_cast<int>((x + 5.0) / grid_res); // Shift x by 15
    y_grid = static_cast<int>((y + 5.0) / grid_res); // Shift y by 15

    // if (x_grid < 0 || x_grid >= grid_w || y_grid < 0 || y_grid >= grid_h) {
    //     // RCLCPP_WARN(this->get_logger(), "Point out of bounds: (%s, %s)", x, y);
    //     x_grid = -1;
    //     y_grid = -1;
    // }


}

void CostmapNode::markObstacle(int x_grid, int y_grid){
  if (x_grid >= 0 && x_grid < static_cast<int>(grid_w) && y_grid >= 0 && y_grid < static_cast<int>(grid_h)) {
        grid_[y_grid][x_grid] = 100;
  }
}


void CostmapNode::inflateObstacles() {
  double inflation_radius = 1.0;
  int inflation_radius_cells = static_cast<int>(inflation_radius / grid_res);
  int max_cost = 50; //MAX COST FOR INFLATED CELLS, CHANGE DEPENDING ON OUTCOME

  std::vector<std::vector<int>> inflated_grid = grid_;

  for (int x = 0; x < grid_w; ++x) {
        for (int y = 0; y < grid_h; ++y) {
            if (grid_[y][x] == 100) {
                for (int dx = -inflation_radius_cells; dx <= inflation_radius_cells; ++dx) {
                    for (int dy = -inflation_radius_cells; dy <= inflation_radius_cells; ++dy) {
                        int nx = x + dx;
                        int ny = y + dy;

                        
                        if (nx >= 0 && nx < grid_w && ny >= 0 && ny < grid_h) {
                            
                            double distance = std::sqrt(dx * dx + dy * dy) * grid_res; //euclidean distance

                            
                            if (distance > inflation_radius) {
                                continue;
                            }

                            // Calculate the cost for the neighboring cell
                            int cost = static_cast<int>(max_cost * (1.0 - distance / inflation_radius));

                            // Assign the cost if it's greater than the current value
                            if (cost > inflated_grid[ny][nx]) {
                                inflated_grid[ny][nx] = cost;
                            }
                        }
                    }
                }
            }
        }
    }

    grid_ = inflated_grid;

}

void CostmapNode::publishCostmap(const std_msgs::msg::Header& lidar_header){
  nav_msgs::msg::OccupancyGrid costmap_msg;

  costmap_msg.header = lidar_header;

  costmap_msg.info.resolution = grid_res; // Resolution in meters per cell
  costmap_msg.info.width = grid_w;      // Grid width in cells
  costmap_msg.info.height = grid_h;    // Grid height in cells

  costmap_msg.info.origin.position.x = -5.0; // Center x-coordinate
  costmap_msg.info.origin.position.y = -5.0; // Center y-coordinate
  // costmap_msg.info.origin.position.z = 0.0; // Flat ground
  costmap_msg.info.origin.orientation.w = 1.0; // No rotation
  

  costmap_msg.data.resize(grid_w * grid_h);
  for (int y = 0; y < grid_h; ++y) {
      for (int x = 0; x < grid_w; ++x) {
          int index = y * grid_w + x;
          costmap_msg.data[index] = grid_[y][x]; // Copy cell value
      }
  }

  costmap_pub_->publish(costmap_msg);
  // RCLCPP_INFO(this->get_logger(), "Published costmap to /costmap");

}

void CostmapNode::clearLaserCoveredArea() {
    // Reset cells in the area covered by the Lidar scan
    for (int y = 0; y < grid_h; ++y) {
        for (int x = 0; x < grid_w; ++x) {
            if (grid_[y][x] > 0) {
                grid_[y][x] = 0; // Clear occupied cells
            }
        }
    }
}


void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // RCLCPP_INFO(this->get_logger(), "LaserScan received:");
    // RCLCPP_INFO(this->get_logger(), "Angle Range [%f]", scan->angle_min);

    //clear laser covered area
    RCLCPP_INFO(this->get_logger(), "LaserScan received:");
    clearLaserCoveredArea();
    
 
    // Step 2: Convert LaserScan to grid and mark obstacles
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        if (range <= scan->range_max && range >= scan->range_min) {
            // Calculate grid coordinates
            int x_grid, y_grid;
            convertToGrid(range, angle, x_grid, y_grid);
            markObstacle(x_grid, y_grid);
        }
    }
 
    // Step 3: Inflate obstacles
    inflateObstacles();
 
    // Step 4: Publish costmap
    // RCLCPP_INFO(this->get_logger(), "Header: [%s]", scan->header.frame_id.c_str());
    publishCostmap(scan->header);
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}