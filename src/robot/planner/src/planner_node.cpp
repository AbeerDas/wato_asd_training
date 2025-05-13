#include "planner_node.hpp"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>

PlannerNode::PlannerNode() 
: Node("planner"), planner_(robot::PlannerCore(this->get_logger())),
  obstacle_threshold_(50), planning_timeout_(5.0)
{
    // Initialize subscribers
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/goal_point", 10, std::bind(&PlannerNode::goalPointCallback, this, std::placeholders::_1));
    
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

    // Initialize publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
}

void PlannerNode::goalPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    current_goal_ = msg;
    RCLCPP_INFO(this->get_logger(), "Received goal point at: x: %f, y: %f, z: %f",
                msg->point.x, msg->point.y, msg->point.z);
    planPath();
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    current_map_ = msg;
    if (current_goal_) {
        planPath();
    }
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_odom_ = msg;
}

void PlannerNode::planPath() {
    if (!current_map_ || !current_odom_ || !current_goal_) {
        RCLCPP_WARN(this->get_logger(), "Missing required data for planning");
        return;
    }

    // Convert start and goal positions to grid coordinates
    int start_x, start_y, goal_x, goal_y;
    if (!worldToGrid(current_odom_->pose.pose.position.x, 
                    current_odom_->pose.pose.position.y, 
                    start_x, start_y) ||
        !worldToGrid(current_goal_->point.x, 
                    current_goal_->point.y, 
                    goal_x, goal_y)) {
        RCLCPP_ERROR(this->get_logger(), "Start or goal position out of map bounds");
        return;
    }

    // Find path using A*
    auto path_indices = findPathAStar(CellIndex(start_x, start_y), 
                                    CellIndex(goal_x, goal_y));

    if (path_indices.empty()) {
        RCLCPP_WARN(this->get_logger(), "No valid path found");
        return;
    }

    // Convert path to ROS message
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "sim_world";
    path_msg.header.stamp = this->now();

    for (const auto& idx : path_indices) {
        geometry_msgs::msg::PoseStamped pose;
        double world_x, world_y;
        gridToWorld(idx.x, idx.y, world_x, world_y);
        
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;  // No rotation
        
        path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);
    RCLCPP_INFO(this->get_logger(), "Published path with %zu waypoints", path_msg.poses.size());
}

std::vector<CellIndex> PlannerNode::findPathAStar(const CellIndex &start, const CellIndex &goal) {
    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
    std::unordered_map<CellIndex, double, CellIndexHash> g_score;
    std::unordered_set<CellIndex, CellIndexHash> closed_set;

    // Initialize start node
    open_set.push(AStarNode(start, heuristic(start, goal)));
    g_score[start] = 0;

    auto start_time = this->now();

    while (!open_set.empty()) {
        // Check timeout
        if ((this->now() - start_time).seconds() > planning_timeout_) {
            RCLCPP_WARN(this->get_logger(), "Path planning timed out");
            return std::vector<CellIndex>();
        }

        auto current = open_set.top().index;
        open_set.pop();

        if (current == goal) {
            // Reconstruct path
            std::vector<CellIndex> path;
            while (current == start) {
                path.push_back(current);
                current = came_from[current];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }

        if (closed_set.find(current) != closed_set.end()) {
            continue;
        }
        closed_set.insert(current);

        // Check neighbors
        for (const auto& neighbor : getNeighbors(current)) {
            if (closed_set.find(neighbor) != closed_set.end()) {
                continue;
            }

            double tentative_g_score = g_score[current] + 1.0;  // Assuming grid cost of 1
            
            if (g_score.find(neighbor) == g_score.end() || 
                tentative_g_score < g_score[neighbor]) {
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g_score;
                double f_score = tentative_g_score + heuristic(neighbor, goal);
                open_set.push(AStarNode(neighbor, f_score));
            }
        }
    }

    return std::vector<CellIndex>();  // No path found
}

double PlannerNode::heuristic(const CellIndex &a, const CellIndex &b) {
    // Manhattan distance
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

bool PlannerNode::isValidCell(const CellIndex &cell) {
    if (!current_map_) return false;

    // Check bounds
    if (cell.x < 0 || cell.x >= static_cast<int>(current_map_->info.width) ||
        cell.y < 0 || cell.y >= static_cast<int>(current_map_->info.height)) {
        return false;
    }

    // Check if cell is obstacle-free
    int index = cell.y * current_map_->info.width + cell.x;
    return current_map_->data[index] < obstacle_threshold_;
}

std::vector<CellIndex> PlannerNode::getNeighbors(const CellIndex &cell) {
    std::vector<CellIndex> neighbors;
    std::vector<std::pair<int, int>> offsets = {
        {-1, 0}, {1, 0}, {0, -1}, {0, 1}  // 4-connected grid
    };

    for (const auto& [dx, dy] : offsets) {
        CellIndex neighbor(cell.x + dx, cell.y + dy);
        if (isValidCell(neighbor)) {
            neighbors.push_back(neighbor);
        }
    }

    return neighbors;
}

bool PlannerNode::worldToGrid(double x, double y, int &grid_x, int &grid_y) {
    if (!current_map_) return false;

    grid_x = static_cast<int>((x - current_map_->info.origin.position.x) / 
                             current_map_->info.resolution);
    grid_y = static_cast<int>((y - current_map_->info.origin.position.y) / 
                             current_map_->info.resolution);

    return (grid_x >= 0 && grid_x < static_cast<int>(current_map_->info.width) &&
            grid_y >= 0 && grid_y < static_cast<int>(current_map_->info.height));
}

void PlannerNode::gridToWorld(int grid_x, int grid_y, double &x, double &y) {
    if (!current_map_) return;

    x = current_map_->info.origin.position.x + (grid_x + 0.5) * current_map_->info.resolution;
    y = current_map_->info.origin.position.y + (grid_y + 0.5) * current_map_->info.resolution;
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}
