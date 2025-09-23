#include "costmap_node.hpp"

CostmapNode::CostmapNode() 
: Node("costmap"), 
  costmap_(robot::CostmapCore(this->get_logger())) 
{
  // declare ROS parameters with defaults
  this->declare_parameter("inflation_radius", INFLATION_RADIUS);
  this->declare_parameter("max_cost", MAX_COST);
  this->declare_parameter("resolution", RESOLUTION);
  this->declare_parameter("width", WIDTH);
  this->declare_parameter("height", HEIGHT);

  // debug/test string publisher
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&CostmapNode::publishMessage, this));

  // actual costmap publisher
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  // subscribe to LIDAR topic
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10,
    std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));
}

// -----------------------------------------------------
// Utility Functions
// -----------------------------------------------------

bool CostmapNode::isValidGridCell(int gx, int gy) const {
  return gx >= 0 && gx < WIDTH && gy >= 0 && gy < HEIGHT;
}

void CostmapNode::toGridCoordinates(double wx, double wy, int &gx, int &gy) const {
  // world origin at map center
  const double origin_x = -(WIDTH * RESOLUTION) / 2.0;
  const double origin_y = -(HEIGHT * RESOLUTION) / 2.0;

  gx = static_cast<int>((wx - origin_x) / RESOLUTION);
  gy = static_cast<int>((wy - origin_y) / RESOLUTION);
}

void CostmapNode::setOccupied(std::vector<std::vector<int8_t>> &grid, int gx, int gy) const {
  if (isValidGridCell(gx, gy)) {
    grid[gx][gy] = MAX_COST;
  }
}

void CostmapNode::inflateCell(std::vector<std::vector<int8_t>> &grid, int gx, int gy) const {
  const int inflation_steps = static_cast<int>(INFLATION_RADIUS / RESOLUTION);

  for (int dx = -inflation_steps; dx <= inflation_steps; ++dx) {
    for (int dy = -inflation_steps; dy <= inflation_steps; ++dy) {
      const int nx = gx + dx;
      const int ny = gy + dy;

      if (!isValidGridCell(nx, ny)) continue;

      const double d = std::hypot(dx * RESOLUTION, dy * RESOLUTION);
      if (d > INFLATION_RADIUS) continue;

      const int val = static_cast<int>(MAX_COST * (1.0 - std::min(1.0, d / INFLATION_RADIUS)));
      grid[nx][ny] = std::max(static_cast<int>(grid[nx][ny]), val);
    }
  }
}

// -----------------------------------------------------
// ROS Callbacks
// -----------------------------------------------------

void CostmapNode::publishMessage() {
  std_msgs::msg::String msg;
  msg.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
  string_pub_->publish(msg);
}

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // Initialize empty costmap
  std::vector<std::vector<int8_t>> temp_map(WIDTH, std::vector<int8_t>(HEIGHT, 0));

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    const double r = scan->ranges[i];
    if (r < scan->range_min || r > scan->range_max || std::isnan(r)) {
      continue;
    }

    const double theta = scan->angle_min + i * scan->angle_increment;
    const double wx = r * std::cos(theta);
    const double wy = r * std::sin(theta);

    int gx, gy;
    toGridCoordinates(wx, wy, gx, gy);

    if (!isValidGridCell(gx, gy)) continue;

    setOccupied(temp_map, gx, gy);
    inflateCell(temp_map, gx, gy);
  }

  publishCostmap(temp_map, scan->header);
}

// -----------------------------------------------------
// Publisher
// -----------------------------------------------------

void CostmapNode::publishCostmap(
  const std::vector<std::vector<int8_t>> &grid,
  const std_msgs::msg::Header &hdr) 
{
  nav_msgs::msg::OccupancyGrid out;
  out.header = hdr;
  out.header.frame_id = "map";

  out.info.resolution = RESOLUTION;
  out.info.width = WIDTH;
  out.info.height = HEIGHT;

  out.info.origin.position.x = -(WIDTH * RESOLUTION) / 2.0;
  out.info.origin.position.y = -(HEIGHT * RESOLUTION) / 2.0;
  out.info.origin.orientation.w = 1.0;

  out.data.resize(WIDTH * HEIGHT);

  // flatten row-major grid
  for (int j = 0; j < HEIGHT; ++j) {
    for (int i = 0; i < WIDTH; ++i) {
      out.data[j * WIDTH + i] = grid[i][j];
    }
  }

  costmap_pub_->publish(out);
}

// -----------------------------------------------------
// Main
// -----------------------------------------------------

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
