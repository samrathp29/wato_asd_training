
#include "map_memory_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>


MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  costmap_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10, std::bind(&MapMemoryNode::onCostmap, this, std::placeholders::_1));
  odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&MapMemoryNode::onOdom, this, std::placeholders::_1));
  map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  periodic_timer_ = this->create_wall_timer(std::chrono::milliseconds(PUB_INTERVAL_MS), std::bind(&MapMemoryNode::onTimer, this));

  persistent_map_.header.frame_id = "sim_world";

  robot_pos_x_ = 0.0;
  robot_pos_y_ = 0.0;
  robot_theta_ = 0.0;
  last_pos_x_ = 0.0;
  last_pos_y_ = 0.0;

  persistent_map_.info.resolution = RESOLUTION;
  persistent_map_.info.width = WIDTH_CELLS;
  persistent_map_.info.height = HEIGHT_CELLS;
  persistent_map_.info.origin.position.x = -WIDTH_CELLS * RESOLUTION / 2.0;
  persistent_map_.info.origin.position.y = -HEIGHT_CELLS * RESOLUTION / 2.0;
  persistent_map_.data.assign(WIDTH_CELLS * HEIGHT_CELLS, 0);
}


void MapMemoryNode::onCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  costmap_snapshot_ = *msg;
  costmap_ready_ = true;
}


void MapMemoryNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double curr_x = msg->pose.pose.position.x;
  double curr_y = msg->pose.pose.position.y;

  double dist = std::hypot(curr_x - last_pos_x_, curr_y - last_pos_y_);
  if (dist < UPDATE_DIST_THRESH) return;

  robot_pos_x_ = curr_x;
  robot_pos_y_ = curr_y;

  auto q_msg = msg->pose.pose.orientation;
  tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  robot_theta_ = yaw;

  should_update_map_ = true;
}


void MapMemoryNode::onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  // not used in this implementation
}


void MapMemoryNode::onTimer() {
  if (costmap_ready_ && should_update_map_) {
    refreshMap();
    persistent_map_.header.stamp = this->now();
    persistent_map_.header.frame_id = "sim_world";
    map_publisher_->publish(persistent_map_);
    costmap_ready_ = false;
    should_update_map_ = false;
  }
}


void MapMemoryNode::refreshMap() {
  if (std::isnan(robot_pos_x_) || std::isnan(robot_pos_y_)) return;

  double local_res = costmap_snapshot_.info.resolution;
  int local_width = costmap_snapshot_.info.width;
  int local_height = costmap_snapshot_.info.height;
  auto& local_data = costmap_snapshot_.data;

  double global_res = persistent_map_.info.resolution;
  double global_origin_x = persistent_map_.info.origin.position.x;
  double global_origin_y = persistent_map_.info.origin.position.y;
  int global_width = persistent_map_.info.width;
  int global_height = persistent_map_.info.height;
  auto& global_data = persistent_map_.data;

  for(int y = 0; y < local_height; ++y) {
    for(int x = 0; x < local_width; ++x) {
      double lx = (x - local_width / 2) * local_res;
      double ly = (y - local_height / 2) * local_res;

      double gx = robot_pos_x_ + (lx * std::cos(robot_theta_) - ly * std::sin(robot_theta_));
      double gy = robot_pos_y_ + (lx * std::sin(robot_theta_) + ly * std::cos(robot_theta_));

      int idx_gx = static_cast<int>(std::round(gx / global_res + global_width / 2));
      int idx_gy = static_cast<int>(std::round(gy / global_res + global_height / 2));

      if (idx_gx < 0 || idx_gx >= global_width || idx_gy < 0 || idx_gy >= global_height) continue;

      int8_t& g_cost = global_data[idx_gy * global_width + idx_gx];
      g_cost = std::max(g_cost, local_data[y * local_width + x]);
    }
  }
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}