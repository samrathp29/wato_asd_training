
#ifndef WATO_MAP_MEMORY_NODE_H_
#define WATO_MAP_MEMORY_NODE_H_


#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include "map_memory_core.hpp"


// node for maintaining a persistent global map using local costmaps
class MapMemoryNode : public rclcpp::Node {
public:
  MapMemoryNode();

private:
  // callback handlers
  void onCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onTimer();
  void refreshMap();

  robot::MapMemoryCore map_memory_;
  nav_msgs::msg::OccupancyGrid costmap_snapshot_;
  nav_msgs::msg::OccupancyGrid persistent_map_;

  double robot_pos_x_;
  double robot_pos_y_;
  double robot_theta_;
  double last_pos_x_;
  double last_pos_y_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::TimerBase::SharedPtr periodic_timer_;

  bool should_update_map_ = false;
  bool costmap_ready_ = false;

  // map parameters
  static constexpr uint32_t PUB_INTERVAL_MS = 1000;    // Publish interval (ms)
  static constexpr double UPDATE_DIST_THRESH = 1.5;    // Distance threshold for update
  static constexpr double RESOLUTION = 0.1;            // Map resolution (m/cell)
  static constexpr int WIDTH_CELLS = 300;              // Map width (cells)
  static constexpr int HEIGHT_CELLS = 300;             // Map height (cells)
  static constexpr double ORIGIN_X = -15.0;            // Map origin x (m)
  static constexpr double ORIGIN_Y = -15.0;            // Map origin y (m)
};

#endif // WATO_MAP_MEMORY_NODE_H_