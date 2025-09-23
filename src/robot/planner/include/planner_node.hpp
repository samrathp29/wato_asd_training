
#ifndef WATO_PLANNER_NODE_H_
#define WATO_PLANNER_NODE_H_


#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


#include "planner_core.hpp"


// represents a 2d cell coordinate in the grid
struct GridCoord {
  int row;
  int col;

  GridCoord(int r, int c) : row(r), col(c) {}
  GridCoord() : row(0), col(0) {}

  bool operator==(const GridCoord &rhs) const {
    return (row == rhs.row && col == rhs.col);
  }

  bool operator!=(const GridCoord &rhs) const {
    return !(*this == rhs);
  }
};

// hash functor for GridCoord for use in unordered_map
struct GridCoordHasher {
  std::size_t operator()(const GridCoord &coord) const {
    return std::hash<int>()(coord.row) ^ (std::hash<int>()(coord.col) << 1);
  }
};

// node structure for a* open list
struct OpenNode {
  GridCoord cell;
  double cost_sum; // f = g + h

  OpenNode(GridCoord c, double cost) : cell(c), cost_sum(cost) {}
};

// priority queue comparator for opennode (min-heap by cost_sum)
struct OpenNodeCompare {
  bool operator()(const OpenNode &lhs, const OpenNode &rhs) {
    return lhs.cost_sum > rhs.cost_sum;
  }
};


// main planner node for ros2
class PlannerNode : public rclcpp::Node {
public:
  PlannerNode();

private:
  robot::PlannerCore planner_;

  enum PlannerState {
    AWAITING_GOAL,
    TRACKING_GOAL
  };
  PlannerState planner_state_;

  // ROS2 interfaces
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::TimerBase::SharedPtr periodic_timer_;

  // Internal state
  nav_msgs::msg::OccupancyGrid map_data_;
  geometry_msgs::msg::PointStamped target_goal_;
  geometry_msgs::msg::Pose robot_pose_;

  bool has_goal_ = false;

  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onGoal(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onTimer();

  bool isGoalReached();
  void computePath();
};

#endif // WATO_PLANNER_NODE_H_