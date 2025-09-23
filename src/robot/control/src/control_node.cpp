#include "control_node.hpp"
#include <cmath>
#include <algorithm>  // for clamp

ControlNode::ControlNode()
: Node("control"),
  control_(robot::ControlCore(this->get_logger())),
  lookahead_distance_(1.0),
  goal_tolerance_(0.1),
  linear_speed_(1.3) 
{
  // sub to path
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

  // sub to odom
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  // pub for velocity commands
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // run loop at ~10hz
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&ControlNode::controlLoop, this));
}

// ----------------- callbacks -----------------

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  current_path_ = msg;
  RCLCPP_INFO(this->get_logger(), "got path with %zu waypoints", msg->poses.size());
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_odom_ = msg;
}

void ControlNode::controlLoop() {
  // make sure we actually have path + odom
  if (!current_path_ || !robot_odom_ || current_path_->poses.empty()) {
    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
    return;
  }

  // get a lookahead target
  auto target = findLookaheadPoint();
  if (!target) {
    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
    return;
  }

  // compute command to move towards target
  auto cmd = computeVelocity(*target);
  cmd_vel_pub_->publish(cmd);
}

// ----------------- helpers -----------------

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  const auto& robot_xy = robot_odom_->pose.pose.position;

  // loop through path until we find a point at least lookahead_distance_ away
  for (size_t i = 0; i < current_path_->poses.size(); ++i) {
    const auto& p = current_path_->poses[i].pose.position;
    double d = computeDistance(robot_xy, p);
    if (d >= lookahead_distance_) {
      return current_path_->poses[i];
    }
  }

  // nothing far enough, check if we're basically at the goal
  if (!current_path_->poses.empty()) {
    const auto& goal = current_path_->poses.back().pose.position;
    if (computeDistance(robot_xy, goal) < goal_tolerance_) {
      return current_path_->poses.back();
    }
  }

  return std::nullopt;
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(
  const geometry_msgs::msg::PoseStamped &target) 
{
  geometry_msgs::msg::Twist cmd;

  // robot pose and heading
  const auto& pos = robot_odom_->pose.pose.position;
  double yaw = extractYaw(robot_odom_->pose.pose.orientation);

  // direction to target
  double dx = target.pose.position.x - pos.x;
  double dy = target.pose.position.y - pos.y;
  double target_heading = std::atan2(dy, dx);

  // wrap angle difference into [-pi, pi]
  double ang_diff = target_heading - yaw;
  while (ang_diff > M_PI) ang_diff -= 2 * M_PI;
  while (ang_diff < -M_PI) ang_diff += 2 * M_PI;

  // slow down when turning sharp
  double slowdown = std::clamp(std::cos(ang_diff), 0.0, 1.0);

  // set velocities
  cmd.linear.x = linear_speed_ * slowdown;
  cmd.angular.z = 2.0 * ang_diff;  // quick p controller for steering

  // clamp angular velocity to max range
  const double max_turn = 1.5;
  if (cmd.angular.z >  max_turn) cmd.angular.z =  max_turn;
  if (cmd.angular.z < -max_turn) cmd.angular.z = -max_turn;

  return cmd;
}

double ControlNode::computeDistance(
  const geometry_msgs::msg::Point &a,
  const geometry_msgs::msg::Point &b) 
{
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &q) {
  // helper to pull yaw from quaternion
  return tf2::getYaw(q);
}

// ----------------- main -----------------

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
