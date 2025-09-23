
#include "planner_node.hpp"
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>


PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  // set up ros2 interfaces
  map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::onMap, this, std::placeholders::_1));
  goal_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::onGoal, this, std::placeholders::_1));
  odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::onOdom, this, std::placeholders::_1));
  path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  periodic_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::onTimer, this));

  planner_state_ = PlannerState::AWAITING_GOAL;

  map_data_ = nav_msgs::msg::OccupancyGrid();
  target_goal_ = geometry_msgs::msg::PointStamped();
  robot_pose_ = geometry_msgs::msg::Pose();
}


void PlannerNode::onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  map_data_ = *msg;
  if (planner_state_ == PlannerState::TRACKING_GOAL) {
    computePath();
  }
}

void PlannerNode::onGoal(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  target_goal_ = *msg;
  has_goal_ = true;
  planner_state_ = PlannerState::TRACKING_GOAL;
  computePath();
}

void PlannerNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::onTimer() {
  if (planner_state_ == PlannerState::TRACKING_GOAL) {
    if (isGoalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      planner_state_ = PlannerState::AWAITING_GOAL;
    } else {
      RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
      computePath();
    }
  }
}

bool PlannerNode::isGoalReached() {
  double dx = target_goal_.point.x - robot_pose_.position.x;
  double dy = target_goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < 0.5;
}

void PlannerNode::computePath() {
  if (!has_goal_ || map_data_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = "sim_world";
  std::vector<GridCoord> path_cells;

  int start_row = static_cast<int>((robot_pose_.position.x - map_data_.info.origin.position.x) / map_data_.info.resolution);
  int start_col = static_cast<int>((robot_pose_.position.y - map_data_.info.origin.position.y) / map_data_.info.resolution);
  int goal_row = static_cast<int>((target_goal_.point.x - map_data_.info.origin.position.x) / map_data_.info.resolution);
  int goal_col = static_cast<int>((target_goal_.point.y - map_data_.info.origin.position.y) / map_data_.info.resolution);

  GridCoord start(start_row, start_col);
  GridCoord goal(goal_row, goal_col);

  if (start_row < 0 || start_row >= static_cast<int>(map_data_.info.width) ||
      start_col < 0 || start_col >= static_cast<int>(map_data_.info.height)) {
    RCLCPP_WARN(this->get_logger(), "Start position is outside map bounds!");
    path_publisher_->publish(path);
    return;
  }

  if (goal_row < 0 || goal_row >= static_cast<int>(map_data_.info.width) ||
      goal_col < 0 || goal_col >= static_cast<int>(map_data_.info.height)) {
    RCLCPP_WARN(this->get_logger(), "Goal position is outside map bounds!");
    path_publisher_->publish(path);
    return;
  }

  int start_index = start_col * map_data_.info.width + start_row;
  int goal_index = goal_col * map_data_.info.width + goal_row;

  if (map_data_.data[start_index] != 0) {
    RCLCPP_WARN(this->get_logger(), "Start position is occupied! Attempting recovery...");
    bool found_free_cell = false;
    int search_radius = 1;
    int max_search_radius = 10;
    while (!found_free_cell && search_radius <= max_search_radius) {
      for (int dx = -search_radius; dx <= search_radius; dx++) {
        for (int dy = -search_radius; dy <= search_radius; dy++) {
          if (dx == 0 && dy == 0) continue;
          int new_row = start_row + dx;
          int new_col = start_col + dy;
          if (new_row < 0 || new_row >= static_cast<int>(map_data_.info.width) ||
              new_col < 0 || new_col >= static_cast<int>(map_data_.info.height)) {
            continue;
          }
          int new_index = new_col * map_data_.info.width + new_row;
          if (map_data_.data[new_index] == 0) {
            start_row = new_row;
            start_col = new_col;
            start = GridCoord(start_row, start_col);
            found_free_cell = true;
            RCLCPP_INFO(this->get_logger(), "Found free cell at (%d, %d) for recovery", start_row, start_col);
            break;
          }
        }
        if (found_free_cell) break;
      }
      if (!found_free_cell) search_radius++;
    }
    if (!found_free_cell) {
      RCLCPP_ERROR(this->get_logger(), "No free cell found for recovery! Robot is completely surrounded!");
      path_publisher_->publish(path);
      return;
    }
  }

  if (map_data_.data[goal_index] != 0) {
    RCLCPP_WARN(this->get_logger(), "Goal position is occupied! Attempting goal relaxation...");
    bool found_free_goal = false;
    int search_radius = 1;
    int max_search_radius = 15;
    while (!found_free_goal && search_radius <= max_search_radius) {
      for (int dx = -search_radius; dx <= search_radius; dx++) {
        for (int dy = -search_radius; dy <= search_radius; dy++) {
          if (dx == 0 && dy == 0) continue;
          int new_row = goal_row + dx;
          int new_col = goal_col + dy;
          if (new_row < 0 || new_row >= static_cast<int>(map_data_.info.width) ||
              new_col < 0 || new_col >= static_cast<int>(map_data_.info.height)) {
            continue;
          }
          int new_index = new_col * map_data_.info.width + new_row;
          if (map_data_.data[new_index] == 0) {
            goal_row = new_row;
            goal_col = new_col;
            goal = GridCoord(goal_row, goal_col);
            found_free_goal = true;
            RCLCPP_INFO(this->get_logger(), "Relaxed goal to (%d, %d)", goal_row, goal_col);
            break;
          }
        }
        if (found_free_goal) break;
      }
      if (!found_free_goal) search_radius++;
    }
    if (!found_free_goal) {
      RCLCPP_ERROR(this->get_logger(), "No free cell found near goal! Goal is completely surrounded!");
      path_publisher_->publish(path);
      return;
    }
  }

  std::priority_queue<OpenNode, std::vector<OpenNode>, OpenNodeCompare> open_set;
  std::unordered_map<GridCoord, GridCoord, GridCoordHasher> came_from;
  std::unordered_map<GridCoord, double, GridCoordHasher> g_score;
  std::unordered_map<GridCoord, double, GridCoordHasher> f_score;
  std::unordered_map<GridCoord, bool, GridCoordHasher> in_open_set;

  g_score[start] = 0.0;
  f_score[start] = std::sqrt((start.row - goal.row) * (start.row - goal.row) + (start.col - goal.col) * (start.col - goal.col));
  open_set.push(OpenNode(start, f_score[start]));
  in_open_set[start] = true;

  RCLCPP_INFO(this->get_logger(), "Starting A* pathfinding from (%d, %d) to (%d, %d)", start.row, start.col, goal.row, goal.col);

  int max_iterations = 10000;
  int iteration_count = 0;
  int replan_attempts = 0;
  int max_replan_attempts = 3;

  while (!open_set.empty() && iteration_count < max_iterations) {
    iteration_count++;
    OpenNode current = open_set.top();
    open_set.pop();
    in_open_set[current.cell] = false;

    if (current.cell == goal) {
      RCLCPP_INFO(this->get_logger(), "Path found!");
      GridCoord path_current = goal;
      while (path_current != start) {
        path_cells.push_back(path_current);
        auto it = came_from.find(path_current);
        if (it == came_from.end()) {
          RCLCPP_WARN(this->get_logger(), "Path reconstruction failed!");
          path_publisher_->publish(path);
          return;
        }
        path_current = it->second;
      }
      path_cells.push_back(start);
      std::reverse(path_cells.begin(), path_cells.end());
      for (const auto& cell : path_cells) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = path.header;
        pose_stamped.pose.position.x = cell.row * map_data_.info.resolution + map_data_.info.origin.position.x;
        pose_stamped.pose.position.y = cell.col * map_data_.info.resolution + map_data_.info.origin.position.y;
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation.w = 1.0;
        path.poses.push_back(pose_stamped);
      }
      RCLCPP_INFO(this->get_logger(), "Published path with %zu waypoints", path.poses.size());
      path_publisher_->publish(path);
      return;
    }

    int current_index = current.cell.col * map_data_.info.width + current.cell.row;
    if (map_data_.data[current_index] != 0) {
      RCLCPP_WARN(this->get_logger(), "Path blocked at (%d, %d)! Attempting dynamic replanning...", current.cell.row, current.cell.col);
      continue;
    }

    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        if (dx == 0 && dy == 0) continue;
        GridCoord neighbor(current.cell.row + dx, current.cell.col + dy);
        if (neighbor.row < 0 || neighbor.row >= static_cast<int>(map_data_.info.width) ||
            neighbor.col < 0 || neighbor.col >= static_cast<int>(map_data_.info.height)) {
          continue;
        }
        int neighbor_index = neighbor.col * map_data_.info.width + neighbor.row;
        if (map_data_.data[neighbor_index] != 0) {
          bool collision_risk = false;
          for (int check_dx = -1; check_dx <= 1; check_dx++) {
            for (int check_dy = -1; check_dy <= 1; check_dy++) {
              int check_row = neighbor.row + check_dx;
              int check_col = neighbor.col + check_dy;
              if (check_row >= 0 && check_row < static_cast<int>(map_data_.info.width) &&
                  check_col >= 0 && check_col < static_cast<int>(map_data_.info.height)) {
                int check_index = check_col * map_data_.info.width + check_row;
                if (map_data_.data[check_index] != 0) {
                  collision_risk = true;
                  break;
                }
              }
            }
            if (collision_risk) break;
          }
          if (collision_risk) {
            RCLCPP_DEBUG(this->get_logger(), "Skipping neighbor (%d, %d) due to collision risk", neighbor.row, neighbor.col);
            continue;
          }
        }
        double base_cost = std::sqrt((current.cell.row - neighbor.row) * (current.cell.row - neighbor.row) +
                                    (current.cell.col - neighbor.col) * (current.cell.col - neighbor.col));
        double safety_penalty = 0.0;
        for (int check_dx = -1; check_dx <= 1; check_dx++) {
          for (int check_dy = -1; check_dy <= 1; check_dy++) {
            int check_row = neighbor.row + check_dx;
            int check_col = neighbor.col + check_dy;
            if (check_row >= 0 && check_row < static_cast<int>(map_data_.info.width) &&
                check_col >= 0 && check_col < static_cast<int>(map_data_.info.height)) {
              int check_index = check_col * map_data_.info.width + check_row;
              if (map_data_.data[check_index] != 0) {
                safety_penalty += 0.1;
              }
            }
          }
        }
        double tentative_g_score = g_score[current.cell] + base_cost + safety_penalty;
        if (g_score.find(neighbor) == g_score.end() || tentative_g_score < g_score[neighbor]) {
          came_from[neighbor] = current.cell;
          g_score[neighbor] = tentative_g_score;
          f_score[neighbor] = tentative_g_score +
            std::sqrt((neighbor.row - goal.row) * (neighbor.row - goal.row) +
                     (neighbor.col - goal.col) * (neighbor.col - goal.col));
          if (!in_open_set[neighbor]) {
            open_set.push(OpenNode(neighbor, f_score[neighbor]));
            in_open_set[neighbor] = true;
          }
        }
      }
    }
  }

  if (iteration_count >= max_iterations) {
    RCLCPP_ERROR(this->get_logger(), "A* search exceeded maximum iterations! Path may be too complex.");
  } else {
    RCLCPP_WARN(this->get_logger(), "No path found to goal! Goal may be unreachable.");
  }

  RCLCPP_INFO(this->get_logger(), "Attempting recovery by finding nearest reachable point...");
  GridCoord best_reached = start;
  double best_distance_to_goal = std::sqrt((start.row - goal.row) * (start.row - goal.row) +
                                          (start.col - goal.col) * (start.col - goal.col));
  for (const auto& pair : g_score) {
    if (pair.first != start) {
      double distance_to_goal = std::sqrt((pair.first.row - goal.row) * (pair.first.row - goal.row) +
                                         (pair.first.col - goal.col) * (pair.first.col - goal.col));
      if (distance_to_goal < best_distance_to_goal) {
        best_reached = pair.first;
        best_distance_to_goal = distance_to_goal;
      }
    }
  }
  if (best_reached != start) {
    RCLCPP_INFO(this->get_logger(), "Found partial path to (%d, %d), distance to goal: %.2f", best_reached.row, best_reached.col, best_distance_to_goal);
    GridCoord path_current = best_reached;
    while (path_current != start) {
      path_cells.push_back(path_current);
      auto it = came_from.find(path_current);
      if (it == came_from.end()) {
        RCLCPP_WARN(this->get_logger(), "Partial path reconstruction failed!");
        break;
      }
      path_current = it->second;
    }
    path_cells.push_back(start);
    std::reverse(path_cells.begin(), path_cells.end());
    for (const auto& cell : path_cells) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header = path.header;
      pose_stamped.pose.position.x = cell.row * map_data_.info.resolution + map_data_.info.origin.position.x;
      pose_stamped.pose.position.y = cell.col * map_data_.info.resolution + map_data_.info.origin.position.y;
      pose_stamped.pose.position.z = 0.0;
      pose_stamped.pose.orientation.w = 1.0;
      path.poses.push_back(pose_stamped);
    }
    RCLCPP_INFO(this->get_logger(), "Published partial path with %zu waypoints", path.poses.size());
  } else {
    RCLCPP_ERROR(this->get_logger(), "No partial path found! Robot may be completely trapped.");
  }
  path_publisher_->publish(path);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}