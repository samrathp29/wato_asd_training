#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
public:
    CostmapNode();

private:
    robot::CostmapCore costmap_;

    // 🔑 ROS2 Interfaces
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

    // 🔑 Callback + Helper
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    nav_msgs::msg::OccupancyGrid createCostmapFromScan(const sensor_msgs::msg::LaserScan &scan);
};

#endif
