#include <chrono>
#include <memory>
#include <cmath>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() 
: Node("costmap"), 
  costmap_(robot::CostmapCore(this->get_logger())) 
{
    // --- Subscribe to Lidar ---
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar", 10,
        std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1)
    );

    // --- Publish Costmap ---
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    auto costmap = createCostmapFromScan(*msg);
    costmap_pub_->publish(costmap);
}

nav_msgs::msg::OccupancyGrid CostmapNode::createCostmapFromScan(const sensor_msgs::msg::LaserScan &scan) {
    nav_msgs::msg::OccupancyGrid grid;

    // --- Fill Metadata ---
    grid.header.stamp = this->now();
    grid.header.frame_id = "base_link";  // adjust if your TF tree uses something else
    grid.info.resolution = 0.05;  // 5 cm per grid cell
    grid.info.width = 200;        // 10m x 10m map
    grid.info.height = 200;
    grid.info.origin.position.x = -5.0;
    grid.info.origin.position.y = -5.0;
    grid.info.origin.position.z = 0.0;

    grid.data.assign(grid.info.width * grid.info.height, 0); // 0 = free

    // --- Fill cells based on lidar ---
    for (size_t i = 0; i < scan.ranges.size(); i++) {
        float r = scan.ranges[i];
        if (std::isnan(r) || r < scan.range_min || r > scan.range_max) continue;

        float angle = scan.angle_min + i * scan.angle_increment;
        float x = r * std::cos(angle);
        float y = r * std::sin(angle);

        int gx = static_cast<int>((x + 5.0) / grid.info.resolution);
        int gy = static_cast<int>((y + 5.0) / grid.info.resolution);

        if (gx >= 0 && gx < (int)grid.info.width && gy >= 0 && gy < (int)grid.info.height) {
            grid.data[gy * grid.info.width + gx] = 100;  // 100 = occupied
        }
    }

    return grid;
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CostmapNode>());
    rclcpp::shutdown();
    return 0;
}
