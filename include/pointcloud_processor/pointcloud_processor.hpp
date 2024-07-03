#ifndef POINTCLOUD_PROCESSOR_HPP_
#define POINTCLOUD_PROCESSOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class PointCloudProcessor : public rclcpp::Node
{
public:
  PointCloudProcessor();

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bounding_boxes_publisher_;
};

#endif  // POINTCLOUD_PROCESSOR_HPP_
