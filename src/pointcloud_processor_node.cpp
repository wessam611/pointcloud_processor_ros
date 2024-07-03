#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "pointcloud_processor/pointcloud_processor.hpp"
#include <iostream>

#include <curl/curl.h>
#include <json/json.h>

std::size_t callback(
  const char* in,
  std::size_t size,
  std::size_t num,
  std::string* out)
{
  const std::size_t totalBytes(size * num);
  out->append(in, totalBytes);
  return totalBytes;
}

PointCloudProcessor::PointCloudProcessor() : Node("point_cloud_processor")
  {
    // Initialize subscriber to PointCloud2 message
    point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lidar_points", 10, std::bind(&PointCloudProcessor::pointCloudCallback, this, std::placeholders::_1));

    // Initialize publisher for bounding boxes
    bounding_boxes_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("bboxes", 10);
  }

  void PointCloudProcessor::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Process the point cloud and generate bounding boxes
    processPointCloud(msg);

  }

  void PointCloudProcessor::processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    CURL *curl = curl_easy_init();
    if (curl) {
      auto pcdata = msg->data.data();
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

      // Create an iterator for the intensity field (assuming it's available in your PointCloud2 message)
      sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg, "intensity");

      // Create a binary buffer to store the converted data
      std::vector<char> binary_buffer;

      // Iterate through the points and convert them to the desired format
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity)
      {
        // Append XYZ values
        binary_buffer.insert(binary_buffer.end(), reinterpret_cast<const char*>(&(*iter_x)), reinterpret_cast<const char*>(&(*iter_x) + 1));
        binary_buffer.insert(binary_buffer.end(), reinterpret_cast<const char*>(&(*iter_y)), reinterpret_cast<const char*>(&(*iter_y) + 1));
        binary_buffer.insert(binary_buffer.end(), reinterpret_cast<const char*>(&(*iter_z)), reinterpret_cast<const char*>(&(*iter_z) + 1));

        // Append intensity value (if available)
        if (iter_intensity != iter_intensity.end())
        {
          // binary_buffer.insert(binary_buffer.end(), reinterpret_cast<const char*>(&(*iter_intensity)), reinterpret_cast<const char*>(&(*iter_intensity) + 1));
          float fill = 0;
          binary_buffer.insert(binary_buffer.end(), reinterpret_cast<const char*>(&(fill)), reinterpret_cast<const char*>(&(fill) + 1));
        }

        // Append zeros in the 5th column
        float zero_value = 0.0f;
        binary_buffer.insert(binary_buffer.end(), reinterpret_cast<const char*>(&zero_value), reinterpret_cast<const char*>(&zero_value + 1));
      }

      // Set the URL for your endpoint
      struct curl_slist *headerlist = NULL;
      headerlist = curl_slist_append(headerlist, "Content-Type: application/x-protobuf");

      curl_easy_setopt(curl, CURLOPT_URL, "http://127.0.0.1:8080/predictions/st3d_hv");



      std::unique_ptr<std::string> httpData(new std::string());

      curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, callback);
      curl_easy_setopt(curl, CURLOPT_WRITEDATA, httpData.get());
      // Set the payload (binary buffer from the PointCloud2 message)

      curl_easy_setopt(curl, CURLOPT_POSTFIELDS, binary_buffer.data());
      // Set the payload size
      curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headerlist);
      curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, binary_buffer.size());

      // Perform the request
      CURLcode res = curl_easy_perform(curl);


      if (res == CURLE_OK) {
        Json::Value root;
        Json::Reader reader;
        std::vector<float> boxes_vector;
        if (reader.parse(*httpData.get(), root)) {
          Json::Value boxes = root["3dbbox"];
          Json::Value lbles = root["labels"];
          int index_lbl = 0;
          for (auto box: boxes) {
            index_lbl++;
            if (lbles[index_lbl-1].asInt() != 1) {
              continue;
            }
            for (auto elem: box) {
              boxes_vector.push_back(elem.asFloat());
            }
          }
          // auto point_cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
          // point_cloud_msg->header.stamp = this->get_clock()->now();
          // point_cloud_msg->header.frame_id = "base_link"; // Set your desired frame_id
          // point_cloud_msg->height = 1;
          // point_cloud_msg->width = (int)(boxes_vector.size()/3); // Increment width for each new point
          // // sensor_msgs::PointCloud2Modifier modifier(*point_cloud_msg);
          // // modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
          // //                                   "y", 1, sensor_msgs::msg::PointField::FLOAT32,
          // //                                   "z", 1, sensor_msgs::msg::PointField::FLOAT32);

          // // sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud_msg, "x");
          // // sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud_msg, "y");
          // // sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud_msg, "z");
          // // for (size_t i = 0; i < point_cloud_msg->width; ++i)
          // // {
          // //     *iter_x = boxes_vector[i*3];
          // //     *iter_y = boxes_vector[i*3+1];
          // //     *iter_z = boxes_vector[i*3+2];
          // //     ++iter_x;++iter_y;++iter_z;
          // // }
          // bboxpc_publisher_->publish(std::move(point_cloud_msg));
          auto marker_array_msg = std::make_unique<visualization_msgs::msg::MarkerArray>();
          // markers.push_back()
          for (int i=0; i < boxes_vector.size(); i+= 7) {
            if (i/7 > 1)
              break;
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "lidar_link";
            marker.header.stamp = this->now();
            marker.id = (int)(i/7);
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.pose.position.x = boxes_vector[i+0];
            marker.pose.position.y = boxes_vector[i+1];
            marker.pose.position.z = boxes_vector[i+2];
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, boxes_vector[i+6]);
            marker.pose.orientation =  tf2::toMsg(q);
            marker.scale.x = boxes_vector[i+3];
            marker.scale.y = boxes_vector[i+4];
            marker.scale.z = boxes_vector[i+5];
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.5;
            marker_array_msg->markers.push_back(marker);
            // std::cout << boxes_vector[i+3] << std::endl;
            // std::cout << boxes_vector[i+4] << std::endl;
            // std::cout << boxes_vector[i+5] << std::endl << std::endl;
          }
          bounding_boxes_publisher_->publish(std::move(marker_array_msg));
        }
      } 
      // else {
      //   int x = 0;
      //   int y = 5/x;
      //   std::cout << y << std::endl;
      // }
      // Cleanup
      curl_easy_cleanup(curl);
    }
  }


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudProcessor>());
  rclcpp::shutdown();
  return 0;
}
