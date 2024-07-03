cmake_minimum_required(VERSION 3.5)
project(pointcloud_processor)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(CURL REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(tf2 REQUIRED)
# Add include directories
include_directories(include ${CURL_INCLUDE_DIRS})


# Create an executable
add_executable(pointcloud_processor_node src/pointcloud_processor_node.cpp)

# Link the executable with the necessary libraries
ament_target_dependencies(pointcloud_processor_node
  rclcpp
  sensor_msgs
  visualization_msgs
  tf2
)


target_link_libraries(pointcloud_processor_node ${CURL_LIBRARIES} jsoncpp_lib)
target_include_directories(pointcloud_processor_node
  PRIVATE
  ${JSONCPP_INCLUDE_DIRS}
)

# Install the executable
install(TARGETS
  pointcloud_processor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch file
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install header files
install(DIRECTORY
  include/
  DESTINATION include/
)

ament_package()
