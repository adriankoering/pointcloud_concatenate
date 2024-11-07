cmake_minimum_required(VERSION 3.8)
project(pointcloud_concatenate VERSION 2.0.0)

add_compile_options(-Wall -Wextra -O2)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

### Find dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp

  tf2
  tf2_ros
  
  pcl_ros
  sensor_msgs
)

### Define package
catkin_package(
  INCLUDE_DIRS src
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Declare cpp executables
add_executable(pointcloud_concatenate_node
  src/pointcloud_concatenate_node.cpp
)
target_link_libraries(pointcloud_concatenate_node
  ${catkin_LIBRARIES}
)