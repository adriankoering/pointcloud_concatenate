cmake_minimum_required(VERSION 3.8)
project(pointcloud_concatenate VERSION 2.0.0)

add_compile_options(-Wall -Wextra -O2)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ## Find dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet

  tf2
  tf2_ros

  pcl_ros
  sensor_msgs
)

# ## Define package
catkin_package(
  INCLUDE_DIRS src
  LIBRARIES concatenate_clouds
  CATKIN_DEPENDS roscpp tf2 tf2_ros pcl_ros sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(concatenate_clouds
  src/ConcatenateTwoClouds.cpp
  src/ConcatenateFourClouds.cpp

  # src/ConcatenateSixClouds.cpp
  # src/ConcatenateEightClouds.cpp
)
target_link_libraries(concatenate_clouds
  ${catkin_LIBRARIES}
)