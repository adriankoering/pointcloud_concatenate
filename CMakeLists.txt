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

add_library(concatenate_two_clouds src/ConcatenateTwoClouds.cpp)
target_link_libraries(concatenate_two_clouds
  ${catkin_LIBRARIES}
)

add_library(concatenate_four_clouds src/ConcatenateFourClouds.cpp)
target_link_libraries(concatenate_four_clouds
  ${catkin_LIBRARIES}
)

add_library(concatenate_six_clouds src/ConcatenateSixClouds.cpp)
target_link_libraries(concatenate_six_clouds
  ${catkin_LIBRARIES}
)

add_library(concatenate_eight_clouds src/ConcatenateEightClouds.cpp)
target_link_libraries(concatenate_eight_clouds
  ${catkin_LIBRARIES}
)