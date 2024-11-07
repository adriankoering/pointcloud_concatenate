#include "pointcloud_concatenate.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "pointcloud_concatenate");

  PointcloudConcatenate node;

  ros::spin();
  return 0;
}