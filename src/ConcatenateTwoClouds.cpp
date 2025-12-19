#include "ConcatenateTwoClouds.h"
#include "common.h"

#include <pcl_ros/transforms.h>
#include <ros/ros.h>

void ConcatenateTwoClouds::onInit() {
  auto &nh = getNodeHandle();
  auto &pnh = getPrivateNodeHandle();

  pnh.param("target_frame", target_frame_, target_frame_);

  // Initialize Publisher first s.t. its ready before first callback
  cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("cloud_out", 10);

  // Setup Subscribers and callback
  cloud1_sub_.subscribe(nh, "cloud_in1", 5);
  cloud2_sub_.subscribe(nh, "cloud_in2", 5);

  sync_ =
      std::make_unique<Synchronizer>(SyncPolicy(5), cloud1_sub_, cloud2_sub_);
  sync_->registerCallback(
      boost::bind(&ConcatenateTwoClouds::msgCallback, this, _1, _2));
}

void ConcatenateTwoClouds::msgCallback(
    const sensor_msgs::PointCloud2ConstPtr &cloud1,
    const sensor_msgs::PointCloud2ConstPtr &cloud2) {

  try {
    sensor_msgs::PointCloud2 target_cloud1;
    // TODO: Does this do ego-motion compensation already?
    // TODO: Should transform everything into a common point in time, too!
    pcl_ros::transformPointCloud(target_frame_, *cloud1, target_cloud1,
                                 buffer_);

    sensor_msgs::PointCloud2 target_cloud2;
    pcl_ros::transformPointCloud(target_frame_, *cloud2, target_cloud2,
                                 buffer_);

    sensor_msgs::PointCloud2 out = concatenate({target_cloud1, target_cloud2});
    cloud_pub_.publish(out);

  } catch (const std::exception &e) {
    ROS_WARN_STREAM("[Cloud|Concat] Failed with " << e.what());
  }
}
