#include "ConcatenateFourClouds.h"
#include "common.h"

#include <pcl_ros/transforms.h>
#include <ros/ros.h>

void ConcatenateFourClouds::onInit() {
  auto &nh = getNodeHandle();
  auto &pnh = getPrivateNodeHandle();

  pnh.param("target_frame", target_frame_, target_frame_);

  // Initialize Publisher first s.t. its ready before first callback
  cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("cloud_out", 10);

  // Setup Subscribers and callback
  cloud1_sub_.subscribe(nh, "cloud_in1", 50);
  cloud2_sub_.subscribe(nh, "cloud_in2", 50);
  cloud3_sub_.subscribe(nh, "cloud_in3", 50);
  cloud4_sub_.subscribe(nh, "cloud_in4", 50);

  sync_ = std::make_unique<Synchronizer>(SyncPolicy(50), cloud1_sub_,
                                         cloud2_sub_, cloud3_sub_, cloud4_sub_);
  sync_->registerCallback(
      boost::bind(&ConcatenateFourClouds::msgCallback, this, _1, _2, _3, _4));

  ROS_INFO("[ConcatFourClouds] Startup :)");
}

void ConcatenateFourClouds::msgCallback(
    const sensor_msgs::PointCloud2ConstPtr &cloud1,
    const sensor_msgs::PointCloud2ConstPtr &cloud2,
    const sensor_msgs::PointCloud2ConstPtr &cloud3,
    const sensor_msgs::PointCloud2ConstPtr &cloud4) {
  ROS_INFO_STREAM_ONCE("Concat 4 Clouds: msgCallback");

  // find common set of fields between clouds:
  std::vector<std::string> common_fields = getCommonFields(
      {cloud1->fields, cloud2->fields, cloud3->fields, cloud4->fields});

  sensor_msgs::PointCloud2 target_cloud1;
  pcl_ros::transformPointCloud(target_frame_, *cloud1, target_cloud1, buffer_);

  sensor_msgs::PointCloud2 target_cloud2;
  pcl_ros::transformPointCloud(target_frame_, *cloud2, target_cloud2, buffer_);

  sensor_msgs::PointCloud2 target_cloud3;
  pcl_ros::transformPointCloud(target_frame_, *cloud3, target_cloud3, buffer_);

  sensor_msgs::PointCloud2 target_cloud4;
  pcl_ros::transformPointCloud(target_frame_, *cloud4, target_cloud4, buffer_);

  // TODO: generic concatenation of multiple clouds with common fields only
  sensor_msgs::PointCloud2 tmp1;
  pcl::concatenatePointCloud(target_cloud1, target_cloud2, tmp1);

  sensor_msgs::PointCloud2 tmp2;
  pcl::concatenatePointCloud(target_cloud3, target_cloud4, tmp2);

  sensor_msgs::PointCloud2 out;
  pcl::concatenatePointCloud(tmp1, tmp2, out);

  cloud_pub_.publish(out);
}
