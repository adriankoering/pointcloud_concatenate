#include "ConcatenateSixClouds.h"

#include <ros/ros.h>
#include <pcl_ros/transforms.h>

void ConcatenateSixClouds::onInit() {
    auto& nh = getNodeHandle();
    auto& pnh = getPrivateNodeHandle();

    pnh.param("target_frame", target_frame_, target_frame_);

  // Initialize Publisher first s.t. its ready before first callback
  cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("cloud_out", 10);

  // Setup Subscribers and callback
  cloud1_sub_.subscribe(nh, "cloud_in1", 5);
  cloud2_sub_.subscribe(nh, "cloud_in2", 5);
  cloud3_sub_.subscribe(nh, "cloud_in3", 5);
  cloud4_sub_.subscribe(nh, "cloud_in4", 5);
  cloud5_sub_.subscribe(nh, "cloud_in5", 5);
  cloud6_sub_.subscribe(nh, "cloud_in6", 5);
  
  sync_ =
      std::make_unique<Synchronizer>(SyncPolicy(5), cloud1_sub_, cloud2_sub_, cloud3_sub_, cloud4_sub_, cloud5_sub_, cloud6_sub_);
  sync_->registerCallback(
      boost::bind(&ConcatenateSixClouds::msgCallback, this, _1, _2, _3, _4, _5, _6));
}

void ConcatenateSixClouds::msgCallback(const sensor_msgs::PointCloud2ConstPtr &cloud1,
                                       const sensor_msgs::PointCloud2ConstPtr &cloud2,
                                       const sensor_msgs::PointCloud2ConstPtr &cloud3,
                                       const sensor_msgs::PointCloud2ConstPtr &cloud4,
                                       const sensor_msgs::PointCloud2ConstPtr &cloud5,
                                       const sensor_msgs::PointCloud2ConstPtr &cloud6) {
 
  sensor_msgs::PointCloud2 target_cloud1;
  pcl_ros::transformPointCloud(target_frame_, *cloud1, target_cloud1, buffer_);

  sensor_msgs::PointCloud2 target_cloud2;
  pcl_ros::transformPointCloud(target_frame_, *cloud2, target_cloud2, buffer_);

  sensor_msgs::PointCloud2 target_cloud3;
  pcl_ros::transformPointCloud(target_frame_, *cloud3, target_cloud3, buffer_);

  sensor_msgs::PointCloud2 target_cloud4;
  pcl_ros::transformPointCloud(target_frame_, *cloud4, target_cloud4, buffer_);

  sensor_msgs::PointCloud2 target_cloud5;
  pcl_ros::transformPointCloud(target_frame_, *cloud5, target_cloud5, buffer_);

  sensor_msgs::PointCloud2 target_cloud6;
  pcl_ros::transformPointCloud(target_frame_, *cloud6, target_cloud6, buffer_);

  sensor_msgs::PointCloud2 tmp1;
  pcl::concatenatePointCloud(target_cloud1, target_cloud2, tmp1);

  sensor_msgs::PointCloud2 tmp2;
  pcl::concatenatePointCloud(target_cloud3, target_cloud4, tmp2);

  sensor_msgs::PointCloud2 tmp3;
  pcl::concatenatePointCloud(tmp1, tmp2, tmp3);
  
  sensor_msgs::PointCloud2 tmp4;
  pcl::concatenatePointCloud(target_cloud5, target_cloud6, tmp4);

  sensor_msgs::PointCloud2 out;
  pcl::concatenatePointCloud(tmp3, tmp4, out);

  cloud_pub_.publish(out);
}
