#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl_ros/point_cloud.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

using Point = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<Point>;

struct ConcatenateSixClouds : public nodelet::Nodelet {
  ConcatenateSixClouds() : listener_(buffer_) {}
  virtual ~ConcatenateSixClouds() = default;

  void onInit() override;

  void msgCallback(const sensor_msgs::PointCloud2ConstPtr &cloud1,
                   const sensor_msgs::PointCloud2ConstPtr &cloud2,
                   const sensor_msgs::PointCloud2ConstPtr &cloud3,
                   const sensor_msgs::PointCloud2ConstPtr &cloud4,
                   const sensor_msgs::PointCloud2ConstPtr &cloud5,
                   const sensor_msgs::PointCloud2ConstPtr &cloud6);

private:
  // Publisher
  ros::Publisher cloud_pub_;

  // Subscriptions
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud1_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud2_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud3_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud4_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud5_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud6_sub_;

  // Synchronization
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,
      sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,
      sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  std::unique_ptr<Synchronizer> sync_;

  // Transforms
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;

  // Parameters
  std::string target_frame_;
};

PLUGINLIB_EXPORT_CLASS(ConcatenateSixClouds, nodelet::Nodelet)
