#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

struct PointcloudConcatenate {
  ~PointcloudConcatenate() = default;

  PointcloudConcatenate() : listener(buffer) {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    if (not pnh.getParam("target_frame", target_frame)) {
      ROS_ERROR_STREAM("[Concat|Cloud] Failed to get 'target_frame' parameter");
      exit(1);
    }

    // Initialize Publisher first s.t. its ready before first callback
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_out", 10);

    // Setup Subscribers and callback
    cloud1_sub.subscribe(nh, "cloud1", 5);
    cloud2_sub.subscribe(nh, "cloud2", 5);

    sync =
        std::make_unique<Synchronizer>(SyncPolicy(5), cloud1_sub, cloud2_sub);
    sync->registerCallback(
        boost::bind(&PointcloudConcatenate::msgCallback, this, _1, _2));
  }

  void msgCallback(const sensor_msgs::PointCloud2ConstPtr &cloud1,
                   const sensor_msgs::PointCloud2ConstPtr &cloud2) {

    sensor_msgs::PointCloud2 target_cloud1;
    pcl_ros::transformPointCloud(target_frame, *cloud1, target_cloud1, buffer);

    sensor_msgs::PointCloud2 target_cloud2;
    pcl_ros::transformPointCloud(target_frame, *cloud2, target_cloud2, buffer);

    sensor_msgs::PointCloud2 out;
    pcl::concatenatePointCloud(target_cloud1, target_cloud2, out);
    cloud_pub.publish(out);
  }

private:
  // Publisher
  ros::Publisher cloud_pub;

  // Subscriptions
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud1_sub;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud2_sub;

  // Synchronization
  using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                      sensor_msgs::PointCloud2>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  std::unique_ptr<Synchronizer> sync;

  // Transforms
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener;

  // Parameters
  std::string target_frame;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "pointcloud_concatenate");

  PointcloudConcatenate node;

  ros::spin();
  return 0;
}