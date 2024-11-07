# Merge Pointclouds and Laserscans

### **Dependencies**

This package depends on the following libraries

* `pcl`
* `pcl_ros`
* TODO: laser geometry ... 


## `pointcloud_concatenate`

This package provides a node and nodelet - either one combines two pointclouds into one pointcloud in a common reference frame.

### Subscribers

* `cloud1` - [`sensor_msgs/PointCloud2`]  
  The first pointcloud to add to the output.
* `cloud2` - [`sensor_msgs/PointCloud2`]  
  The second pointcloud to add to the output.

### Publisher

* `cloud_out` - [`sensor_msgs/PointCloud2`]  
  The concatenated pointcloud.

### Parameter

* `target_frame` - [a valid frame_id]  
  Sets the frame_id which the pointclouds will be collected in before concatenation. `cloud_out` will be in this frame.  