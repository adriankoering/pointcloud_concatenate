#pragma once

#include <string>
#include <vector>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

std::vector<std::string>
getFieldNames(const std::vector<sensor_msgs::PointField> &fields);

std::vector<std::string> getCommonFieldNames(
    const std::vector<std::vector<sensor_msgs::PointField>> &fieldss);

std::vector<sensor_msgs::PointField> getCommonFields(
    const std::vector<std::vector<sensor_msgs::PointField>> &fieldss);

sensor_msgs::PointCloud2
concatenate(const std::vector<sensor_msgs::PointCloud2> &clouds);