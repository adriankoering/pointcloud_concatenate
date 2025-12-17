#pragma once

#include <string>
#include <vector>

#include <sensor_msgs/PointField.h>

std::vector<std::string>
getFieldNames(const std::vector<sensor_msgs::PointField> &fields);

std::vector<std::string>
getCommonFields(const std::vector<sensor_msgs::PointField> &fields1,
                const std::vector<sensor_msgs::PointField> &fields2);

std::vector<std::string>
getCommonFields(const std::vector<sensor_msgs::PointField> &fields1,
                const std::vector<sensor_msgs::PointField> &fields2,
                const std::vector<sensor_msgs::PointField> &fields3,
                const std::vector<sensor_msgs::PointField> &fields4);

std::vector<std::string>
getCommonFields(const std::vector<sensor_msgs::PointField> &fields1,
                const std::vector<sensor_msgs::PointField> &fields2,
                const std::vector<sensor_msgs::PointField> &fields3,
                const std::vector<sensor_msgs::PointField> &fields4,
                const std::vector<sensor_msgs::PointField> &fields5,
                const std::vector<sensor_msgs::PointField> &fields6);

std::vector<std::string>
getCommonFields(const std::vector<sensor_msgs::PointField> &fields1,
                const std::vector<sensor_msgs::PointField> &fields2,
                const std::vector<sensor_msgs::PointField> &fields3,
                const std::vector<sensor_msgs::PointField> &fields4,
                const std::vector<sensor_msgs::PointField> &fields5,
                const std::vector<sensor_msgs::PointField> &fields6,
                const std::vector<sensor_msgs::PointField> &fields7,
                const std::vector<sensor_msgs::PointField> &fields8);