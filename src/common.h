#pragma once

#include <string>
#include <vector>

#include <sensor_msgs/PointField.h>

std::vector<std::string>
getFieldNames(const std::vector<sensor_msgs::PointField> &fields);

std::vector<std::string> getCommonFields(
    const std::vector<std::vector<sensor_msgs::PointField>> &fieldss);
