#include "common.h"

#include <algorithm>

std::vector<std::string>
getFieldNames(const std::vector<sensor_msgs::PointField> &fields) {
  std::vector<std::string> field_set;
  for (const auto &field : fields) {
    field_set.emplace_back(field.name);
  }
  return field_set;
}

std::vector<std::string> getCommonFields(
    const std::vector<std::vector<sensor_msgs::PointField>> &fieldss) {

  std::vector<std::string> common;
  for (const auto &fields : fieldss) {
    if (common.empty()) {
      // First cloud initializes the space of common fields
      common = getFieldNames(fields);
    } else {
      // Later clouds remove fields from set of commons
      auto next = getFieldNames(fields);

      // Empty output container for set intersection
      std::vector<std::string> inter;
      std::set_intersection(next.begin(), next.end(), common.begin(),
                            common.end(), std::back_inserter(inter));

      // If intersection is empty
      if (inter.empty()) {
        throw std::runtime_error("Clouds have no fields in common");
      } else {
        common = inter;
      }
    }
  }
  return common;
}
