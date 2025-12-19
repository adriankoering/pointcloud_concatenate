#include "common.h"

#include <algorithm>
#include <numeric>
#include <set>

#include <ros/ros.h>

std::vector<std::string>
getFieldNames(const std::vector<sensor_msgs::PointField> &fields) {
  std::vector<std::string> field_set;
  for (const auto &field : fields) {
    field_set.emplace_back(field.name);
  }
  return field_set;
}

std::vector<std::string> getCommonFieldNames(
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
        common = std::move(inter);
      }
    }
  }
  return common;
}

struct PointFieldCompare {
  bool operator()(const sensor_msgs::PointField &a,
                  const sensor_msgs::PointField &b) const {
    return std::lexicographical_compare(
        a.name.begin(), a.name.end(), b.name.begin(), b.name.end(),
        [](unsigned char x, unsigned char y) {
          return std::tolower(x) < std::tolower(y);
        });
  }
};

std::vector<sensor_msgs::PointField> getCommonFields(
    const std::vector<std::vector<sensor_msgs::PointField>> &fieldss) {
  // find common set of fields between clouds based on name:
  std::vector<std::string> common_names = getCommonFieldNames(fieldss);
  std::set<std::string> common_names_set{common_names.begin(),
                                         common_names.end()};

  std::set<sensor_msgs::PointField, PointFieldCompare> common_fields_set;
  for (const auto &fields : fieldss) {
    for (const auto &field : fields) {
      // Check if this field belongs to the set of common fields
      if (common_names_set.count(field.name) > 0) {
        // insert it into the set of common fields
        // (also deduplicates the same field from multiple clouds :))
        common_fields_set.insert(field);
      }
    }
  }

  if (common_names_set.size() != common_fields_set.size()) {
    throw std::runtime_error("Deduplication didnt work?");
  }

  std::vector<sensor_msgs::PointField> out{common_fields_set.begin(),
                                           common_fields_set.end()};
  return out;
}

size_t sizeOf(uint8_t datatype) {
  switch (datatype) {
  case sensor_msgs::PointField::INT8:
  case sensor_msgs::PointField::UINT8:
    return 1;
  case sensor_msgs::PointField::INT16:
  case sensor_msgs::PointField::UINT16:
    return 2;
  case sensor_msgs::PointField::INT32:
  case sensor_msgs::PointField::UINT32:
  case sensor_msgs::PointField::FLOAT32:
    return 4;
  case sensor_msgs::PointField::FLOAT64:
    return 8;
  default:
    throw std::runtime_error("Unknown PointField datatype");
  }
}

// TODO: much simpler if we subscribe to pcl::PointCloud<pcl::PointXYZ>>...
sensor_msgs::PointCloud2
concatenate(const std::vector<sensor_msgs::PointCloud2> &clouds) {
  sensor_msgs::PointCloud2 out;
  if (clouds.empty()) {
    throw std::runtime_error("Cannot concatenate! std::vector is empty");
  }

  std::vector<std::vector<sensor_msgs::PointField>> fieldss;
  std::transform(clouds.begin(), clouds.end(), std::back_inserter(fieldss),
                 [](const auto &c) { return c.fields; });
  std::vector<sensor_msgs::PointField> common_fields = getCommonFields(fieldss);

  // for (const auto &field : common_fields) {
  //   ROS_INFO_STREAM("Field " << field.name << ":\n offset: " << field.offset
  //                            << " dtype: " << (int32_t)field.datatype
  //                            << " count: " << field.count);
  // }
  uint32_t point_step = 0;
  for (auto &field : common_fields) {
    // This field lives at the offset accumulated by all prior values
    field.offset = point_step;
    // Add the memory required by this dtype
    point_step += field.count * sizeOf(field.datatype);
  }

  // for (const auto &field : common_fields) {
  //   ROS_INFO_STREAM("Field " << field.name << ":\n offset: " << field.offset
  //                            << " dtype: " << (int32_t)field.datatype
  //                            << " count: " << field.count);
  // }

  // Allocated memory to concatenate all points of this cloud:
  std::vector<size_t> num_points;
  std::transform(clouds.begin(), clouds.end(), std::back_inserter(num_points),
                 [](const auto &c) { return c.height * c.width; });
  size_t total_num_points =
      std::accumulate(num_points.begin(), num_points.end(), 0);

  // Copy Meta-Data - assumes all clouds share the same header:
  out.header = clouds.at(0).header;

  out.height = 1;
  out.width = total_num_points;

  out.fields = common_fields;

  out.is_bigendian = clouds.at(0).is_bigendian;
  out.point_step = point_step;
  out.row_step = total_num_points * out.point_step;

  out.data.resize(out.row_step);
  // Grab a pointer into the raw memory of this vector
  // (Safe, since the vector should need to re-allocate!?)
  uint8_t *out_point = out.data.data();

  // Do the actual concatenation
  for (const auto &cloud : clouds) {
    const uint8_t *source_point = cloud.data.data();

    // Copy each field over, one after another:
    // (s.t. there a now branches in the critical copy-loop)
    for (const auto &out_field : out.fields) {
      // Determine the destination start point of this field
      // (s.t. we can then iterate over out_field += point_step)
      uint8_t *out_point_field = out_point + out_field.offset;

      // Do the same for the corresponding source field
      const uint8_t *source_point_field = nullptr;
      for (const auto &source_field : cloud.fields) {
        if (source_field.name == out_field.name) {
          source_point_field = source_point + source_field.offset;
          break;
        }
      }

      // Now that we have two pointers at the correct offsets,
      //   do a straight loop with row_step size and copy:
      size_t num_bytes = cloud.height * cloud.width * cloud.point_step;
      while (source_point_field < cloud.data.data() + num_bytes) {
        memcpy(/*dest=*/out_point_field,
               /*src=*/source_point_field,
               /*count=*/out_field.count * sizeOf(out_field.datatype));

        // Increment both points by respective step sizes
        out_point_field += out.point_step;
        source_point_field += cloud.point_step;
      }
    }

    size_t num_points = cloud.height * cloud.width;
    out_point += num_points * out.point_step;
  }

  out.is_dense = clouds.at(1).is_dense;
  return out;
}
