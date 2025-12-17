#include "common_fields.h"

#include <algorithm>

std::vector<std::string>
getFieldNames(const std::vector<sensor_msgs::PointField> &fields) {
  std::vector<std::string> field_set;
  for (const auto &field : fields) {
    field_set.emplace_back(field.name);
  }
  return field_set;
}

std::vector<std::string>
getCommonFields(const std::vector<sensor_msgs::PointField> &fields1,
                const std::vector<sensor_msgs::PointField> &fields2) {

  auto fn1 = getFieldNames(fields1);
  auto fn2 = getFieldNames(fields2);

  // determine pairwise common fields
  std::vector<std::string> c;
  std::set_intersection(fn1.begin(), fn1.end(), fn2.begin(), fn2.end(),
                        std::back_inserter(c));
  return c;
}

std::vector<std::string>
getCommonFields(const std::vector<sensor_msgs::PointField> &fields1,
                const std::vector<sensor_msgs::PointField> &fields2,
                const std::vector<sensor_msgs::PointField> &fields3,
                const std::vector<sensor_msgs::PointField> &fields4) {

  auto fn1 = getFieldNames(fields1);
  auto fn2 = getFieldNames(fields2);
  auto fn3 = getFieldNames(fields3);
  auto fn4 = getFieldNames(fields4);

  // determine pairwise common fields
  std::vector<std::string> c12;
  std::set_intersection(fn1.begin(), fn1.end(), fn2.begin(), fn2.end(),
                        std::back_inserter(c12));
  std::vector<std::string> c34;
  std::set_intersection(fn3.begin(), fn3.end(), fn4.begin(), fn4.end(),
                        std::back_inserter(c34));
  std::vector<std::string> c;
  std::set_intersection(c12.begin(), c12.end(), c34.begin(), c34.end(),
                        std::back_inserter(c));
  return c;
}

std::vector<std::string>
getCommonFields(const std::vector<sensor_msgs::PointField> &fields1,
                const std::vector<sensor_msgs::PointField> &fields2,
                const std::vector<sensor_msgs::PointField> &fields3,
                const std::vector<sensor_msgs::PointField> &fields4,
                const std::vector<sensor_msgs::PointField> &fields5,
                const std::vector<sensor_msgs::PointField> &fields6) {

  auto fn1 = getFieldNames(fields1);
  auto fn2 = getFieldNames(fields2);
  auto fn3 = getFieldNames(fields3);
  auto fn4 = getFieldNames(fields4);
  auto fn5 = getFieldNames(fields5);
  auto fn6 = getFieldNames(fields6);

  // determine pairwise common fields
  std::vector<std::string> c12;
  std::set_intersection(fn1.begin(), fn1.end(), fn2.begin(), fn2.end(),
                        std::back_inserter(c12));
  std::vector<std::string> c34;
  std::set_intersection(fn3.begin(), fn3.end(), fn4.begin(), fn4.end(),
                        std::back_inserter(c34));
  std::vector<std::string> c56;
  std::set_intersection(fn5.begin(), fn5.end(), fn6.begin(), fn6.end(),
                        std::back_inserter(c56));

  std::vector<std::string> c1234;
  std::set_intersection(c12.begin(), c12.end(), c34.begin(), c34.end(),
                        std::back_inserter(c1234));
  std::vector<std::string> c;
  std::set_intersection(c1234.begin(), c1234.end(), c56.begin(), c56.end(),
                        std::back_inserter(c));
  return c;
}

std::vector<std::string>
getCommonFields(const std::vector<sensor_msgs::PointField> &fields1,
                const std::vector<sensor_msgs::PointField> &fields2,
                const std::vector<sensor_msgs::PointField> &fields3,
                const std::vector<sensor_msgs::PointField> &fields4,
                const std::vector<sensor_msgs::PointField> &fields5,
                const std::vector<sensor_msgs::PointField> &fields6,
                const std::vector<sensor_msgs::PointField> &fields7,
                const std::vector<sensor_msgs::PointField> &fields8) {

  auto fn1 = getFieldNames(fields1);
  auto fn2 = getFieldNames(fields2);
  auto fn3 = getFieldNames(fields3);
  auto fn4 = getFieldNames(fields4);
  auto fn5 = getFieldNames(fields5);
  auto fn6 = getFieldNames(fields6);
  auto fn7 = getFieldNames(fields7);
  auto fn8 = getFieldNames(fields8);

  // determine pairwise common fields
  std::vector<std::string> c12;
  std::set_intersection(fn1.begin(), fn1.end(), fn2.begin(), fn2.end(),
                        std::back_inserter(c12));
  std::vector<std::string> c34;
  std::set_intersection(fn3.begin(), fn3.end(), fn4.begin(), fn4.end(),
                        std::back_inserter(c34));
  std::vector<std::string> c56;
  std::set_intersection(fn5.begin(), fn5.end(), fn6.begin(), fn6.end(),
                        std::back_inserter(c56));
  std::vector<std::string> c78;
  std::set_intersection(fn7.begin(), fn7.end(), fn8.begin(), fn8.end(),
                        std::back_inserter(c78));

  std::vector<std::string> c1234;
  std::set_intersection(c12.begin(), c12.end(), c34.begin(), c34.end(),
                        std::back_inserter(c1234));
  std::vector<std::string> c5678;
  std::set_intersection(c56.begin(), c56.end(), c78.begin(), c78.end(),
                        std::back_inserter(c5678));
  std::vector<std::string> c;
  std::set_intersection(c1234.begin(), c1234.end(), c5678.begin(), c5678.end(),
                        std::back_inserter(c));
  return c;
}
