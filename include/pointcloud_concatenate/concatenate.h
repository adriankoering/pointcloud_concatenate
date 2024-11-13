#pragma once


template <typename T>
T concatenate(const std::vector<T>& clouds)
{
    T concat_cloud;
    for (const auto& cloud: clouds)
        pcl::concatenatePointCloud(cloud, concat_cloud, concat_cloud);

    return concat_cloud;
}

