// Voxelizes a poin cloud using a voxel grid filter

#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/filters/filter.h>
#include <ros/ros.h>

namespace pointcloud_merger {

    class VoxelFilter : public pcl_ros::Filter {
    protected:
      bool child_init(ros::NodeHandle& nh, bool& has_service);

      void filter(const PointCloud2::ConstPtr& input, const IndicesPtr& indices, PointCloud2& output);

    private:
      double voxel_size_;
      pcl::VoxelGrid<pcl::PCLPointCloud2> vox_;
    };

}  // namespace pointcloud_merger