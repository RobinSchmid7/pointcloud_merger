// Voxelizes a poin cloud using a voxel grid filter

#include "pointcloud_merger/voxel_filter.h"

#include <pluginlib/class_list_macros.h>

namespace pointcloud_merger {

    bool VoxelFilter::child_init(ros::NodeHandle& nh, bool& has_service) {
      pnh_->getParam("voxel_size", voxel_size_);
      return true;
    }

    void VoxelFilter::filter(const PointCloud2::ConstPtr& input, const IndicesPtr& indices, PointCloud2& output) {
      // Run filter
      pcl::PCLPointCloud2::Ptr pcl_input(new pcl::PCLPointCloud2);
      pcl_conversions::toPCL(*input, *pcl_input);
      vox_.setInputCloud(pcl_input);
      vox_.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
      pcl::PCLPointCloud2 pcl_output;
      vox_.filter(pcl_output);  
      pcl_conversions::moveFromPCL(pcl_output, output);
    }

}  // namespace pointercloud_merger

PLUGINLIB_EXPORT_CLASS(pointcloud_merger::VoxelFilter, nodelet::Nodelet)


