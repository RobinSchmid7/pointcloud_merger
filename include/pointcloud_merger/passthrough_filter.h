// Filters out points outside an interval on a given axis

#pragma once

#include <pcl/filters/passthrough.h>
#include <pcl_ros/filters/filter.h>
#include <ros/ros.h>

namespace pointcloud_merger {

    class PassThroughFilter : public pcl_ros::Filter {
    protected:
      bool child_init(ros::NodeHandle& nh, bool& has_service);

      void filter(const PointCloud2::ConstPtr& input, const IndicesPtr& indices, PointCloud2& output);

    private:
      std::string tf_ref_frame_;
      std::string field_name_;
      double relative_filter_min_, relative_filter_max_;
      pcl::PassThrough<pcl::PCLPointCloud2> impl_;
    };

}  // namespace passthrough_filter