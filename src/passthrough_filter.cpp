// Filters out points outside an interval on a given axis

#include "pointcloud_merger/passthrough_filter.h"

#include <pluginlib/class_list_macros.h>

namespace pointcloud_merger {

    bool PassThroughFilter::child_init(ros::NodeHandle& nh, bool& has_service) {
      pnh_->getParam("reference_frame", tf_ref_frame_);
      pnh_->getParam("filter_field_name", field_name_);
      pnh_->getParam("filter_limit_min", relative_filter_min_);
      pnh_->getParam("filter_limit_max", relative_filter_max_);
      return true;
    }

    void PassThroughFilter::filter(const PointCloud2::ConstPtr& input, const IndicesPtr& indices, PointCloud2& output) {
      // Get pose of reference frame
      tf::StampedTransform transform;
      try {
        tf_listener_.waitForTransform(input->header.frame_id, tf_ref_frame_, input->header.stamp, ros::Duration(1.0));
        tf_listener_.lookupTransform(input->header.frame_id, tf_ref_frame_, input->header.stamp, transform);
      }
      catch (tf::TransformException& ex) {
        ROS_ERROR("TF error : %s", ex.what());
        output = *input;
        return;
      }

      // Configure filter
      double base_value = 0;
      if (field_name_ == "x") {
        base_value = transform.getOrigin().x();
      }
      else if (field_name_ == "y") {
        base_value = transform.getOrigin().y();
      }
      else if (field_name_ == "z") {
        base_value = transform.getOrigin().z();
      }
      else {
        ROS_ERROR("Unsupported field name: %s", field_name_.c_str());
        return;
      }

      impl_.setFilterFieldName(field_name_);
      impl_.setFilterLimits(base_value + relative_filter_min_, base_value + relative_filter_max_);

      // Run filter
      pcl::PCLPointCloud2::Ptr pcl_input(new pcl::PCLPointCloud2);
      pcl_conversions::toPCL(*input, *pcl_input);
      impl_.setInputCloud(pcl_input);
      impl_.setIndices(indices);
      pcl::PCLPointCloud2 pcl_output;
      impl_.filter(pcl_output);
      pcl_conversions::moveFromPCL(pcl_output, output);
    }

}  // namespace passthrough_filter

PLUGINLIB_EXPORT_CLASS(pointcloud_merger::PassThroughFilter, nodelet::Nodelet)