// Merges consequtive point clouds

#include "pointcloud_merger/pointcloud_merger.h"

#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_ros/transforms.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>

using geometry_msgs::TransformStamped;
using sensor_msgs::PointCloud2;

namespace pointcloud_merger {

    bool PointCloudMerger::child_init(ros::NodeHandle& nh, bool& has_service) {
      // Load parameters
      output_queue_size_ = pnh_->param("output_queue_size", 10);
      transform_tolerance_ = pnh_->param("transform_tolerance", 0.5);

      min_delta_time_ = pnh_->param("min_time_between_input", 0.2);
      min_delta_translation_ = pnh_->param("min_translation_between_input", 0.2);
      min_delta_rotation_ = pnh_->param("min_rotation_between_input", 0.1);

      fixed_frame_ = pnh_->param("merge_frame", std::string());
      if (fixed_frame_.empty()) {
        NODELET_FATAL("'merge_frame' param must be set");
        return false;
      }

      publish_time_offset_ = pnh_->param("publish_point_time_offset", false);

      // Set up TF transformer
      // If use_tf is set, use the default TF for fixed to sensor transform
      // Otherwise, it is obtained from the specified odometry topic
      use_tf_ = pnh_->param("use_tf", true);
      if (use_tf_) {
        tf_.reset(new tf::TransformListener);
      }
      else {
        tf_.reset(new tf::Transformer);

        std::string odom_topic;
        if (pnh_->getParam("odom_topic", odom_topic)) {
          pnh_->getParam("odom_topic_child_frame", odom_topic_child_frame_);
          odom_sub_ = nh_->subscribe(odom_topic, 1, &PointCloudMerger::odomCB, this);
        }
        else {
          NODELET_FATAL("'odom_topic' param is not set");
          return false;
        }
      }

      return true;
    }

    void PointCloudMerger::filter(const PointCloud2::ConstPtr& input, const IndicesPtr& indices,
                                          PointCloud2& output) {
      boost::mutex::scoped_lock lock(data_mutex_);

      if (input->header.frame_id == fixed_frame_) {
        NODELET_WARN_ONCE(
            "The input frame_id and fixed frame is the same. May not work as expected");
      }

      // Process input
      sensor_frame_ = input->header.frame_id;
      input_queue_.push_back(input);
      processInputQueue();

      // Generate output point cloud
      generateMergedPointCloud(input, output);

      // Handle merge failure
      if (output.data.empty()) {
        if (last_output_) {
          // Use the last cloud if available
          NODELET_WARN("No merged cloud available. Using the last pointcloud");
          output = *last_output_;

          // Update timestamp as pcl_ros::Filter uses this timestamp to transform to
          // the output frame. Since we represent the point cloud in a fixed frame,
          // it is safe to overwrite
          output.header.stamp = input->header.stamp;
        }
        else {
          // Use input cloud when there is no data in queue
          NODELET_WARN("No merged cloud available. Using instantaneous pointcloud");
          output = *input;
          if (publish_time_offset_) {
            setIntensityValue(output, output, 0);
          }
        }
      }
      else {
        last_output_ = boost::make_shared<sensor_msgs::PointCloud2 const>(output);
      }
    }

    void PointCloudMerger::odomCB(const nav_msgs::Odometry::ConstPtr& msg) {
      // Add fixed --> base_link
      tf::StampedTransform tf_pose;
      tf::poseMsgToTF(msg->pose.pose, tf_pose);
      tf_pose.frame_id_ = fixed_frame_;  // Assume odometry is in fixed frame
      if (odom_topic_child_frame_.empty()) {
        tf_pose.child_frame_id_ = msg->child_frame_id;
      }
      else {
        tf_pose.child_frame_id_ = odom_topic_child_frame_;
      }
      tf_pose.stamp_ = msg->header.stamp;
      tf_->setTransform(tf_pose);

      // Add base_link --> sensor
      if (!sensor_frame_.empty() && tf_pose.child_frame_id_ != sensor_frame_) {
        tf::StampedTransform tf_sensor;
        try {
          tf_listener_.lookupTransform(tf_pose.child_frame_id_, sensor_frame_,
                                       msg->header.stamp, tf_sensor);
          tf_->setTransform(tf_sensor);
        }
        catch (tf::TransformException& ex) {
          NODELET_WARN_STREAM("odomCB base_link to sensor tf failure: " << ex.what());
        }
      }
    }

    // Synchronize TF and point cloud in the input queue and populate the data queue
    bool PointCloudMerger::processInputQueue() {
      if (input_queue_.empty()) {
        return false;
      }

      // Remove old data in the queue
      auto now = ros::Time::now();
      auto timeout = ros::Duration(transform_tolerance_);
      while (!input_queue_.empty() && (now - input_queue_.front()->header.stamp > timeout)) {
        input_queue_.pop_front();
      }

      if (input_queue_.empty()) {
        NODELET_WARN("No clouds in input queue - latest cloud must be older than timeout!");
      }

      // Find corresponding TFs
      bool has_update = false;
      for (auto it = input_queue_.begin(); it != input_queue_.end();) {
        auto cloud = *it;

        // Resolve transform
        tf::StampedTransform transform_tf;
        try {
          tf_->lookupTransform(fixed_frame_, sensor_frame_, cloud->header.stamp, transform_tf);
        }
        catch (tf::TransformException& ex) {
          NODELET_WARN_STREAM("processInputQueue fixed_frame to sensor frame tf failure: " << ex.what());
          ++it;
          continue;
        }
        auto transform = boost::make_shared<TransformStamped>();
        tf::transformStampedTFToMsg(transform_tf, *transform);

        // Transform point cloud
        auto cloud_transformed = boost::make_shared<PointCloud2>();
        pcl_ros::transformPointCloud(fixed_frame_, transform_tf, *cloud, *cloud_transformed);

        // Move to output queue if condition is met
        latest_data_ = Data{ .cloud = cloud_transformed, .transform = transform };
        if (isGoodData(latest_data_)) {
          output_queue_.push_back(latest_data_);
          has_update = true;
        }

        // Remove from the input queue and continue
        it = input_queue_.erase(it);
      }

      // Shrink output queue
      while (!output_queue_.empty() && output_queue_.size() > output_queue_size_) {
        output_queue_.pop_front();
      }

      return has_update;
    }

    // Check if we should add this data in the output queue
    bool PointCloudMerger::isGoodData(const Data& data) {
      // Always append the first data
      if (output_queue_.empty()) {
        return true;
      }

      // Check time since the last measurement
      if (min_delta_time_ > 0) {
        auto delta_time = data.cloud->header.stamp - output_queue_.back().cloud->header.stamp;
        if (delta_time < ros::Duration(min_delta_time_)) {
          return false;
        }
      }

      // Check motion since the last measurement
      tf::StampedTransform input_tf, last_tf;
      tf::transformStampedMsgToTF(*data.transform, input_tf);
      tf::transformStampedMsgToTF(*output_queue_.back().transform, last_tf);

      auto distance = input_tf.getOrigin().distance(last_tf.getOrigin());
      auto rotation = input_tf.getRotation().angleShortestPath(last_tf.getRotation());

      if (min_delta_translation_ > 0 && min_delta_rotation_ > 0) {
        if (distance < min_delta_translation_ && rotation < min_delta_rotation_) {
          return false;
        }
      }
      else if (min_delta_translation_ > 0) {
        if (distance < min_delta_translation_) {
          return false;
        }
      }
      else if (min_delta_rotation_ > 0) {
        if (rotation < min_delta_rotation_) {
          return false;
        }
      }

      return true;
    }

    void PointCloudMerger::generateMergedPointCloud(const PointCloud2::ConstPtr& input, PointCloud2& output) {
      if (!latest_data_.cloud) {
        return;
      }

      // Initialize with latest point cloud with odometry data
      if (publish_time_offset_) {
        auto time_offset = input->header.stamp - latest_data_.cloud->header.stamp;
        setIntensityValue(*latest_data_.cloud, output, time_offset.toSec());
      }
      else {
        output = *latest_data_.cloud;
      }

      // Concatenate point clouds
      for (auto& data : output_queue_) {
        PointCloud2 cloud = *data.cloud;
        if (publish_time_offset_) {
          auto time_offset = input->header.stamp - data.cloud->header.stamp;
          setIntensityValue(cloud, cloud, time_offset.toSec());
        }

        pcl::concatenatePointCloud(output, cloud, output);
      }

      // Transform back to the proper fixed frame if odometry-based alignment is used
      if (!use_tf_) {
        tf::StampedTransform transform_odom, transform_tf;
        tf::transformStampedMsgToTF(*latest_data_.transform, transform_odom);
        try {
          tf_listener_.waitForTransform(latest_data_.transform->header.frame_id, latest_data_.transform->child_frame_id,
                                        latest_data_.transform->header.stamp, ros::Duration(0.5));
          tf_listener_.lookupTransform(latest_data_.transform->header.frame_id, latest_data_.transform->child_frame_id,
                                       latest_data_.transform->header.stamp, transform_tf);
        }
        catch (tf::TransformException& ex) {
          NODELET_WARN_STREAM("generateMergedPointCloud tf failure: " << ex.what());
          NODELET_ERROR("Error transforming output pointcloud");
          output.data.clear();
          return;
        }

        if (!output.data.empty()) {
          output.header.frame_id = "";  // Has to be different with the target frame
          pcl_ros::transformPointCloud(fixed_frame_, transform_tf * transform_odom.inverse(), output, output);
        }
      }

      // Add latest point cloud if TF is available
      if (latest_data_.cloud->header.stamp != input->header.stamp) {
        auto input_transformed = boost::make_shared<PointCloud2>();
        if (pcl_ros::transformPointCloud(fixed_frame_, *input, *input_transformed, tf_listener_)) {
          if (publish_time_offset_) {
            setIntensityValue(*input_transformed, *input_transformed, 0);
          }
          pcl::concatenatePointCloud(output, *input_transformed, output);
        }
        else {
          NODELET_WARN("Could not add latest pointcloud as TF to %s is not available", fixed_frame_.c_str());
        }
      }

      // Update timestamp
      output.header.stamp = input->header.stamp;
    }

    void PointCloudMerger::setIntensityValue(const PointCloud2& input, PointCloud2& output, const float value) {
      output = input;

      for (sensor_msgs::PointCloud2Iterator<float> it(output, "intensity"); it != it.end(); ++it) {
        *it = value;
      }
    }

}  // namespace pointcloud_filters

PLUGINLIB_EXPORT_CLASS(pointcloud_merger::PointCloudMerger, nodelet::Nodelet)