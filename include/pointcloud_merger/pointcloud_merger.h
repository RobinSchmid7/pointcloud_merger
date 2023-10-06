// Merges consequtive point clouds

#pragma once

#include <deque>

#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/filters/filter.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>

namespace pointcloud_merger {

    class PointCloudMerger : public pcl_ros::Filter {
    protected:
      typedef struct {
        sensor_msgs::PointCloud2::ConstPtr cloud;
        geometry_msgs::TransformStamped::ConstPtr transform;
      } Data;

      bool child_init(ros::NodeHandle& nh, bool& has_service);

      void filter(const PointCloud2::ConstPtr& input, const IndicesPtr& indices, PointCloud2& output);

    private:
      void odomCB(const nav_msgs::Odometry::ConstPtr& msg);

      bool processInputQueue();
      void generateMergedPointCloud(const PointCloud2::ConstPtr& input, PointCloud2& output);

      void setIntensityValue(const PointCloud2& input, PointCloud2& output, const float value);

      bool isGoodData(const Data& data);

      boost::shared_ptr<tf::Transformer> tf_;

      std::deque<sensor_msgs::PointCloud2::ConstPtr> input_queue_;
      std::deque<Data> output_queue_;
      Data latest_data_;
      sensor_msgs::PointCloud2::ConstPtr last_output_;

      std::string fixed_frame_;
      std::string sensor_frame_;

      int output_queue_size_;
      double transform_tolerance_;
      double min_delta_time_;
      double min_delta_translation_;
      double min_delta_rotation_;
      bool publish_time_offset_;

      bool use_tf_;
      ros::Subscriber odom_sub_;
      std::string odom_topic_child_frame_;

      ros::ServiceServer clear_map_srv_;

      boost::mutex data_mutex_;
    };

}  // namespace pointcloud_filters