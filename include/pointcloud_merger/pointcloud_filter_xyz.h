// Only keeps the XYZ fields of the pointcloud and drops the rest

#pragma once

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pointcloud_merger
{

class PointCloudXYZFilterNodelet : public nodelet::Nodelet
{
public:
    PointCloudXYZFilterNodelet() = default;

private:
    virtual void onInit() override;
    void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
};

} // namespace pointcloud_merger