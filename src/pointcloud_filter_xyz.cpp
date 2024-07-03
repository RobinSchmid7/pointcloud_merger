#include "pointcloud_merger/pointcloud_filter_xyz.h"
#include <pluginlib/class_list_macros.h>

namespace pointcloud_merger
{

void PointCloudXYZFilterNodelet::onInit()
{
    nh_ = getNodeHandle();
    private_nh_ = getPrivateNodeHandle();

    // Get input and output topics from parameter server
    std::string input_topic, output_topic;
    private_nh_.getParam("input_topic", input_topic);
    private_nh_.getParam("output_topic", output_topic);

    // Initialize subscriber and publisher
    sub_ = nh_.subscribe(input_topic, 1, &PointCloudXYZFilterNodelet::pointcloudCallback, this);
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic, 1);
}

void PointCloudXYZFilterNodelet::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // Convert ROS PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*msg, pcl_cloud);

    // Convert PCL PointCloud to ROS PointCloud2
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(pcl_cloud, output);
    output.header = msg->header;

    // Publish filtered point cloud
    pub_.publish(output);
}

} // namespace pointcloud_merger

PLUGINLIB_EXPORT_CLASS(pointcloud_merger::PointCloudXYZFilterNodelet, nodelet::Nodelet)
