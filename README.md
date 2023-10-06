# Point Cloud Merger

## Installation
``
catkin build pointcloud_merger
``

## Usage
``
roslaunch pointcloud_merger pointcloud_merger.launch
``
Fuses multiple pointclouds from multiple time stamps into a single point cloud.

``
roslaunch pointcloud_merger pointcloud_merger_crop.launch
``
Fuses multiple poinclouds from multiple time stamps into a single point cloud and crops thme from a given bounding box.
