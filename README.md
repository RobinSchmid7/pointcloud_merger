# Pointcloud Merger

## Installation
``
catkin build pointcloud_merger
``

## Usage

### A) Fusing pointclouds from multiple time stamps
``
roslaunch pointcloud_merger pointcloud_merger.launch
``

Fuses multiple pointclouds from multiple time stamps into a single point cloud.

#### Parameters for merging
* `cloud_in`: input point cloud
* `cloud_out`: output point cloud
* `odom`: odometry topic
* `fixed_frame`: odometry frame
* `min_time_between_input`: minimum time before new measurement is added [m]
* `min_translation_between_input`: minimum translation before new measurement is added [m]
* `min_rotation_between_input`: minimum rotation before new measurement is added [rad]
* `transform_tolerance`: allowable delay in TF data [s]

### B) Fusing pointclouds from multiple time stamps and crop them spacially
``
roslaunch pointcloud_merger pointcloud_merger_crop.launch
``

Fuses multiple poinclouds from multiple time stamps into a single point cloud and crops thme from a given bounding box.

#### Parameters for merging
* `cloud_in`: input point cloud
* `cloud_out`: output point cloud
* `odom`: odometry topic
* `fixed_frame`: odometry frame
* `min_time_between_input`: minimum time before new measurement is added [m]
* `min_translation_between_input`: minimum translation before new measurement is added [m]
* `min_rotation_between_input`: minimum rotation before new measurement is added [rad]
* `transform_tolerance`: allowable delay in TF data [s]

#### Parameters for cropping
Add an arbitrary number of `PassThroughFilter` to crop pointcloud along different axis in the launch file.
* `reference_frame`: reference frame to crop in
* `filter_field_name`: axis to crop along
* `filter_limit_min`: min value for filter
* `filter_limit_max`: max value for filter

## Credits
Robin Schmid, schmidrobin@outlook.com

