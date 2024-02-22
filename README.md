# Pointcloud Merger

![demo](docs/demo.gif)

## Installation
``
catkin build pointcloud_merger
``

## Usage

### Fusing pointclouds from multiple time stamps
``
roslaunch pointcloud_merger pointcloud_filter_examples.launch
``

This file contains a filter chain to merge pointclouds temporally, crop and voxelize them spacially into a single point cloud.

#### Parameters for merging
* `cloud_in`: input point cloud
* `cloud_out`: output point cloud
* `odom`: odometry topic
* `fixed_frame`: odometry frame
* `min_time_between_input`: minimum time before new measurement is added [s]
* `min_translation_between_input`: minimum translation before new measurement is added [m]
* `min_rotation_between_input`: minimum rotation before new measurement is added [rad]
* `transform_tolerance`: allowable delay in TF data [s]

#### Parameters for cropping
* `reference_frame`: reference frame to crop in
* `filter_field_name`: axis to crop along
* `filter_limit_min`: min value for filter
* `filter_limit_max`: max value for filter

#### Parameters for voxelizing
* `voxel_size`: size of voxels (cube) to aggregate points [m]

## Credits
Robin Schmid, schmidrobin@outlook.com

