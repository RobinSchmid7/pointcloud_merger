cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_merger)

add_compile_options(-std=c++17)

set(CATKIN_DEPS
  dynamic_reconfigure
  message_filters
  nodelet
  pcl_ros
  pcl_conversions
  roscpp
  tf
  tf2_ros
  sensor_msgs
  std_msgs
  cv_bridge
  image_geometry
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPS})


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_DEPS}
)

include_directories(
        include
  ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
        src/pointcloud_merger.cpp
        src/passthrough_filter.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
