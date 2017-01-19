cmake_minimum_required(VERSION 2.8.3)
project(gear_data_handler)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  nodelet
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
  image_transport
  pcl_ros
  pcl_conversions
  apriltags_ros
  ar_track_alvar_msgs
  rosbag
  camera_info_manager
  tf2
  tf2_ros
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Use C++11
add_definitions(-std=gnu++0x)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES gear_data_handler
  CATKIN_DEPENDS pluginlib nodelet cv_bridge roscpp rospy sensor_msgs std_msgs 
  std_srvs pcl_ros pcl_conversions apriltags_ros rosbag ar_track_alvar_msgs
  camera_info_manager tf2 tf2_ros
  # DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


# Declare a C++ library
add_library(data_handler
  src/log_images.cpp
  src/log_pointcloud.cpp
  src/log_tracking.cpp
  src/synchronize_images.cpp
  src/logger.cpp
)

target_link_libraries(data_handler
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(playback src/playback.cpp)
target_link_libraries(playback ${catkin_LIBRARIES})
