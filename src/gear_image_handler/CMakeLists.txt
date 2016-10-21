cmake_minimum_required(VERSION 2.8.3)
project(gear_image_handler)

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
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)

# Use C++11
add_definitions(-std=gnu++0x)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES gear_image_handler
  CATKIN_DEPENDS pluginlib nodelet cv_bridge roscpp rospy sensor_msgs std_msgs std_srvs pcl_ros
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
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# Declare a C++ library
add_library(image_handler
  src/log_images.cpp
  src/voxelizer.cpp
)

target_link_libraries(image_handler
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

## Declare a cpp executable
add_executable(test_pcl
    src/test_pcl.cpp
)
target_link_libraries (test_pcl ${PCL_LIBRARIES})

