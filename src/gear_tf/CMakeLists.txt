cmake_minimum_required(VERSION 2.8.3)
project(gear_tf)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf2
  tf2_ros
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy tf2 tf2_ros
)
