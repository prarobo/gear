cmake_minimum_required(VERSION 2.8.3)
project(gear_diagnostics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  diagnostic_updater
)
find_package(Boost REQUIRED COMPONENTS system)

# Use C++11
add_definitions(-std=gnu++0x)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES gear_diagnostics
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs diagnostic_updater
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
)

add_executable(camera_monitor src/camera_monitor.cpp src/message_monitor.cpp)

target_link_libraries(camera_monitor
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(logger_monitor src/logger_monitor.cpp src/message_monitor.cpp)

target_link_libraries(logger_monitor
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
