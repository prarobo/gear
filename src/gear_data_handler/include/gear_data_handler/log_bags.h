#ifndef LOG_BAGS_H_
#define LOG_BAGS_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <gear_data_handler/logger.h>
#include <rosbag/bag.h>

namespace gear_data_handler {

class BagLogger : public Logger {

public:
  BagLogger();

  ~BagLogger() {};

  /**
   * Overrides default directory initialization
   */
  virtual void initializeSessionDirectories();

  /**
   * Overrides default toggling of logger
   */
  virtual bool toggleLogger(std_srvs::SetBool::Request  &req,
                            std_srvs::SetBool::Response &res);

protected:
  ros::Subscriber sub_;
  rosbag::Bag bag_;

}; //class BagLogger

} // namespace gear_data_handler

#endif // LOG_BAGS_H_
