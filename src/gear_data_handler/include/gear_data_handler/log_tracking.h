#ifndef LOG_TRACKING_H_
#define LOG_TRACKING_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <gear_data_handler/log_bags.h>
#include <rosbag/bag.h>

namespace gear_data_handler {

class TrackingLogger : public nodelet::Nodelet, public BagLogger {

public:
  TrackingLogger();

  ~TrackingLogger() {};

  /**
   * Implements the Nodelet interface
   **/
  void onInit();

  /**
   * April-tag tracking callback function
   **/
  void aprilTrackingCallback(const apriltags_ros::AprilTagDetectionArray& msg);

  /**
   * AR-tag tracking callback function
   **/
  void arTrackingCallback(const ar_track_alvar_msgs::AlvarMarkers& msg);

private:
  std::string tag_type_;

}; //class PointcloudLogger

PLUGINLIB_DECLARE_CLASS(gear_data_handler, TrackingLogger,
                        gear_data_handler::TrackingLogger,
                        nodelet::Nodelet);

} // namespace gear_data_handler

#endif // LOG_TRACKING_H_
