#ifndef PLAYBACK_H_
#define PLAYBACK_H_

// ROS dependencies
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>

namespace gear_data_handler {

class Playback {
public:
  /**
   * Constructor
   */
  Playback(ros::NodeHandle nh, ros::NodeHandle pnh);

  /**
   * Load all the required parameters
   */
  void loadParams();

  /**
   * Load all image information with timestamps
   */
  void loadImageInfo();

  /**
   * Parse timestamp from image name
   */
  double parseTimeStamp(const std::string &image_name);

private:
  boost::shared_ptr<ros::NodeHandle> nh_, pnh_;
  std::string data_dir_;
  std::string subject_id_;
  std::string session_id_;
  std::string activity_id_;
  std::string condition_id_;
  int trial_id_;
  double rate_;
  std::string image_extn_;
  std::string image_prefix_;
  std::multimap <double, std::string> image_info_;
};
};

#endif // PLAYBACK_H_
