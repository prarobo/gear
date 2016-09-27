#ifndef LOG_IMAGES_H_
#define LOG_IMAGES_H_

// ROS Dependencies
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

// Boost Dependencies
#include <boost/filesystem.hpp>

// C++ Dependencies
#include <pthread.h>

namespace gear_image_handler {

class ImageLogger : public nodelet::Nodelet {

public:
  ImageLogger();

  ~ImageLogger() {};

  /**
     Implements the Nodelet interface
   **/
  void onInit();

  /**
   * Initialized the directories where images are to be stored
   */
  void initializeSessionDirectories();

  /**
     Image callback function
   **/
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  /**
   * Image logger service callback
   */
  bool toggleLogger(std_srvs::SetBool::Request  &req,
                    std_srvs::SetBool::Response &res);

  /**
   * Session information service callback
   */
  bool setSessionInfo(std_srvs::Trigger::Request  &req,
                      std_srvs::Trigger::Response &res);

private:
  ros::Subscriber image_sub_;
  ros::Publisher image_count_pub_;
  ros::ServiceServer toggle_logger_;
  ros::ServiceServer session_info_;

  std::string session_id_;
  std::string activity_id_;
  std::string trial_id_;

  std::string sensor_id_;
  std::string data_dir_;
  std::string image_extn_;
  std::string image_type_;
  std::string encoding_;

  bool enable_;
  pthread_mutex_t enable_lock_, session_param_lock_;

  int image_count_;

  boost::filesystem::path image_dir_;
}; //class ImageLogger

PLUGINLIB_DECLARE_CLASS(gear_image_handler, ImageLogger,
                        gear_image_handler::ImageLogger,
                        nodelet::Nodelet);

} // namespace image_handler

#endif // LOG_IMAGES_H_
