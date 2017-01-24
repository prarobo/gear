#ifndef LOGGER_H_
#define LOGGER_H_

// ROS Dependencies
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

// Boost Dependencies
#include <boost/filesystem.hpp>
#include <boost/thread/mutex.hpp>

namespace gear_data_handler {

class Logger {

public:
  Logger();

  virtual ~Logger() {};

  /**
   * Initialization after onInit
   */
  void initialize(ros::NodeHandle nh, ros::NodeHandle pnh);

  /**
   * Initialized the directories where images are to be stored
   */
  virtual void initializeSessionDirectories();

  /**
   * Image logger service callback
   */
  virtual bool toggleLogger(std_srvs::SetBool::Request  &req,
                            std_srvs::SetBool::Response &res);

  /**
   * Session information service callback
   */
  virtual bool setSessionInfo(std_srvs::Trigger::Request  &req,
                              std_srvs::Trigger::Response &res);

  /**
   * Publishing image count
   */
  virtual void publishCount();

  /**
   * Get the path of a file to be saved
   */
  virtual boost::filesystem::path getFilePath(std::string sec, std::string nsec);

protected:
  boost::shared_ptr<ros::NodeHandle> nh_, pnh_;
  ros::Publisher count_pub_;
  ros::ServiceServer toggle_logger_;
  ros::ServiceServer session_info_;

  std::string subject_id_;
  std::string session_id_;
  std::string activity_id_;
  std::string condition_id_;
  int trial_id_;

  std::string sensor_id_;
  std::string data_dir_;
  std::string image_extn_;
  std::string image_type_;
  std::string image_prefix_;

  bool enable_;
  boost::recursive_mutex enable_lock_, session_param_lock_;

  int count_;

  boost::filesystem::path image_dir_;
}; //class Logger

} // namespace image_handler

#endif // LOGGER_H_
