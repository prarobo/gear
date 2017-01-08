#ifndef PLAYBACK_H_
#define PLAYBACK_H_

// ROS dependencies
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

// Boost dependencies
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/filesystem.hpp>

namespace fs=boost::filesystem;

namespace gear_data_handler {

class Playback {
public:
  /**
   * Constructor
   */
  Playback(ros::NodeHandle nh, ros::NodeHandle pnh, int clock_frequency=100);

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
  ros::Time parseTimeStamp(const std::string &image_name);

  /**
   * Enable playback service callback
   */
  bool startPlayback(std_srvs::Trigger::Request  &req,
                     std_srvs::Trigger::Response &res);

  /**
   * Initialize components required for playback
   */
  void initializePlayback();

  /**
   * Play the data
   */
  void play();

  /**
   * Publish image from disk
   */
  void publishImage(const std::multimap <ros::Time, std::string>::iterator &image_it);

  /**
   * Create a publisher for an image type
   */
  void createPublisher(const std::string &dir_name);

private:
  boost::shared_ptr<ros::NodeHandle> nh_, pnh_;
  ros::ServiceServer enable_playback_;
  ros::Publisher clock_pub_;
  std::map<std::string, boost::shared_ptr<image_transport::Publisher>> image_pub_;
  boost::shared_ptr<image_transport::ImageTransport> it_;

  std::string data_dir_;
  std::string subject_id_;
  std::string session_id_;
  std::string activity_id_;
  std::string condition_id_;
  int trial_id_;

  double rate_;
  int clock_frequency_;

  fs::path image_root_dir_;
  std::string image_extn_;
  std::string image_prefix_;
  std::string image_def_;
  std::multimap <ros::Time, std::string> image_info_;

  bool enabled_;
  boost::recursive_mutex enable_lock_;
};
};

#endif // PLAYBACK_H_
