#ifndef PLAYBACK_H_
#define PLAYBACK_H_

// ROS dependencies
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/static_transform_broadcaster.h>

// Boost dependencies
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/filesystem.hpp>

// OpenCV dependencies
#include <opencv2/core/core.hpp>

namespace fs=boost::filesystem;

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

  /**
   * Load pointgrey calibration information
   */
  boost::shared_ptr<sensor_msgs::CameraInfo> loadPointGreyCameraInfo(const std::string &camera_name);

  /**
   * Load kinect calibration information
   */
  boost::shared_ptr<sensor_msgs::CameraInfo> loadKinectCameraInfo(const std::string &camera_name);

  /**
   * Loading kinect calibration file from disk
   */
  bool loadKinectCalibrationFile(const std::string &filename, cv::Mat &camera_matrix, cv::Mat &distortion) const;

  /**
   * Create camera info object for kinect
   */
  void createKinectCameraInfo(const cv::Size &size, const cv::Mat &camera_matrix,
                              const cv::Mat &distortion, const cv::Mat &rotation,
                              const cv::Mat &projection,
                              boost::shared_ptr<sensor_msgs::CameraInfo> camera_info) const;

  /**
   * Get the frame id for a camera frame
   */
  std::string getFrameID(const std::string &camera_name) const;

  /**
   * Publish static kinect transforms
   */
  void publishKinectStaticTF(const std::string &kinect, const cv::Mat &rotation, const cv::Mat &translation);

  /**
   * Load kinect tf camera poses
   */
  bool loadKinectCalibrationPoseFile(const std::string &filename,
                                     cv::Mat &rotation, cv::Mat &translation) const;

  /**
   * Parse camera id from directory name
   */
  std::string getCameraID(const std::string &camera_name) const;

  /**
   * Parse camera type from directory name
   */
  std::string getCameraType(const std::string &camera_name) const;

private:
  boost::shared_ptr<ros::NodeHandle> nh_, pnh_;
  ros::ServiceServer enable_playback_;
  ros::Publisher clock_pub_;
  std::map<std::string, boost::shared_ptr<image_transport::CameraPublisher>> image_pub_;
  std::map<std::string, boost::shared_ptr<sensor_msgs::CameraInfo>> camera_info_;
  boost::shared_ptr<image_transport::ImageTransport> it_;
  tf2_ros::StaticTransformBroadcaster tf_pub_;

  std::string data_dir_;
  std::string subject_id_;
  std::string session_id_;
  std::string activity_id_;
  std::string condition_id_;
  int trial_id_;

  double rate_;
  int clock_frequency_;

  fs::path image_root_dir_, calibration_root_dir_;
  std::string image_prefix_;
  std::string image_def_;
  std::map <std::string, std::string> image_extn_;
  std::multimap <ros::Time, std::string> image_info_;

  bool enabled_;
  boost::recursive_mutex enable_lock_;
};
};

#endif // PLAYBACK_H_
