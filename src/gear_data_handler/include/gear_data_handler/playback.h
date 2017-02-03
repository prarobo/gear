#ifndef PLAYBACK_H_
#define PLAYBACK_H_

// ROS dependencies
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <audio_common_msgs/StampedAudioData.h>
#include <rosgraph_msgs/Clock.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <gear_data_handler/TimeExtent.h>
#include <dynamic_reconfigure/server.h>

// Boost dependencies
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/filesystem.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <boost/atomic.hpp>
#include <boost/thread/thread.hpp>
#include <gear_data_handler/PlaybackConfig.h>

// OpenCV dependencies
#include <opencv2/core/core.hpp>

namespace fs=boost::filesystem;

namespace gear_data_handler {

class MessageWrapper {
public:
  /**
   * Default constructor
   */
  MessageWrapper();

  /**
   * Set audio data
   */
  bool setAudioData(const audio_common_msgs::StampedAudioDataConstPtr &audio_msg);

  /**
   * Set image data
   */
  bool setImageData(const sensor_msgs::ImageConstPtr &image_msg, const std::string &pub_name_);

  /**
   * Set clock data
   */
  bool setClockData(const rosgraph_msgs::ClockConstPtr &clock_msg);

  /**
   * Accessor for getting audio data
   */
  audio_common_msgs::StampedAudioDataConstPtr getAudioData() const;

  /**
   * Accessor for getting image data
   */
  sensor_msgs::ImageConstPtr getImageData() const;

  /**
   * Accessor for getting clock data
   */
  rosgraph_msgs::ClockConstPtr getClockData() const;

  /**
   * Accessor for publisher name
   */
  std::string getPublisherName() const {return pub_name_;}

  /**
   * Get the timestamp of wrapped message
   */
  ros::Time getMessageTime();

  /**
   * Functions to check data availability
   */
  bool hasData() const {return has_data_;};
  bool hasImageData() const {return has_image_data_;};
  bool hasAudioData() const {return has_audio_data_;};
  bool hasClockData() const {return has_clock_data_;};

private:
  /**
   * Update the status of the data availability in the container
   */
  void updateStatus();

  audio_common_msgs::StampedAudioDataConstPtr audio_msg_;
  sensor_msgs::ImageConstPtr image_msg_;
  rosgraph_msgs::ClockConstPtr clock_msg_;
  std::string pub_name_;
  bool has_image_data_;
  bool has_audio_data_;
  bool has_clock_data_;
  bool has_data_;
};

class Playback: public nodelet::Nodelet {
public:
  /**
   * Constructor
   */
  Playback();

  /**
   * Initialize playback
   */
  void initialize();

  /**
   * Implements the Nodelet interface
   **/
  void onInit();

  /**
   * Load all the required parameters
   */
  void loadParams();

  /**
   * Load all image information with timestamps
   */
  void loadImageInfo();

  /**
   * Load all clock message information
   */
  void loadClockInfo();

  /**
   * Load all audio message information
   */
  void loadAudioInfo();

  /**
   * Publish the time extents of playback
   */
  bool publishTimeExtent(gear_data_handler::TimeExtent::Request &req,
                         gear_data_handler::TimeExtent::Response &res);

  /**
   * Parse timestamp from image name
   */
  ros::Time parseTimeStamp(const std::string &image_name);

  /**
   * Start playback service callback
   */
  bool startPlayback(std_srvs::Trigger::Request  &req,
                     std_srvs::Trigger::Response &res);

  /**
   * Stop playback service callback
   */
  bool stopPlayback(std_srvs::Trigger::Request  &req,
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
  void publishImageOld(const std::multimap <ros::Time, std::string>::iterator &image_it);

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

  /**
   * Create transform message
   */
  geometry_msgs::TransformStamped createTransformMsg(const std::string &parent_frame,
                                                     const std::string &child_frame,
                                                     const tf2::Vector3 &trans = tf2::Vector3(0,0,0),
                                                     const tf2::Quaternion &quat = tf2::Quaternion(0,0,0,1),
                                                     const ros::Time &stamp = ros::Time::now()) const;

  /**
   * Publish the static tf that links camera frame with world frame
   */
  void publishLinkStaticTF(const std::string &camera_id);

  /**
   * Pickup message handler
   */
  void pickupMsg();

  /**
   * Publish message from message wrapper
   */
  void publishMsg(const MessageWrapper &msg_wrapper) const;

  /**
   * Load message handler
   */
  void loadMsg();

  /**
   * Create image message from raw data
   */
  sensor_msgs::ImageConstPtr
  createImageMsg(const std::multimap <ros::Time, std::string>::iterator &image_it);

  void reconfigureCallback(gear_data_handler::PlaybackConfig &config, uint32_t level);

private:
  boost::shared_ptr<ros::NodeHandle> nh_, pnh_;
  ros::ServiceServer start_playback_, pause_playback_, stop_playback_;
  ros::Publisher clock_pub_;
  std::map<std::string, boost::shared_ptr<image_transport::CameraPublisher>> image_pub_;
  std::map<std::string, boost::shared_ptr<sensor_msgs::CameraInfo>> camera_info_;
  ros::Publisher audio_pub_;
  boost::shared_ptr<image_transport::ImageTransport> it_;
  tf2_ros::StaticTransformBroadcaster tf_pub_;
  dynamic_reconfigure::Server<gear_data_handler::PlaybackConfig> server_;
  ros::ServiceServer time_extent_service_;

  std::string data_dir_;
  std::string subject_id_;
  std::string session_id_;
  std::string activity_id_;
  std::string condition_id_;
  int trial_id_;

  boost::atomic<double> rate_;
  int clock_frequency_;

  int published_msgs_, loaded_msgs_;
  size_t queue_size_;
  boost::shared_ptr<boost::lockfree::spsc_queue<MessageWrapper>> msg_queue_;

  bool publish_tf_;
  std::vector<geometry_msgs::TransformStamped> transforms_;

  bool initialized_;

  fs::path image_root_dir_, calibration_root_dir_, trial_root_dir_;
  std::string image_prefix_;
  std::string image_def_color_, image_def_depth_;
  std::map <std::string, std::string> image_extn_;

  std::multimap <ros::Time, std::string> image_info_;
  std::vector <ros::Time> clock_info_;
  std::multimap <ros::Time, audio_common_msgs::StampedAudioDataConstPtr> audio_info_;

  boost::atomic<bool> finished_, started_;
  boost::shared_ptr<boost::thread> load_thread_, pickup_thread_;
};

PLUGINLIB_DECLARE_CLASS(gear_data_handler, Playback,
                        gear_data_handler::Playback,
                        nodelet::Nodelet);
};

#endif // PLAYBACK_H_
