#include <gear_data_handler/playback.h>

// ROS dependencies
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// Boost dependencies
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>

// OpenCV dependencies
#include <opencv2/highgui/highgui.hpp>

#define KINECT_CALIB_CAMERA_MATRIX "cameraMatrix"
#define KINECT_CALIB_DISTORTION "distortionCoefficients"
#define KINECT_HD_SIZE_COLOR cv::Size(1920, 1080)
#define KINECT_SD_SIZE_DEPTH cv::Size(512, 424)
#define KINECT_TF_RGB_OPT_FRAME "_rgb_optical_frame"
#define KINECT_TF_IR_OPT_FRAME "_ir_optical_frame"
#define LINK_TF_FRAME "_link"
#define KINECT_CALIB_ROTATION "rotation"
#define KINECT_CALIB_TRANSLATION "translation"
#define POSES_FILE_SUFFIX "_pose"
#define SPSC_QUEUE_SIZE 250

namespace gear_data_handler {
Playback::Playback():
    publish_tf_(true), queue_size_(SPSC_QUEUE_SIZE),
    clock_frequency_(50), rate_(1.0), trial_id_(1) {
  initialize();
};

void Playback::initialize() {
  started_ = false;
  finished_ = false;
  published_msgs_ = 0;
  loaded_msgs_ = 0;

  // Clear all message information
  image_info_.clear();
  clock_info_.clear();
  audio_info_.clear();

  // Initialize message queue
  msg_queue_.reset(new boost::lockfree::spsc_queue<MessageWrapper>(queue_size_));
}

void Playback::onInit() {
  // Get node handles
  nh_.reset(new ros::NodeHandle(getNodeHandle()));
  pnh_.reset(new ros::NodeHandle(getPrivateNodeHandle()));

  // Time publisher
  clock_pub_ = nh_->advertise<rosgraph_msgs::Clock>("clock", 1);

  // Image transport
  it_.reset(new image_transport::ImageTransport(*nh_));

  // Create service to start playback
  start_playback_ = nh_->advertiseService("start_playback", &Playback::startPlayback, this);

  // Create service to stop playback
  stop_playback_ = nh_->advertiseService("stop_playback", &Playback::stopPlayback, this);

  // Create audio publisher
  audio_pub_ = nh_->advertise<audio_common_msgs::StampedAudioData>("/audio/audio", 10);

  // Create service to send time extents
  time_extent_service_ = nh_->advertiseService("playback_time_extents", &Playback::publishTimeExtent, this);

  // Setting up dynamic reconfigure
  server_.setCallback(boost::bind(&Playback::reconfigureCallback, this, _1, _2));
}

bool Playback::stopPlayback(std_srvs::Trigger::Request  &req,
                            std_srvs::Trigger::Response &res) {
  if(started_) {
    // Stop the threads
    load_thread_->interrupt();
    pickup_thread_->interrupt();

    // Wait for things to finish cleanly
    if (load_thread_->joinable()) {
      ROS_INFO("[Playback] Waiting for load thread to stop ...");
      load_thread_->join();
    } else {
      ROS_WARN("[Playback] Failed to join load");
    }
    if (pickup_thread_->joinable()) {
      ROS_INFO("[Playback] Waiting for pickup thread to stop ...");
      pickup_thread_->join();
    } else {
      ROS_WARN("[Playback] Failed to join pickup");
    }

    ROS_INFO("[Playback] Stopped playback");
    res.message = "Playback stopped";
    res.success = true;
  } else {
    ROS_INFO("[Playback] Playback not started yet cannot stop, doing nothing!");
    res.message = "Playback not started yet cannot stop, doing nothing!";
    res.success = false;
  }
  started_ = false;
  ROS_INFO_STREAM("[Playback] Started value: "<<started_);
  return true;
}

bool Playback::publishTimeExtent(gear_data_handler::TimeExtent::Request &req,
                                 gear_data_handler::TimeExtent::Response &res) {
  if(clock_info_.size() != 0) {
    res.start_time = clock_info_.front();
    res.stop_time = clock_info_.back();
    res.success = true;
    return true;
  } else {
    res.success = false;
    return false;
  }
}

void Playback::initializePlayback() {
  //Load associated parameters
  loadParams();

  //Set the trial root directory where all data is present
  trial_root_dir_ = fs::path(data_dir_)/fs::path(subject_id_)/fs::path(session_id_)/
                    fs::path(activity_id_+std::string("_")
                    +condition_id_+std::string("_")+std::to_string(trial_id_));

  //Load image information
  ROS_INFO("[Playback] Loading image information ...");
  loadImageInfo();
  ROS_INFO("[Playback] Loaded image information: %d directories, %d images",
           int(image_extn_.size()),int(image_info_.size()));

  //Load audio information
  ROS_INFO("[Playback] Loading audio information ...");
  loadAudioInfo();
  ROS_INFO("[Playback] Loaded audio information: %d messages", int(audio_info_.size()));

  //Load clock information
  ROS_INFO("[Playback] Loading clock information ...");
  loadClockInfo();
  ROS_INFO("[Playback] Loaded clock information: %d messages", int(clock_info_.size()));
}

bool Playback::startPlayback(std_srvs::Trigger::Request  &req,
                             std_srvs::Trigger::Response &res) {
  if (!started_ || finished_) {

    // Initialize playback
    ROS_INFO("[Playback] Initializing playback ...");
    initialize();
    initializePlayback();

    // Create threads
    load_thread_.reset(new boost::thread(boost::bind(&Playback::loadMsg, this)));
    pickup_thread_.reset(new boost::thread(boost::bind(&Playback::pickupMsg, this)));
    ROS_INFO("[Playback] Started playback");

    started_ = true;
    res.message = "Playback ON";
    res.success = true;
  } else {
    ROS_WARN("[Playback] Playback already started, doing nothing!");
    res.message = "Playback already started, doing nothing!";
    res.success = false;
  }
  return true;
}

void Playback::loadParams() {
  // Getting session wide parameters
  nh_->param<std::string>("/subject_id", subject_id_,"x");
  nh_->param<std::string>("/session_id", session_id_,"session_1");
  nh_->param<std::string>("/activity_id", activity_id_,"act");
  nh_->param<std::string>("/condition_id", condition_id_,"in");
  nh_->param<int>("/trial_id", trial_id_, 1);

  // Getting node specific parameters
  pnh_->param<std::string>("data_dir", data_dir_, "/mnt/md0/gear_data");
  pnh_->param<std::string>("image_prefix", image_prefix_, "im");
  pnh_->param<std::string>("image_def_color", image_def_color_, "hd");
  pnh_->param<std::string>("image_def_depth", image_def_depth_, "sd");
  pnh_->param<int>("clock_frequency", clock_frequency_, 100);
  pnh_->param<bool>("publish_tf", publish_tf_, true);
  //pnh_->param<int>("queue_size", queue_size_, 1000);

  // Getting rate
  double rate;
  pnh_->param<double>("rate", rate, 1.0);
  rate_ = rate;

  // Print
  ROS_INFO_STREAM("[Playback] Parameters loaded");
  ROS_INFO_STREAM("[Playback] Subject: "<<subject_id_);
  ROS_INFO_STREAM("[Playback] Session: "<<session_id_);
  ROS_INFO_STREAM("[Playback] Activity: "<<activity_id_);
  ROS_INFO_STREAM("[Playback] Condition: "<<condition_id_);
  ROS_INFO_STREAM("[Playback] Trial: "<<trial_id_);
  ROS_INFO_STREAM("[Playback] Data directory: "<<data_dir_);
  ROS_INFO_STREAM("[Playback] Image prefix: "<<image_prefix_);
  ROS_INFO_STREAM("[Playback] Image definition color: "<<image_def_color_);
  ROS_INFO_STREAM("[Playback] Image definition depth: "<<image_def_depth_);
  ROS_INFO_STREAM("[Playback] Clock frequency: "<<clock_frequency_);
  ROS_INFO_STREAM("[Playback] Rate: "<<rate_);
  ROS_INFO_STREAM("[Playback] Publish TF: "<<publish_tf_);
}

void Playback::loadImageInfo() {

  // Get all directories where images are present
  image_root_dir_ = trial_root_dir_/fs::path("images");

  // Get the directory where calibration information found
  calibration_root_dir_ = trial_root_dir_/fs::path("calibration");

  // Check if calibration data is present
  if ( !fs::exists(calibration_root_dir_) || fs::is_empty(calibration_root_dir_))  {
    ROS_ERROR_STREAM("[Playback] Calibration data is missing: "<<calibration_root_dir_.string());
    return;
  }

  // Iterate over all images over directories in image root directory
  if ( fs::exists(image_root_dir_) && fs::is_directory(image_root_dir_) && !fs::is_empty(image_root_dir_))  {
    for( fs::directory_iterator dir_iter(image_root_dir_) ; dir_iter != fs::directory_iterator{} ; ++dir_iter) {

      // Check if the directory is not empty before proceeding
      if (fs::is_directory(dir_iter->status()) && !fs::is_empty(dir_iter->path())) {

        std::string dir_name = dir_iter->path().filename().string();
        createPublisher(dir_name);
        for( fs::directory_iterator im_iter(*dir_iter) ; im_iter != fs::directory_iterator() ; ++im_iter) {
          if (fs::is_regular_file(im_iter->status()) ) {

            // Get image extension
            std::string image_extn = im_iter->path().extension().string();
            if (image_extn_.find(dir_name) == image_extn_.end()) {
              image_extn_.insert(std::map<std::string, std::string>::value_type(dir_name, image_extn));
            }

            // Get image name
            std::string image_name = im_iter->path().filename().string();
            image_info_.insert(std::multimap<ros::Time,std::string>::value_type(parseTimeStamp(image_name),
                                                                                dir_name));
          }
        }
      }
    }
  } else {
    ROS_ERROR("[Playback] No image data found");
  }
}

void Playback::loadClockInfo() {

  ros::Time t_start, t_stop;

  // Check if audio data is available
  if( audio_info_.begin() != audio_info_.end()) {

    // Set start time as minimum of all messages
    t_start = std::min(image_info_.begin()->first, audio_info_.begin()->first);

    // Set stop time as maximum of all messages
    t_stop = std::max(image_info_.rbegin()->first, audio_info_.rbegin()->first);
  } else {
    t_start = image_info_.begin()->first;
    t_stop = image_info_.rbegin()->first;
  }

  // Compute time step
  ros::Duration t_interval(1.0/clock_frequency_);

  // Fill clock vector
  for(ros::Time t=t_start; t<=t_stop; t+=t_interval) {
    clock_info_.push_back(t);
  }
}

void Playback::loadAudioInfo() {

  // Get audio bag path
  fs::path audio_bag_path = trial_root_dir_/fs::path("bags/audio.bag");

  // Check if audio information exists
  if (!fs::exists(audio_bag_path)) {
    ROS_WARN_STREAM("[Playback] Audio file missing: "<<audio_bag_path.string());
    ROS_WARN("[Playback] Not playing audio");
    return;
  }

  // Open audio bag
  rosbag::Bag audio_bag;
  audio_bag.open(audio_bag_path.string(), rosbag::bagmode::Read);

  // Get bag topics
  std::vector<std::string> topics;
  topics.push_back(std::string("audio_info"));

  // Iterate through messages in bag
  rosbag::View view(audio_bag, rosbag::TopicQuery(topics));

  BOOST_FOREACH(rosbag::MessageInstance const m, view) {
      audio_common_msgs::StampedAudioDataConstPtr msg = m.instantiate<audio_common_msgs::StampedAudioData>();
      audio_info_.insert(std::multimap<ros::Time, audio_common_msgs::StampedAudioDataConstPtr>::value_type
          (msg->header.stamp, msg));
  }

  audio_bag.close();
}

ros::Time Playback::parseTimeStamp(const std::string &image_name) {
  std::vector<std::string> strs;
  boost::split(strs, image_name, boost::is_any_of("_."));
  ros::Time t(boost::lexical_cast<uint32_t>(strs[1]), boost::lexical_cast<uint32_t>(strs[2]));
  return t;
}

void Playback::publishImageOld(const std::multimap <ros::Time, std::string>::iterator &image_it) {
  // Create image message
  sensor_msgs::ImageConstPtr ros_image_ptr = createImageMsg(image_it);

  // Publish
  image_pub_.at(image_it->second)->publish(*ros_image_ptr, *camera_info_.at(image_it->second));
}

void Playback::createPublisher(const std::string &dir_name) {
  // Check if the specified publisher exists
  if(image_pub_.find(dir_name) == image_pub_.end()) {

    // Parse directory name to get publisher details
    std::string camera_id = getCameraID(dir_name);
    std::string camera_type = getCameraType(dir_name);

    std::string image_def;
    if (camera_type.compare("depth") == 0) {
      image_def = image_def_depth_;
    } else {
      image_def = image_def_color_;
    }

    // Create message name
    std::string msg_name = std::string("/")+camera_id+std::string("/")+image_def
                           +std::string("/")+std::string("image_")+camera_type;
    boost::shared_ptr<image_transport::CameraPublisher> pub(new image_transport::CameraPublisher
        (it_->advertiseCamera(msg_name, 1)));
    image_pub_[dir_name] = pub;

    // Load camera information
    if (dir_name[0] == 'p') {

      // Loading pointgray camera info
      camera_info_[dir_name] = loadPointGreyCameraInfo(dir_name);
    } else if (dir_name[0] == 'k') {

      // Loading kinect camera info
      camera_info_[dir_name] = loadKinectCameraInfo(dir_name);
    } else {
      ROS_ERROR("[Playback] Unknown sensor type. Cannot find calibration information");
    }

    // Publish tf
    if (publish_tf_) {
      if (dir_name[0] == 'p') {

        // Publish the link tf that connects to world tf tree
        publishLinkStaticTF(camera_id);
      } else if (dir_name[0] == 'k') {

        // Publish the link tf that connects to world tf tree
        publishLinkStaticTF(camera_id);

        // Publish kinect internal frames TF
        cv::Mat rotation, translation;
        fs::path pose_url = fs::path(calibration_root_dir_)/
                            fs::path(camera_id+POSES_FILE_SUFFIX+std::string(".yaml"));

        if(!loadKinectCalibrationPoseFile(pose_url.string(), rotation, translation)) {
          ROS_ERROR_STREAM("[Playback] Failed to open pose file: " << pose_url.string());
        }
        publishKinectStaticTF(camera_id, rotation, translation);

      } else {
        ROS_WARN("[Playback] Unknown sensor type. Cannot find tf information");
      }

      // Publish transforms
      tf_pub_.sendTransform(transforms_);
    }
  }
}

std::string Playback::getCameraID(const std::string &camera_name) const {
  std::vector<std::string> strs;
  boost::split(strs, camera_name, boost::is_any_of("_"));
  return strs[0];
}

std::string Playback::getCameraType(const std::string &camera_name) const {
  std::vector<std::string> strs;
  boost::split(strs, camera_name, boost::is_any_of("_"));
  return strs[1];
}

boost::shared_ptr<sensor_msgs::CameraInfo>
Playback::loadPointGreyCameraInfo(const std::string &camera_name) {
  fs::path camera_info_url = fs::path(calibration_root_dir_)/fs::path(camera_name+std::string(".yaml"));
  boost::shared_ptr<sensor_msgs::CameraInfo> c_info;

  // Load camera info if it exists
  if (fs::exists(camera_info_url)) {
    camera_info_manager::CameraInfoManager c_info_manager(*nh_, "narrow_stereo", std::string("file://")+camera_info_url.string());
    c_info.reset(new sensor_msgs::CameraInfo(c_info_manager.getCameraInfo()));
  } else {
    ROS_ERROR("[Playback] Unable to find calibration file: %s", camera_info_url.string().c_str());
  }
  return c_info;
}

boost::shared_ptr<sensor_msgs::CameraInfo>
Playback::loadKinectCameraInfo(const std::string &camera_name) {

  // Initialize camera matrices
  cv::Mat camera_matrix;
  cv::Mat distortion;
  cv::Mat rotation = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat projection = cv::Mat::zeros(3, 4, CV_64F);

  // Initialize calibration filename
  fs::path camera_info_url = fs::path(calibration_root_dir_)/fs::path(camera_name+std::string(".yaml"));

  // Load information from input calibration files
  if(!loadKinectCalibrationFile(camera_info_url.string(), camera_matrix, distortion)) {
    ROS_ERROR_STREAM("[Playback] Failed to open calibration file: " << camera_info_url.string());
  }

  // Fill the projection matrix
  camera_matrix.copyTo(projection(cv::Rect(0, 0, 3, 3)));

  // Create the camera info object
  boost::shared_ptr<sensor_msgs::CameraInfo> camera_info(new sensor_msgs::CameraInfo());

  // Assign the right frame id
  camera_info->header.frame_id = getFrameID(camera_name);

  if (camera_name.find("color") != std::string::npos) {
    createKinectCameraInfo(KINECT_HD_SIZE_COLOR, camera_matrix, distortion, rotation, projection, camera_info);
  } else if (camera_name.find("depth") != std::string::npos) {
    createKinectCameraInfo(KINECT_SD_SIZE_DEPTH, camera_matrix, distortion, rotation, projection, camera_info);
  } else {
    ROS_ERROR("[Playback] Unknown kinect data type %s", camera_name.c_str());
  }

  return camera_info;
}

bool Playback::loadKinectCalibrationFile(const std::string &filename,
                                         cv::Mat &camera_matrix, cv::Mat &distortion) const {
  cv::FileStorage fs;
  if(fs.open(filename, cv::FileStorage::READ))
  {
    fs[KINECT_CALIB_CAMERA_MATRIX] >> camera_matrix;
    fs[KINECT_CALIB_DISTORTION] >> distortion;
    fs.release();
  }
  else  {
    ROS_ERROR_STREAM("[Playback] Failed to open calibration file: " << filename);
    return false;
  }
  return true;
}

void Playback::createKinectCameraInfo(const cv::Size &size, const cv::Mat &camera_matrix,
                                      const cv::Mat &distortion, const cv::Mat &rotation,
                                      const cv::Mat &projection,
                                      boost::shared_ptr<sensor_msgs::CameraInfo> camera_info) const {
  camera_info->height = size.height;
  camera_info->width = size.width;

  const double *itC = camera_matrix.ptr<double>(0, 0);
  for(size_t i = 0; i < 9; ++i, ++itC)  {
    camera_info->K[i] = *itC;
  }

  const double *itR = rotation.ptr<double>(0, 0);
  for(size_t i = 0; i < 9; ++i, ++itR)  {
    camera_info->R[i] = *itR;
  }

  const double *itP = projection.ptr<double>(0, 0);
  for(size_t i = 0; i < 12; ++i, ++itP)  {
    camera_info->P[i] = *itP;
  }

  camera_info->distortion_model = "plumb_bob";
  camera_info->D.resize(distortion.cols);
  const double *itD = distortion.ptr<double>(0, 0);
  for(size_t i = 0; i < (size_t)distortion.cols; ++i, ++itD)  {
    camera_info->D[i] = *itD;
  }
}

std::string Playback::getFrameID(const std::string &camera_name) const {
  std::string frame_id;
  std::string camera_id = getCameraID(camera_name);
  std::string camera_type = getCameraType(camera_name);

  if (camera_name[0] == 'p') {
    frame_id = camera_id;
  } else if (camera_name[0] == 'k') {
    if (camera_type.compare("color") == 0) {
      frame_id = camera_id+KINECT_TF_RGB_OPT_FRAME;
    } else if (camera_type.compare("depth") == 0) {
      frame_id = camera_id+KINECT_TF_IR_OPT_FRAME;
    } else {
      ROS_ERROR("[Playback] Unknown kinect data type: %s", camera_name.c_str());
    }
  } else {
    ROS_ERROR("[Playback] Unknown sensor type. Cannot find calibration information");
  }
  return frame_id;
}

void Playback::publishKinectStaticTF(const std::string &kinect, const cv::Mat &rotation, const cv::Mat &translation) {
  // Get the rotation matrix
  tf2::Matrix3x3 rot(rotation.at<double>(0, 0), rotation.at<double>(0, 1), rotation.at<double>(0, 2),
                     rotation.at<double>(1, 0), rotation.at<double>(1, 1), rotation.at<double>(1, 2),
                     rotation.at<double>(2, 0), rotation.at<double>(2, 1), rotation.at<double>(2, 2));

  // Get the quarternion from rotation matrix
  tf2::Quaternion quat;
  rot.getRotation(quat);

  // Get the translation
  tf2::Vector3 trans(translation.at<double>(0), translation.at<double>(1), translation.at<double>(2));

  // Get the color transform
  geometry_msgs::TransformStamped color_transform = createTransformMsg(kinect + LINK_TF_FRAME,
                                                                       kinect + KINECT_TF_RGB_OPT_FRAME);
  transforms_.push_back(color_transform);

  // Get the depth transform
  geometry_msgs::TransformStamped depth_transform = createTransformMsg(kinect + KINECT_TF_RGB_OPT_FRAME,
                                                                       kinect + KINECT_TF_IR_OPT_FRAME,
                                                                       trans, quat);
  transforms_.push_back(depth_transform);
}

void Playback::publishLinkStaticTF(const std::string &camera_id) {
  geometry_msgs::TransformStamped tf_transform = createTransformMsg(camera_id, camera_id+LINK_TF_FRAME);
  transforms_.push_back(tf_transform);
}

geometry_msgs::TransformStamped Playback::createTransformMsg(const std::string &parent_frame,
                                                             const std::string &child_frame,
                                                             const tf2::Vector3 &trans,
                                                             const tf2::Quaternion &quat,
                                                             const ros::Time &stamp) const{

  // Get the color transform
  geometry_msgs::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = parent_frame;
  transform.child_frame_id = child_frame;
  transform.transform.translation.x = trans.x();
  transform.transform.translation.y = trans.y();
  transform.transform.translation.z = trans.z();
  transform.transform.rotation.x = quat.x();
  transform.transform.rotation.y = quat.y();
  transform.transform.rotation.z = quat.z();
  transform.transform.rotation.w = quat.w();
  return transform;
}

bool Playback::loadKinectCalibrationPoseFile(const std::string &filename,
                                             cv::Mat &rotation, cv::Mat &translation) const{
  cv::FileStorage fs;
  if(fs.open(filename, cv::FileStorage::READ))
  {
    fs[KINECT_CALIB_ROTATION] >> rotation;
    fs[KINECT_CALIB_TRANSLATION] >> translation;
    fs.release();
  }
  else
  {
    ROS_ERROR_STREAM("[Playback can't open calibration pose file: " << filename);
    return false;
  }
  return true;
}

void Playback::loadMsg(){
  std::multimap<ros::Time, std::string>::iterator image_it=image_info_.begin();
  std::multimap<ros::Time, audio_common_msgs::StampedAudioDataConstPtr>::iterator audio_it=audio_info_.begin();
  std::vector<ros::Time>::iterator clock_it=clock_info_.begin();

  bool all_done = image_it == image_info_.end() && audio_it == audio_info_.end() && clock_it == clock_info_.end();

  try {
    while (!all_done) {
      boost::this_thread::interruption_point();

      ros::Time image_time = (image_it != image_info_.end()) ? image_it->first: ros::TIME_MAX;
      ros::Time audio_time = (audio_it != audio_info_.end()) ? audio_it->first: ros::TIME_MAX;
      ros::Time clock_time = (clock_it != clock_info_.end()) ? *clock_it: ros::TIME_MAX;

      // Get the current time vector
      std::vector<ros::Time> curr_time = {image_time, audio_time, clock_time};

      // Get the minimum element
      std::vector<ros::Time>::iterator min_time = std::min_element(curr_time.begin(), curr_time.end());
      std::vector<ros::Time>::iterator::difference_type index = std::distance(curr_time.begin(), min_time);

      // Create message wrapper based on message type
      MessageWrapper msg_wrapper;
      switch(index){
        case 0: {
          ROS_DEBUG("[Playback] Loading image message into queue ...");

          // Check if someone needs image data before loading
          if (image_pub_.at(image_it->second)->getNumSubscribers() > 0) {
            sensor_msgs::ImageConstPtr image_msg_ptr(createImageMsg(image_it));
            msg_wrapper.setImageData(image_msg_ptr, image_it->second);
          }
          image_it++;
          break;
        }
        case 1: {
          ROS_DEBUG("[Playback] Loading audio message queue ...");
          msg_wrapper.setAudioData(audio_it->second);
          audio_it++;
          break;
        }
        case 2: {
          ROS_DEBUG("[Playback] Loading clock message into queue ...");
          rosgraph_msgs::Clock clk;
          clk.clock = *clock_it;
          rosgraph_msgs::ClockConstPtr clk_ptr(new rosgraph_msgs::Clock(clk));
          msg_wrapper.setClockData(clk_ptr);
          clock_it++;
          break;
        }
        default: {
          ROS_ERROR("[Playback] Unknown message type, not loading message");
          loaded_msgs_++;
          all_done = image_it == image_info_.end() && audio_it == audio_info_.end() && clock_it == clock_info_.end();
          continue;
        }
      }

      // Push into message queue
      if(msg_wrapper.hasData()) {
        ROS_DEBUG("[Playback] Pushing message into queue (has data: %d)", msg_wrapper.hasData());
        while (!msg_queue_->push(msg_wrapper)) {
          boost::this_thread::interruption_point();
        }
        loaded_msgs_++;
      }

      all_done = image_it == image_info_.end() && audio_it == audio_info_.end() && clock_it == clock_info_.end();
    }
  } catch (boost::thread_interrupted&) {
    ROS_WARN("[Playback] Interrupted loading messages");
  }
  finished_ = true;
}

sensor_msgs::ImageConstPtr
Playback::createImageMsg(const std::multimap <ros::Time, std::string>::iterator &image_it) {
  // Pad nano seconds with zeros
  boost::format nsec("%09d");
  nsec % image_it->first.nsec;

  // Get image file path
  std::string image_name = image_prefix_+std::string("_")+std::to_string(image_it->first.sec)
                           +std::string("_")+nsec.str()
                           +image_extn_.at(image_it->second);
  fs::path image_path = image_root_dir_/fs::path(image_it->second)/fs::path(image_name);

  // Load image
  cv_bridge::CvImage cv_image;
  if (fs::exists(image_path)) {
    cv_image.image = cv::imread(image_path.string(),CV_LOAD_IMAGE_COLOR);
  } else {
    ROS_ERROR("[Playback] Unable to find image: %s", image_path.string().c_str());
  }
  cv_image.encoding = "bgr8";
  sensor_msgs::Image ros_image;
  cv_image.toImageMsg(ros_image);

  // Set header information
  ros_image.header.stamp = image_it->first;
  ros_image.header.frame_id = getFrameID(image_it->second);

  sensor_msgs::ImageConstPtr ros_image_ptr(new sensor_msgs::Image(ros_image));
  return ros_image_ptr;
}

void Playback::pickupMsg(){
  MessageWrapper msg_wrapper;
  ros::Time t_prev(clock_info_.front());
  ros::Time t_next(clock_info_.front());

  try {
    while (!finished_) {
      boost::this_thread::interruption_point();
      while (!msg_queue_->pop(msg_wrapper)) {
        boost::this_thread::interruption_point();
      }

      // Get the time of current message
      t_next = msg_wrapper.getMessageTime();

      // Wait for sometime
      ros::Duration t_diff = t_next-t_prev;
      ros::Duration t_rate_diff(t_diff.toSec()/rate_);
      ROS_DEBUG("[Playback] Times computed (t_prev, t_next, rate, t_diff, t_rate_diff) = (%f, %f, %f, %f, %f)",
                t_prev.toSec(), t_next.toSec(), rate_.load(), t_diff.toSec(), t_rate_diff.toSec());
      ros::Time::sleepUntil(ros::Time::now()+t_rate_diff);
      t_prev = t_next;

      published_msgs_++;

      // Publish message
      publishMsg(msg_wrapper);
    }
    while (!msg_queue_->pop(msg_wrapper)) {
      boost::this_thread::interruption_point();
    }
    publishMsg(msg_wrapper);
  } catch (boost::thread_interrupted&) {
    ROS_WARN("[Playback] Interrupted pickup messages");
  }
  ROS_INFO_STREAM("Loaded messages: " << loaded_msgs_);
  ROS_INFO_STREAM("Published messages: " << published_msgs_);
}

void Playback::publishMsg(const MessageWrapper &msg_wrapper) const{
  ROS_ERROR_COND(!msg_wrapper.hasData(), "[Playback] No data in message wrapper");

  if (msg_wrapper.hasAudioData()) {

    // Publish audio data
    ROS_DEBUG("[Playback] Publishing audio data ...");
    audio_pub_.publish(msg_wrapper.getAudioData());
  } else if (msg_wrapper.hasImageData()) {
    std::string pub_name = msg_wrapper.getPublisherName();

    // Publish image data
    ROS_DEBUG("[Playback] Publishing image data ...");
    sensor_msgs::ImageConstPtr img_msg = msg_wrapper.getImageData();
    boost::shared_ptr<sensor_msgs::CameraInfo> cam_info(camera_info_.at(pub_name));
    cam_info->header.stamp = img_msg->header.stamp;
    image_pub_.at(pub_name)->publish(*img_msg, *cam_info);
  } else if (msg_wrapper.hasClockData()) {

    // Publish clock data
    ROS_DEBUG("[Playback] Publishing clock data ...");
    clock_pub_.publish(msg_wrapper.getClockData());
  }
}

void Playback::reconfigureCallback(gear_data_handler::PlaybackConfig &config, uint32_t level) {
  ROS_INFO("Rate reconfigure requested: %f", config.rate);
  pnh_->setParam("rate", config.rate);
  rate_ = config.rate;
}

MessageWrapper::MessageWrapper() : has_data_(false), has_audio_data_(false),
    has_image_data_(false), has_clock_data_(false), pub_name_(""){};

bool MessageWrapper::setAudioData(const audio_common_msgs::StampedAudioDataConstPtr &audio_msg) {
  if (!has_data_) {
    audio_msg_ = audio_msg;
    has_audio_data_ = true;
    updateStatus();
    return false;
  } else {
    ROS_ERROR("[Playback] Cannot add audio data to a container that has data");
    return true;
  }
}

bool MessageWrapper::setClockData(const rosgraph_msgs::ClockConstPtr &clock_msg) {
  if (!has_data_) {
    clock_msg_ = clock_msg;
    has_clock_data_ = true;
    updateStatus();
    return false;
  } else {
    ROS_ERROR("[Playback] Cannot add audio data to a container that has data");
    return true;
  }
}

bool MessageWrapper::setImageData(const sensor_msgs::ImageConstPtr &image_msg, const std::string &pub_name) {
  if (!has_data_) {
    image_msg_ = image_msg;
    pub_name_ = pub_name;
    has_image_data_ = true;
    updateStatus();
    return false;
  } else {
    ROS_ERROR("[Playback] Cannot add image data to a container that has data");
    return true;
  }
}

void MessageWrapper::updateStatus() {
  has_data_ = (has_image_data_ || has_audio_data_ || has_clock_data_);
}

ros::Time MessageWrapper::getMessageTime() {
  if (hasAudioData()) {
    return audio_msg_->header.stamp;
  } else if (hasImageData()) {
    return image_msg_->header.stamp;
  } else if (hasClockData()) {
    return clock_msg_->clock;
  } else {
    ROS_ERROR("[Playback] No recognized data to get time stamp from");
    return ros::Time(0);
  }
}

audio_common_msgs::StampedAudioDataConstPtr MessageWrapper::getAudioData() const{
  return audio_msg_;
}

sensor_msgs::ImageConstPtr MessageWrapper::getImageData() const{
  return image_msg_;
}

rosgraph_msgs::ClockConstPtr MessageWrapper::getClockData() const{
  return clock_msg_;
}

};
