#include <gear_data_handler/playback.h>

// ROS dependencies
#include <rosgraph_msgs/Clock.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>

// Boost dependencies
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

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

namespace gear_data_handler {
Playback::Playback(ros::NodeHandle nh, ros::NodeHandle pnh):
    enabled_(false), initialized_(false), publish_tf_(true) {
  boost::mutex::scoped_lock(enable_lock_);

  // Get node handles
  nh_.reset(new ros::NodeHandle(nh));
  pnh_.reset(new ros::NodeHandle(pnh));

  // Time publisher
  clock_pub_ = nh_->advertise<rosgraph_msgs::Clock>("clock", 1);

  // Image transport
  it_.reset(new image_transport::ImageTransport(*nh_));

  // Create service to enable playback
  enable_playback_ = nh_->advertiseService("enable_playback", &Playback::startPlayback, this);
}

void Playback::initializePlayback() {
  //Load associated parameters
  loadParams();

  //Load image information
  loadImageInfo();
  ROS_INFO("[Playback] Loaded image information: %d directories, %d images", int(image_extn_.size()),int(image_info_.size()));
}

void Playback::play() {
  boost::mutex::scoped_lock(enable_lock_);
  if(enabled_) {
    std::multimap<ros::Time, std::string>::iterator it=image_info_.begin();
    ros::Time sim_t_current = it->first;
    ros::Time sim_t_next = it->first+ros::Duration(1.0/clock_frequency_);
    ros::Duration t_diff(0);

    // Publish current time
    rosgraph_msgs::Clock clk;
    clk.clock = sim_t_current;
    clock_pub_.publish(clk);

    // Iterate over all time stamps in tandem with clock frequency
    while (it != image_info_.end()) {

      ros::Time t_start=ros::Time::now();
      ROS_DEBUG_STREAM("Wall start time: "<<t_start);

      ros::Time sim_next_msg_t = it->first;
      bool msg_published = false;

      // Check if there are any messages before the next clock tick
      if (sim_next_msg_t<=sim_t_next) {
        t_diff = ros::Duration((sim_next_msg_t.toSec()-sim_t_current.toSec())/rate_);
        sim_t_current = sim_next_msg_t;

        msg_published = true;
      }

      // If no messages within the next clock tick update clock ticks
      if(!msg_published) {
        t_diff = ros::Duration((sim_t_next.toSec()-sim_t_current.toSec())/rate_);
        sim_t_current = sim_t_next;
        sim_t_next = sim_t_current+ros::Duration(1.0/clock_frequency_);
      }

      // Update only clock tick if the message has the same time stamp as clock tick
      if(msg_published && sim_t_current == sim_t_next) {
        sim_t_next = sim_t_current+ros::Duration(1.0/clock_frequency_);
      }

      // Wait for some time
      ros::Time t_before = ros::Time::now();
      ros::Time::sleepUntil(ros::Time::now()+t_diff);
      ros::Time t_after = ros::Time::now();

      // Publish current time
      if (t_diff>ros::Duration(0.0)) {
        clk.clock = sim_t_current;
        clock_pub_.publish(clk);
      }

      // Publish message
      if (msg_published) {
        publishImage(it);
        it++;
      }

      ros::Time t_end=ros::Time::now();

      ROS_DEBUG_STREAM("Before sleeping: "<<t_before);
      ROS_DEBUG_STREAM("After sleeping: "<<t_after);
      ROS_DEBUG_STREAM("Expected time difference: "<<t_diff);
      ROS_DEBUG_STREAM("Actual time difference: "<<t_after-t_before);
      ROS_DEBUG_STREAM("Msg published "<<msg_published);
      ROS_DEBUG_STREAM("Wall end time: "<<t_end);
      ROS_DEBUG_STREAM("Iteration time: "<<t_end-t_start);
    }
  }
}

bool Playback::startPlayback(std_srvs::Trigger::Request  &req,
                             std_srvs::Trigger::Response &res) {
  boost::mutex::scoped_lock(enable_lock_);
  if (!enabled_) {
    // Set the enable parameter based on service call value
    ROS_INFO("[Playback] Starting playback");
    res.message = "Playback ON";
    enabled_ = true;

    // Initialize playback
    if (!initialized_) {
      initialized_ = true;
      initializePlayback();
    }

    res.success = true;
  } else {
    res.message = "Playback already started, doing nothing!";
    res.success = false;
  }
  return res.success;
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
  pnh_->param<std::string>("image_def", image_def_, "hd");
  pnh_->param<int>("clock_frequency", clock_frequency_, 100);
  pnh_->param<double>("rate", rate_, 1.0);
  pnh_->param<bool>("publish_tf", publish_tf_, true);
}

void Playback::loadImageInfo() {

  // Get all directories where images are present
  image_root_dir_ = fs::path(data_dir_)/fs::path(subject_id_)/fs::path(session_id_)/
                    fs::path(activity_id_+std::string("_")+condition_id_+std::string("_")+std::to_string(trial_id_))/
                    fs::path("images");

  // Get the directory where calibration information found
  calibration_root_dir_ = fs::path(data_dir_)/fs::path(subject_id_)/fs::path(session_id_)/
                          fs::path(activity_id_+std::string("_")+condition_id_+std::string("_")+std::to_string(trial_id_))/
                          fs::path("calibration");

  // Check if calibration data is present
  if ( !fs::exists(calibration_root_dir_) || fs::is_empty(calibration_root_dir_))  {
    ROS_ERROR("[Playback] Calibration data is missing");
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

ros::Time Playback::parseTimeStamp(const std::string &image_name) {
  std::vector<std::string> strs;
  boost::split(strs, image_name, boost::is_any_of("_."));
  ros::Time t(boost::lexical_cast<uint32_t>(strs[1]), boost::lexical_cast<uint32_t>(strs[2]));
  return t;
}

void Playback::publishImage(const std::multimap <ros::Time, std::string>::iterator &image_it) {
  // Pad non seconds with zeros
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
  ros_image.header.frame_id = getFrameID(image_it->second);

  // Publish
  image_pub_.at(image_it->second)->publish(ros_image, *camera_info_.at(image_it->second));
}

void Playback::createPublisher(const std::string &dir_name) {
  // Check if the specified publisher exists
  if(image_pub_.find(dir_name) == image_pub_.end()) {

    // Parse directory name to get publisher details
    std::string camera_id = getCameraID(dir_name);
    std::string camera_type = getCameraType(dir_name);

    // Create message name
    std::string msg_name = std::string("/")+camera_id+std::string("/")+image_def_
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

};

int main(int argc, char **argv){
  ros::init(argc, argv, "playback");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  gear_data_handler::Playback player(nh, pnh);
  while(ros::ok()) {
    player.play();
    ros::spinOnce();
  }
}
