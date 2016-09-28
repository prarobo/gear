#include <gear_image_handler/log_images.h>

// Opencv dependencies
#include <cv.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>

// Ros Dependencies
#include <std_msgs/Int64.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/transport_hints.h>

// Boost Dependencies
#include <boost/format.hpp>

namespace enc = sensor_msgs::image_encodings;

namespace gear_image_handler {

ImageLogger::ImageLogger(): image_count_(0), enable_(false){};

void ImageLogger::onInit(){
  image_transport::TransportHints hints("compressed", ros::TransportHints());
  it_.reset(new image_transport::ImageTransport(getNodeHandle()));
  image_sub_ = it_->subscribe("/log_image", 5, &ImageLogger::imageCallback, this, hints);
  image_count_pub_ = getNodeHandle().advertise<std_msgs::Int64>("/image_count", 15);

  // Getting session wide parameters
  pthread_mutex_lock(&session_param_lock_);
  getNodeHandle().param<std::string>("session_id", session_id_,"test");
  getNodeHandle().param<std::string>("activity_id", activity_id_,"act");
  getNodeHandle().param<std::string>("trial_id", trial_id_,"1");
  pthread_mutex_unlock(&session_param_lock_);

  // Getting node specific parameters
  getPrivateNodeHandle().getParam("sensor_id", sensor_id_);
  getPrivateNodeHandle().getParam("data_dir", data_dir_);
  getPrivateNodeHandle().getParam("image_extn", image_extn_);
  getPrivateNodeHandle().getParam("image_type", image_type_);

  NODELET_INFO_STREAM("[ImageLogger] Parameter session_id: "<<session_id_.c_str());
  NODELET_INFO_STREAM("[ImageLogger] Parameter sensor_id: "<<sensor_id_.c_str());
  NODELET_INFO_STREAM("[ImageLogger] Parameter data_dir: "<<data_dir_.c_str());
  NODELET_INFO_STREAM("[ImageLogger] Parameter image_extn: "<<image_extn_.c_str());
  NODELET_INFO_STREAM("[ImageLogger] Parameter image_type: "<<image_type_.c_str());

  // Setting encoding
  if (image_type_.compare("color") == 0) {
    encoding_ = enc::BGR8;
  }

  if (image_type_.compare("depth") == 0) {
    encoding_ = enc::TYPE_16UC1;
  }

  // Create service to toggle image logging
  toggle_logger_ = getNodeHandle().advertiseService(ros::this_node::getName()+std::string("_enable"),
                                                    &ImageLogger::toggleLogger, this);

  // Create service to toggle image logging
  session_info_ = getNodeHandle().advertiseService(ros::this_node::getName()+std::string("_session_info"),
                                                   &ImageLogger::setSessionInfo, this);
}

void ImageLogger::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  if (enable_) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try  {
      cv_ptr = cv_bridge::toCvShare(msg, encoding_);
    }
    catch (cv_bridge::Exception& e)  {
      NODELET_ERROR("[ImageLogger] Could not convert from '%s' to %s.",
                    msg->encoding.c_str(), encoding_.c_str());
    }

    // Construct the filename
    ros::Time stamp = msg->header.stamp;
    boost::format nsec("%09d");
    nsec % stamp.nsec;
    std::string image_name = std::string("im_")+std::to_string(stamp.sec)+
                             std::string("_")+ nsec.str() +image_extn_;

    pthread_mutex_lock(&session_param_lock_);
    boost::filesystem::path image_path = image_dir_/boost::filesystem::path(image_name);
    pthread_mutex_unlock(&session_param_lock_);

    //  to disk
    cv::imwrite(image_path.string(), cv_ptr->image);
    image_count_++;

    // Publish running image counts
    std_msgs::Int64 count_msg;
    count_msg.data = image_count_;
    image_count_pub_.publish(count_msg);

    //Debug
    /*cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
    cv::imshow("image", cv_ptr->image);
    cv::waitKey(1);*/
  }
}

bool ImageLogger::toggleLogger(std_srvs::SetBool::Request  &req,
                               std_srvs::SetBool::Response &res){

  // Set the enable parameter based on service call value
  if (req.data) {
    initializeSessionDirectories();
    pthread_mutex_lock(&session_param_lock_);
    NODELET_INFO("[ImageLogger] [%s] Logging images to directory: %s",
                  ros::this_node::getName().c_str(), image_dir_.string().c_str());
    pthread_mutex_unlock(&session_param_lock_);

    pthread_mutex_lock(&enable_lock_);
    enable_ = true;
    pthread_mutex_unlock(&enable_lock_);
    res.message = ros::this_node::getName()+std::string(": ImageLogger ON");
  } else {
    ROS_INFO("[ImageLogger] [%s] Turning off image logging",
              ros::this_node::getName().c_str());
    pthread_mutex_lock(&enable_lock_);
    enable_ = false;
    pthread_mutex_unlock(&enable_lock_);
    res.message = ros::this_node::getName()+std::string(": ImageLogger OFF");
  }

  res.success = true;
  return true;
}

bool ImageLogger::setSessionInfo(std_srvs::Trigger::Request  &req,
                                 std_srvs::Trigger::Response &res){

  // Read session parameters from parameter server
  pthread_mutex_lock(&session_param_lock_);
  getNodeHandle().getParam("session_id", session_id_);
  getNodeHandle().getParam("action_id", activity_id_);
  getNodeHandle().getParam("trial_id", trial_id_);
  pthread_mutex_unlock(&session_param_lock_);

  res.message = ros::this_node::getName()+std::string(": Session parameters SET");
  res.success = true;
  return true;
}

void ImageLogger::initializeSessionDirectories() {
  // Setting paths
  pthread_mutex_lock(&session_param_lock_);
  image_dir_ =  boost::filesystem::path(data_dir_)/
                boost::filesystem::path(session_id_)/
                boost::filesystem::path(activity_id_+std::string("_")+trial_id_)/
                boost::filesystem::path("images")/
                boost::filesystem::path(sensor_id_+std::string("_")+image_type_);
  if (!boost::filesystem::exists(image_dir_)) {
    boost::filesystem::create_directories(image_dir_);
  }
  pthread_mutex_unlock(&session_param_lock_);
}
};

