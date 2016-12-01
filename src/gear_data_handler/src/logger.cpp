#include "../include/gear_data_handler/logger.h"

#include <std_msgs/Int64.h>

// Boost Dependencies
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

namespace gear_data_handler {

Logger::Logger(): count_(0), enable_(false){};

void Logger::initialize(ros::NodeHandle nh, ros::NodeHandle pnh){

  // Get node handles
  nh_.reset(new ros::NodeHandle(nh));
  pnh_.reset(new ros::NodeHandle(pnh));

  boost::mutex::scoped_lock(session_param_lock_);

  // Getting session wide parameters
  nh_->param<std::string>("/subject_id", subject_id_,"x");
  nh_->param<std::string>("/session_id", session_id_,"session_1");
  nh_->param<std::string>("/activity_id", activity_id_,"act");
  nh_->param<std::string>("/condition_id", condition_id_,"in");
  nh_->param<int>("/trial_id", trial_id_, 1);

  // Getting node specific parameters
  pnh_->param<std::string>("sensor_id", sensor_id_, "k1");
  pnh_->param<std::string>("data_dir", data_dir_, "/mnt/md0/gear_data");
  pnh_->param<std::string>("image_extn", image_extn_, ".jpg");
  pnh_->param<std::string>("image_type", image_type_, "color");
  pnh_->param<std::string>("image_prefix", image_prefix_, "im");

  count_pub_ = nh_->advertise<std_msgs::Int64>("/image_count", 15);

  ROS_INFO_STREAM("[Logger] Parameter subject_id: "<<subject_id_.c_str());
  ROS_INFO_STREAM("[Logger] Parameter session_id: "<<session_id_.c_str());
  ROS_INFO_STREAM("[Logger] Parameter activity_id: "<<activity_id_.c_str());
  ROS_INFO_STREAM("[Logger] Parameter condition_id: "<<condition_id_.c_str());
  ROS_INFO_STREAM("[Logger] Parameter trial_id: "<<trial_id_);
  ROS_INFO_STREAM("[Logger] Parameter sensor_id: "<<sensor_id_.c_str());
  ROS_INFO_STREAM("[Logger] Parameter data_dir: "<<data_dir_.c_str());
  ROS_INFO_STREAM("[Logger] Parameter image_extn: "<<image_extn_.c_str());
  ROS_INFO_STREAM("[Logger] Parameter image_type: "<<image_type_.c_str());
  ROS_INFO_STREAM("[Logger] Parameter image_prefix: "<<image_prefix_.c_str());

  // Create service to toggle image logging
  toggle_logger_ = nh_->advertiseService(sensor_id_+"_"+image_type_+"_enable",
                                         &Logger::toggleLogger, this);

  // Create service to toggle image logging
  session_info_ = nh_->advertiseService(sensor_id_+"_"+image_type_+"_session_info",
                                        &Logger::setSessionInfo, this);
}

bool Logger::toggleLogger(std_srvs::SetBool::Request  &req,
                          std_srvs::SetBool::Response &res){

  boost::mutex::scoped_lock(session_param_lock_);
  boost::mutex::scoped_lock(enable_lock_);

  // Set the enable parameter based on service call value
  if (req.data) {
    initializeSessionDirectories();
    ROS_INFO("[Logger][%s%s] Turning on image logging", sensor_id_.c_str(), image_type_.c_str());

    enable_ = true;
    res.message = sensor_id_+"_"+image_type_+": ImageLogger ON";
  } else {
    ROS_INFO("[Logger][%s%s] Turning off image logging", sensor_id_.c_str(), image_type_.c_str());
    enable_ = false;
    res.message = sensor_id_+"_"+image_type_+": ImageLogger OFF";
  }

  res.success = true;
  return true;
}

bool Logger::setSessionInfo(std_srvs::Trigger::Request  &req,
                            std_srvs::Trigger::Response &res){

  boost::mutex::scoped_lock(session_param_lock_);

  // Read session parameters from parameter server
  if (nh_->hasParam("subject_id"))  nh_->getParam("/subject_id", subject_id_);
  if (nh_->hasParam("session_id"))  nh_->getParam("/session_id", session_id_);
  if (nh_->hasParam("activity_id"))  nh_->getParam("/activity_id", activity_id_);
  if (nh_->hasParam("condition_id"))  nh_->getParam("/condition_id", condition_id_);
  if (nh_->hasParam("trial_id"))  nh_->getParam("/trial_id", trial_id_);

  ROS_INFO("[Logger][%s%s] Parameter session_id: %s",
            sensor_id_.c_str(), image_type_.c_str(), session_id_.c_str());
  ROS_INFO("[Logger][%s%s] Parameter activity_id: %s",
            sensor_id_.c_str(), image_type_.c_str(), activity_id_.c_str());
  ROS_INFO("[Logger][%s%s] Parameter trial_id: %d",
            sensor_id_.c_str(), image_type_.c_str(), trial_id_);

  res.message = sensor_id_+"_"+image_type_+": Session parameters SET";
  res.success = true;
  return true;
}

void Logger::initializeSessionDirectories() {
  boost::mutex::scoped_lock(session_param_lock_);

  // Setting paths
  image_dir_ =  boost::filesystem::path(data_dir_)/
                boost::filesystem::path(subject_id_)/
                boost::filesystem::path(session_id_)/
                boost::filesystem::path(activity_id_+"_"+condition_id_+"_"
                                        +boost::lexical_cast<std::string>(trial_id_))/
                boost::filesystem::path("images")/
                boost::filesystem::path(sensor_id_+std::string("_")+image_type_);
  if (!boost::filesystem::exists(image_dir_)) {
    boost::filesystem::create_directories(image_dir_);
  }
  ROS_INFO("[Logger][%s%s] Logging images to directory: %s",
            sensor_id_.c_str(), image_type_.c_str(), image_dir_.c_str());
}

void Logger::publishCount() {
  // Increment image count
  count_++;

  // Publish running image counts
  std_msgs::Int64 count_msg;
  count_msg.data = count_;
  count_pub_.publish(count_msg);
}

boost::filesystem::path Logger::getFilePath(std::string sec, std::string nsec) {
  std::string image_name = image_prefix_+"_"+sec+"_"+nsec+image_extn_;
  boost::filesystem::path path = image_dir_/boost::filesystem::path(image_name);
  return path;
}


};

