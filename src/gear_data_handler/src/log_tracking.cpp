#include <gear_data_handler/log_tracking.h>

// Boost Dependencies
#include <boost/format.hpp>

namespace gear_data_handler {

TrackingLogger::TrackingLogger(): Logger(){};

void TrackingLogger::onInit(){

  // Initialize
  initialize(getNodeHandle(), getPrivateNodeHandle());

  // Loading tag type
  getPrivateNodeHandle().param<std::string>("tag_type", tag_type_,"ar");
  ROS_INFO_STREAM("[Logger] Parameter subject_id: "<<tag_type_.c_str());

  if (tag_type_.compare("april")==0) {
    tracking_sub_ = getNodeHandle().subscribe("/log_tracking", 5, &TrackingLogger::aprilTrackingCallback, this);
  } else {
    tracking_sub_ = getNodeHandle().subscribe("/log_tracking", 5, &TrackingLogger::arTrackingCallback, this);
  }
}

void TrackingLogger::aprilTrackingCallback(const apriltags_ros::AprilTagDetectionArray& msg) {
  boost::mutex::scoped_lock(session_param_lock_);
  boost::mutex::scoped_lock(enable_lock_);

  if (enable_) {

    // Saving to disk
    bag_.write("tracking_info", ros::Time::now(), msg);

    // Publish running image counts
    publishCount();
  }
}

void TrackingLogger::arTrackingCallback(const ar_track_alvar_msgs::AlvarMarkers& msg) {
  boost::mutex::scoped_lock(session_param_lock_);
  boost::mutex::scoped_lock(enable_lock_);

  if (enable_) {

    // Saving to disk
    bag_.write("tracking_info", ros::Time::now(), msg);

    // Publish running image counts
    publishCount();
  }
}


void TrackingLogger::initializeSessionDirectories() {
  boost::mutex::scoped_lock(session_param_lock_);

  // Setting paths
  image_dir_ =  boost::filesystem::path(data_dir_)/
                boost::filesystem::path(subject_id_)/
                boost::filesystem::path(session_id_)/
                boost::filesystem::path(activity_id_+"_"+condition_id_+"_"
                                        +boost::lexical_cast<std::string>(trial_id_))/
                boost::filesystem::path("bags");
  if (!boost::filesystem::exists(image_dir_)) {
    boost::filesystem::create_directories(image_dir_);
  }
  ROS_INFO("[Logger][%s%s] Logging images to directory: %s",
            sensor_id_.c_str(), image_type_.c_str(), image_dir_.c_str());
}

bool TrackingLogger::toggleLogger(std_srvs::SetBool::Request  &req,
                                  std_srvs::SetBool::Response &res){

  boost::mutex::scoped_lock(session_param_lock_);
  boost::mutex::scoped_lock(enable_lock_);

  // Set the enable parameter based on service call value
  if (req.data) {
    initializeSessionDirectories();
    ROS_INFO("[Logger][%s%s] Turning on image logging", sensor_id_.c_str(), image_type_.c_str());

    // Open rosbag
    boost::filesystem::path bag_path = image_dir_/boost::filesystem::path(image_type_+image_extn_);
    bag_.open(bag_path.string(), rosbag::bagmode::Write);

    enable_ = true;
    res.message = sensor_id_+"_"+image_type_+": ImageLogger ON";
  } else {
    ROS_INFO("[Logger][%s%s] Turning off image logging", sensor_id_.c_str(), image_type_.c_str());

    // Close rosbag
    bag_.close();
    enable_ = false;
    res.message = sensor_id_+"_"+image_type_+": ImageLogger OFF";
  }

  res.success = true;
  return true;
}

};

