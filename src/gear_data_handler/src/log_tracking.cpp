#include <gear_data_handler/log_tracking.h>

// Boost Dependencies
#include <boost/format.hpp>

namespace gear_data_handler {

TrackingLogger::TrackingLogger(): BagLogger(){};

void TrackingLogger::onInit(){

  // Initialize
  initialize(getNodeHandle(), getPrivateNodeHandle());

  // Loading tag type
  getPrivateNodeHandle().param<std::string>("tag_type", tag_type_,"ar");
  ROS_INFO_STREAM("[Logger] Parameter tag type: "<<tag_type_.c_str());

  if (tag_type_.compare("april")==0) {
    sub_ = getNodeHandle().subscribe("/log_tracking", 5, &TrackingLogger::aprilTrackingCallback, this);
  } else {
    sub_ = getNodeHandle().subscribe("/log_tracking", 5, &TrackingLogger::arTrackingCallback, this);
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

};

