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

    for(const auto &m: msg.detections) {
      // Log to text file
      logText(m.pose);
    }

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

    for(const auto &m: msg.markers) {
      // Log to text file
      geometry_msgs::PoseStamped pose_msg = m.pose;
      pose_msg.header.frame_id = m.header.frame_id;
      logText(pose_msg);
    }

    // Publish running image counts
    publishCount();
  }
}

void TrackingLogger::logText(const geometry_msgs::PoseStamped& msg) {

  if (!msg.header.frame_id.compare("")==0) {
    // Get the tf transformation
    geometry_msgs::PoseStamped pose_out;
    tf_listener_.transformPose("/world", msg, pose_out);

    // Write to text file
    text_fp_<<msg.header.stamp.toSec()
            <<"\t"<<msg.pose.position.x<<"\t"<<msg.pose.position.y<<"\t"<<msg.pose.position.z
            <<"\t"<<msg.pose.orientation.x<<"\t"<<msg.pose.orientation.y<<"\t"<<msg.pose.orientation.z
            <<"\t"<<msg.pose.orientation.w<<std::endl;
  }
}

bool TrackingLogger::toggleLogger(std_srvs::SetBool::Request  &req,
                                  std_srvs::SetBool::Response &res){

  bool out = false;

  // Open rosbag
  if (req.data) {
    out = BagLogger::toggleLogger(req, res);
    boost::filesystem::path text_path = image_dir_/boost::filesystem::path(image_type_+".txt");
    text_fp_.open(text_path.string());
  } else {
    text_fp_.close();
  }

  return out;
}

};

