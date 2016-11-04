#include <gear_image_handler/log_pointcloud.h>

// Ros Dependencies
#include <std_msgs/Int64.h>
#include <cv_bridge/cv_bridge.h>

// Boost Dependencies
#include <boost/format.hpp>

//PCL Dependencies
#include <pcl/io/pcd_io.h>

namespace gear_image_handler {

PointcloudLogger::PointcloudLogger(): pointcloud_count_(0), enable_(false){};

void PointcloudLogger::onInit(){
  boost::mutex::scoped_lock(session_param_lock_);

  // Getting session wide parameters
  getNodeHandle().param<std::string>("session_id", session_id_,"test");
  getNodeHandle().param<std::string>("activity_id", activity_id_,"act");
  getNodeHandle().param<std::string>("trial_id", trial_id_,"1");

  // Getting node specific parameters
  getPrivateNodeHandle().getParam("sensor_id", sensor_id_);
  getPrivateNodeHandle().getParam("data_dir", data_dir_);
  getPrivateNodeHandle().getParam("base_name", base_name_);

  pointcloud_sub_ = getNodeHandle().subscribe("/log_pointcloud", 5, &PointcloudLogger::pointcloudCallback, this);
  pointcloud_count_pub_ = getNodeHandle().advertise<std_msgs::Int64>("/pointcloud_count", 15);

  NODELET_INFO_STREAM("[PointcloudLogger] Parameter session_id: "<<session_id_.c_str());
  NODELET_INFO_STREAM("[PointcloudLogger] Parameter sensor_id: "<<sensor_id_.c_str());
  NODELET_INFO_STREAM("[PointcloudLogger] Parameter data_dir: "<<data_dir_.c_str());

  // Create service to toggle image logging
  toggle_logger_ = getNodeHandle().advertiseService(base_name_+std::string("_points")+
                                                    std::string("_enable"),
                                                    &PointcloudLogger::toggleLogger, this);

  // Create service to toggle image logging
  session_info_ = getNodeHandle().advertiseService(base_name_+std::string("_points")+
                                                   std::string("_session_info"),
                                                   &PointcloudLogger::setSessionInfo, this);
}

void PointcloudLogger::pointcloudCallback(const pcl::PCLPointCloud2::ConstPtr& msg) {
  boost::mutex::scoped_lock(session_param_lock_);
  boost::mutex::scoped_lock(enable_lock_);

  if (enable_) {
    // Construct the filename
    ros::Time stamp;
    stamp.fromNSec(msg->header.stamp);
    boost::format nsec("%09d");
    nsec % stamp.nsec;
    std::string pointcloud_name = std::string("pc_")+std::to_string(stamp.sec)+
                                  std::string("_")+nsec.str()+std::string(".pcd");

    boost::filesystem::path pointcloud_path = pointcloud_dir_/boost::filesystem::path(pointcloud_name);

    // Saving to disk
    pcl::PCDWriter writer;
    writer.writeBinaryCompressed(pointcloud_path.string(), *msg);
    pointcloud_count_++;

    // Publish running image counts
    std_msgs::Int64 count_msg;
    count_msg.data = pointcloud_count_;
    pointcloud_count_pub_.publish(count_msg);

  }
}

bool PointcloudLogger::toggleLogger(std_srvs::SetBool::Request  &req,
                                    std_srvs::SetBool::Response &res){

  boost::mutex::scoped_lock(session_param_lock_);
  boost::mutex::scoped_lock(enable_lock_);

  // Set the enable parameter based on service call value
  if (req.data) {
    initializeSessionDirectories();
    NODELET_INFO("[PointcloudLogger] [%sPoints] Turning on pointcloud logging", base_name_.c_str());

    enable_ = true;
    res.message = base_name_+std::string("_points")+std::string(": PointcloudLogger ON");
  } else {
    NODELET_INFO("[PointcloudLogger] [%sPoints] Turning off pointcloud logging", base_name_.c_str());
    enable_ = false;
    res.message = base_name_+std::string("_points")+std::string(": PointcloudLogger OFF");
  }

  res.success = true;
  return true;
}

bool PointcloudLogger::setSessionInfo(std_srvs::Trigger::Request  &req,
                                      std_srvs::Trigger::Response &res){

  boost::mutex::scoped_lock(session_param_lock_);

  // Read session parameters from parameter server
  getNodeHandle().getParam("session_id", session_id_);
  getNodeHandle().getParam("action_id", activity_id_);
  getNodeHandle().getParam("trial_id", trial_id_);

  res.message = base_name_+std::string("_points")+std::string(": Session parameters SET");
  res.success = true;
  return true;
}

void PointcloudLogger::initializeSessionDirectories() {
  boost::mutex::scoped_lock(session_param_lock_);

  // Setting paths
  pointcloud_dir_ =  boost::filesystem::path(data_dir_)/
                     boost::filesystem::path(session_id_)/
                     boost::filesystem::path(activity_id_+std::string("_")+trial_id_)/
                     boost::filesystem::path("images")/
                     boost::filesystem::path(sensor_id_+std::string("_points"));
  if (!boost::filesystem::exists(pointcloud_dir_)) {
    boost::filesystem::create_directories(pointcloud_dir_);
  }
  NODELET_INFO("[PointcloudLogger] [%sPoints] Logging pointcloud to directory: %s",
                base_name_.c_str(), pointcloud_dir_.c_str());
}
};

