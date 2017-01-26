#include <gear_data_handler/log_bags.h>

// Boost Dependencies
#include <boost/format.hpp>

namespace gear_data_handler {

BagLogger::BagLogger(): Logger(){};

void BagLogger::initializeSessionDirectories() {
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
  ROS_INFO("[Logger][%s%s] Logging data to directory: %s",
            sensor_id_.c_str(), image_type_.c_str(), image_dir_.c_str());
}

bool BagLogger::toggleLogger(std_srvs::SetBool::Request  &req,
                             std_srvs::SetBool::Response &res){

  boost::mutex::scoped_lock(session_param_lock_);
  boost::mutex::scoped_lock(enable_lock_);

  // Set the enable parameter based on service call value
  if (req.data) {
    initializeSessionDirectories();
    ROS_INFO("[Logger][%s%s] Turning on data logging", sensor_id_.c_str(), image_type_.c_str());

    // Open rosbag
    boost::filesystem::path bag_path = image_dir_/boost::filesystem::path(image_type_+image_extn_);
    bag_.open(bag_path.string(), rosbag::bagmode::Write);

    enable_ = true;
    res.message = sensor_id_+"_"+image_type_+": Logger ON";
  } else {
    ROS_INFO("[Logger][%s%s] Turning off data logging", sensor_id_.c_str(), image_type_.c_str());

    // Close rosbag
    bag_.close();
    enable_ = false;
    res.message = sensor_id_+"_"+image_type_+": Logger OFF";
  }

  res.success = true;
  return true;
}

};

