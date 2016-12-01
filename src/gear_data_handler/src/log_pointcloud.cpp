#include "../include/gear_data_handler/log_pointcloud.h"

#include <cv_bridge/cv_bridge.h>

// Boost Dependencies
#include <boost/format.hpp>

//PCL Dependencies
#include <pcl/io/pcd_io.h>

namespace gear_data_handler {

PointcloudLogger::PointcloudLogger(): Logger(){};

void PointcloudLogger::onInit(){

  // Initialize
  initialize(getNodeHandle(), getPrivateNodeHandle());

  pointcloud_sub_ = getNodeHandle().subscribe("/log_pointcloud", 5, &PointcloudLogger::pointcloudCallback, this);
}

void PointcloudLogger::pointcloudCallback(const pcl::PCLPointCloud2::ConstPtr& msg) {
  boost::mutex::scoped_lock(session_param_lock_);
  boost::mutex::scoped_lock(enable_lock_);

  if (enable_) {

    // Parse timestamp
    ros::Time stamp;
    stamp.fromNSec(msg->header.stamp);
    boost::format nsec("%09d");
    nsec % stamp.nsec;

    // Construct the filename
    boost::filesystem::path pointcloud_path = getFilePath(std::to_string(stamp.sec), nsec.str());

    // Saving to disk
    pcl::PCDWriter writer;
    writer.writeBinaryCompressed(pointcloud_path.string(), *msg);

    // Publish running image counts
    publishCount();
  }
}

};

