#ifndef LOG_POINTCLOUD_H_
#define LOG_POINTCLOUD_H_

// ROS Dependencies
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <pcl_ros/point_cloud.h>

// PCL Dependencies
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Boost Dependencies
#include <boost/filesystem.hpp>
#include <boost/thread/mutex.hpp>

namespace gear_image_handler {

class PointcloudLogger : public nodelet::Nodelet {

public:
  PointcloudLogger();

  ~PointcloudLogger() {};

  /**
   * Implements the Nodelet interface
   **/
  void onInit();

  /**
   * Initialized the directories where images are to be stored
   */
  void initializeSessionDirectories();

  /**
   * Pointcloud callback function
   **/
  void pointcloudCallback(const pcl::PCLPointCloud2::ConstPtr& msg);

  /**
   * Pointcloud logger service callback
   */
  bool toggleLogger(std_srvs::SetBool::Request  &req,
                    std_srvs::SetBool::Response &res);

  /**
   * Session information service callback
   */
  bool setSessionInfo(std_srvs::Trigger::Request  &req,
                      std_srvs::Trigger::Response &res);

private:
  ros::Subscriber pointcloud_sub_;
  ros::Publisher pointcloud_count_pub_;
  ros::ServiceServer toggle_logger_;
  ros::ServiceServer session_info_;

  std::string session_id_;
  std::string activity_id_;
  std::string trial_id_;

  std::string sensor_id_;
  std::string data_dir_;
  std::string base_name_;

  bool enable_;
  boost::recursive_mutex enable_lock_, session_param_lock_;

  int pointcloud_count_;

  boost::filesystem::path pointcloud_dir_;
}; //class PointcloudLogger

PLUGINLIB_DECLARE_CLASS(gear_image_handler, PointcloudLogger,
                        gear_image_handler::PointcloudLogger,
                        nodelet::Nodelet);

} // namespace image_handler

#endif // LOG_POINTCLOUD_H_
