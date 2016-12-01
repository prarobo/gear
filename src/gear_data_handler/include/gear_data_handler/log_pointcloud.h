#ifndef LOG_POINTCLOUD_H_
#define LOG_POINTCLOUD_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <gear_data_handler/logger.h>

// PCL Dependencies
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace gear_data_handler {

class PointcloudLogger : public nodelet::Nodelet, public Logger {

public:
  PointcloudLogger();

  ~PointcloudLogger() {};

  /**
   * Implements the Nodelet interface
   **/
  void onInit();

  /**
   * Pointcloud callback function
   **/
  void pointcloudCallback(const pcl::PCLPointCloud2::ConstPtr& msg);

private:
  ros::Subscriber pointcloud_sub_;

}; //class PointcloudLogger

PLUGINLIB_DECLARE_CLASS(gear_data_handler, PointcloudLogger,
                        gear_data_handler::PointcloudLogger,
                        nodelet::Nodelet);

} // namespace image_handler

#endif // LOG_POINTCLOUD_H_
