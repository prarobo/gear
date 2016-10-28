#ifndef SYNCHRONIZE_IMAGES_H_
#define SYNCHRONIZE_IMAGES_H_

// ROS Dependencies
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Boost Dependencies
#include <boost/thread/mutex.hpp>

typedef message_filters::sync_policies::ApproximateTime
          <sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image,
           sensor_msgs::Image,sensor_msgs::Image, sensor_msgs::Image,
           sensor_msgs::Image, sensor_msgs::Image> gear_sync_policy;

namespace gear_image_handler {

class ImageSynchronizer : public nodelet::Nodelet {

public:
  ImageSynchronizer();

  ~ImageSynchronizer() {};

  /**
     Implements the Nodelet interface
   **/
  void onInit();

  /**
     Image callback function
   **/
  void imageCallback(const sensor_msgs::ImageConstPtr& msg0,
                     const sensor_msgs::ImageConstPtr& msg1,
                     const sensor_msgs::ImageConstPtr& msg2,
                     const sensor_msgs::ImageConstPtr& msg3,
                     const sensor_msgs::ImageConstPtr& msg4,
                     const sensor_msgs::ImageConstPtr& msg5,
                     const sensor_msgs::ImageConstPtr& msg6,
                     const sensor_msgs::ImageConstPtr& msg7);

  /**
   * Image logger service callback
   */
  bool toggleLogger(std_srvs::SetBool::Request  &req,
                    std_srvs::SetBool::Response &res);

  /**
   * Parse data topic into base name and image type
   */
  void parseDataTopic(std::string topic, std::string &base_name, std::string &image_type);


private:
  std::vector<boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>> image_sub_;
  std::vector<ros::Publisher> image_pub_;
  ros::Publisher image_count_pub_;
  ros::ServiceServer toggle_logger_;
  boost::shared_ptr<message_filters::Synchronizer<gear_sync_policy>> sync_;

  bool enable_;
  boost::recursive_mutex enable_lock_;

  int image_count_;
  std::vector<std::string> default_data_topics_;

}; //class ImageLogger

PLUGINLIB_DECLARE_CLASS(gear_image_handler, ImageSynchronizer,
                        gear_image_handler::ImageSynchronizer,
                        nodelet::Nodelet);

} // namespace image_handler

#endif // SYNCHRONIZE_IMAGES_H_
