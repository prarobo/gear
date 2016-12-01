#ifndef LOG_IMAGES_H_
#define LOG_IMAGES_H_

#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Image.h>
#include "../gear_data_handler/logger.h"

namespace gear_data_handler {

class ImageLogger : public nodelet::Nodelet, public Logger {

public:
  ImageLogger();

  ~ImageLogger() {};

  /**
     Implements the Nodelet interface
   **/
  void onInit();

  /**
     Image callback function
   **/
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

private:
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber image_sub_;

  std::string encoding_;
}; //class ImageLogger

PLUGINLIB_DECLARE_CLASS(gear_data_handler, ImageLogger,
                        gear_data_handler::ImageLogger,
                        nodelet::Nodelet);

} // namespace image_handler

#endif // LOG_IMAGES_H_
