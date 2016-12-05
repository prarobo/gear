#include <gear_data_handler/log_images.h>

#include <cv.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>

// Ros Dependencies
#include <std_msgs/Int64.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/transport_hints.h>

// Boost Dependencies
#include <boost/format.hpp>

namespace enc = sensor_msgs::image_encodings;

namespace gear_data_handler {

ImageLogger::ImageLogger(): Logger(){};

void ImageLogger::onInit(){

  // Initialize
  initialize(getNodeHandle(), getPrivateNodeHandle());

  // Getting node specific parameters
  bool is_compressed;
  Logger::pnh_->getParam("is_compressed", is_compressed);

  // Create image transport subscriber
  boost::shared_ptr<image_transport::TransportHints> hints;
  if (is_compressed) {
    hints.reset(new image_transport::TransportHints("compressed", ros::TransportHints()));
  } else {
    hints.reset(new image_transport::TransportHints("raw", ros::TransportHints()));
  }
  it_.reset(new image_transport::ImageTransport(*Logger::nh_));
  image_sub_ = it_->subscribe("/log_image", 5, &ImageLogger::imageCallback, this, *hints);

  // Setting encoding
  if (image_type_.compare("color") == 0) {
    encoding_ = enc::BGR8;
  }

  if (image_type_.compare("depth") == 0) {
    encoding_ = enc::TYPE_16UC1;
  }
}

void ImageLogger::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  boost::mutex::scoped_lock(session_param_lock_);
  boost::mutex::scoped_lock(enable_lock_);

  if (enable_) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try  {
      cv_ptr = cv_bridge::toCvShare(msg, encoding_);
    }
    catch (cv_bridge::Exception& e)  {
      NODELET_ERROR("[ImageLogger] Could not convert from '%s' to %s.",
                    msg->encoding.c_str(), encoding_.c_str());
    }

    // Parse timestamp
    ros::Time stamp = msg->header.stamp;
    boost::format nsec("%09d");
    nsec % stamp.nsec;

    // Construct the filename
    boost::filesystem::path image_path = getFilePath(std::to_string(stamp.sec), nsec.str());

    //Write to disk
    cv::imwrite(image_path.string(), cv_ptr->image);

    // Publish running image counts
    publishCount();
  }
}
};

