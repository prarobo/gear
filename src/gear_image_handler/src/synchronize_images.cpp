#include <gear_image_handler/synchronize_images.h>

// Opencv dependencies
#include <cv.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>

// Ros Dependencies
#include <std_msgs/Int64.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/transport_hints.h>

// Boost Dependencies
#include <boost/algorithm/string.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/assign/list_of.hpp>

namespace gear_image_handler {

ImageSynchronizer::ImageSynchronizer():
    image_count_(0),
    enable_(false){
  default_data_topics_ = {"/k1/hd/image_color",
                          "/k2/hd/image_color",
                          "/k3/hd/image_color",
                          "/k1/sd/image_depth",
                          "/k2/sd/image_depth",
                          "/k3/sd/image_depth",
                          "/p1/hd/image_color",
                          "/p2/hd/image_color"};
};

void ImageSynchronizer::onInit(){

  // Getting the node handles
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle pnh = getPrivateNodeHandle();

  // Getting topic names
  std::vector<std::string> data_topics;
  pnh.param<std::vector<std::string>>("data_topics", data_topics, default_data_topics_);

  // Create image subscribers and publishers
  for(const std::string &t: data_topics) {
    NODELET_INFO_STREAM("[ImageSynchronizer] Creating subscriber for: "<<t);
    boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> sub(new message_filters::Subscriber<sensor_msgs::Image>(nh, t, 8));
    image_sub_.push_back(sub);

    //Parse data topic to get basename and image type
    std::string base_name, image_type;
    parseDataTopic(t, base_name, image_type);

    std::string topic_name = std::string("/synchronizer/")+base_name+std::string("_")+image_type;
    image_pub_.emplace_back(nh.advertise<sensor_msgs::Image>(topic_name, 5));
  }
  image_count_pub_ = nh.advertise<std_msgs::Int64>("/synchronizer_image_count", 5);

  //TODO: Cleanup to handle arbitrary number of subscribers
  sync_.reset(new message_filters::Synchronizer<gear_sync_policy>(gear_sync_policy(4), *image_sub_[0], *image_sub_[1], *image_sub_[2],
                                                                  *image_sub_[3], *image_sub_[4], *image_sub_[5],
                                                                  *image_sub_[6], *image_sub_[7]));
  sync_->registerCallback(boost::bind(&ImageSynchronizer::imageCallback, this, _1, _2, _3, _4, _5, _6, _7, _8));

  // Create service to toggle image logging
  toggle_logger_ = getNodeHandle().advertiseService("/synchronizer_enable", &ImageSynchronizer::toggleLogger, this);
}

void ImageSynchronizer::imageCallback (const sensor_msgs::ImageConstPtr& msg0,
                                       const sensor_msgs::ImageConstPtr& msg1,
                                       const sensor_msgs::ImageConstPtr& msg2,
                                       const sensor_msgs::ImageConstPtr& msg3,
                                       const sensor_msgs::ImageConstPtr& msg4,
                                       const sensor_msgs::ImageConstPtr& msg5,
                                       const sensor_msgs::ImageConstPtr& msg6,
                                       const sensor_msgs::ImageConstPtr& msg7) {
  NODELET_DEBUG("[ImageSynchronizer] Image set available");
  boost::mutex::scoped_lock(enable_lock_);
  if (enable_) {
    std::vector<sensor_msgs::Image> msg = {*msg0, *msg1, *msg2, *msg3, *msg4, *msg5, *msg6, *msg7};

    // Get the timestamp of first message in set
    ros::Time stamp = msg[0].header.stamp;

    // Publish all messages in set with the first message timestamp
    for(size_t i=0; i<msg.size(); i++) {
      msg[i].header.stamp = stamp;
      image_pub_[i].publish(msg[i]);
    }

    // Publish running image counts
    image_count_++;
    std_msgs::Int64 count_msg;
    count_msg.data = image_count_;
    image_count_pub_.publish(count_msg);
  }
}

bool ImageSynchronizer::toggleLogger(std_srvs::SetBool::Request  &req,
                                     std_srvs::SetBool::Response &res){

  // Set the enable parameter based on service call value
  boost::mutex::scoped_lock(enable_lock_);
  if (req.data) {
    enable_ = true;
    res.message = std::string("Synchronizer: ImageLogger ON");
  } else {
    NODELET_INFO_STREAM("[ImageSychronizer] Turning off image synchronization");
    enable_ = false;
    res.message = std::string("Synchronizer: ImageLogger OFF");
  }

  res.success = true;
  return true;
}

void ImageSynchronizer::parseDataTopic(std::string topic, std::string &base_name, std::string &image_type) {
  std::vector<std::string> strs, strs1;
  boost::split(strs, topic, boost::is_any_of("/"));
  base_name = strs[1];
  boost::split(strs1, strs[3], boost::is_any_of("_"));
  image_type = strs1[1];
}

};

