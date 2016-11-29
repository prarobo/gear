/*
 * message_monitor.cpp
 *
 *  Created on: Oct 31, 2016
 *      Author: cooplab-gear
 */

#include <gear_diagnostics/message_monitor.h>

namespace gear_diagnostics {

MessageMonitor::MessageMonitor():
    pnh_(new ros::NodeHandle("~")),
    default_frame_rate_(15),
    default_window_size_(150),
    default_frequency_tolerance_(0.1),
    default_min_acceptable_delay_(0.0),
    default_max_acceptable_delay_(0.2),
    default_update_interval_(1),
    default_synchronizer_topic_("/synchronizer/image_count"){
  default_sensor_logger_topics_ = {"/k1/color_image_count",
                                  "/k2/color_image_count",
                                  "/k3/color_image_count",
                                  "/p1/color_image_count",
                                  "/p2/color_image_count",
                                  "/k1/depth_image_count",
                                  "/k2/depth_image_count",
                                  "/k3/depth_image_count"};

  //load parameters from parameter server
  loadParameters();

  // Setup subscribers
  setupSubscribers();

  //Setup diagnostics
  setupDiagnostics();

  //Setup diagnostics update timer
  diagnostics_timer_ = pnh_->createTimer(ros::Duration(update_interval_), &MessageMonitor::updateDiagnostics, this);
};

void MessageMonitor::setupSubscribers() {
  // Setup synchronizer subscriber
  synchronizer_sub_.reset(new ros::Subscriber(pnh_->subscribe(synchronizer_topic_, 5, &MessageMonitor::registerEvent, this)));
  for(size_t i=0; i<sensor_logger_topics_.size(); i++){
    sensor_logger_sub_.emplace_back(boost::shared_ptr<ros::Subscriber>(new ros::Subscriber(pnh_->subscribe(sensor_logger_topics_[i],
    																					                                             5, &MessageMonitor::registerEvent, this))));
  }
};

void MessageMonitor::setupDiagnostics() {
  // Setup diagnostic updaters
  synchronizer_updater_.setHardwareID("synchronizer_monitor");
  logger_updater_.setHardwareID("logger_monitor");

  // Setup diagnostic publishers
  synchronizer_diagnostics_.reset(new diagnostic_updater::HeaderlessTopicDiagnostic(
                                       synchronizer_topic_,
                                       synchronizer_updater_,
                                       diagnostic_updater::FrequencyStatusParam(&min_frequency_, &max_frequency_,
                                                                                frequency_tolerance_, window_size_)));
  for(size_t i=0; i<sensor_logger_topics_.size(); i++){
    boost::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic>
      temp(new diagnostic_updater::HeaderlessTopicDiagnostic(sensor_logger_topics_[i],
                                                             logger_updater_,
                                                             diagnostic_updater::FrequencyStatusParam(&min_frequency_,
                                                                                                      &max_frequency_,
                                                                                                      frequency_tolerance_,
                                                                                                      window_size_)));
    sensor_logger_diagnostics_.push_back(temp);
  }
};

void MessageMonitor::registerEvent(const ros::MessageEvent<std_msgs::Int64 const>& event) {

  const ros::M_string& header = event.getConnectionHeader();
  std::string topic = header.at("topic");
  ROS_DEBUG("[MessageMonitor] Received message from publisher: %s", topic.c_str());

  // Update tick
  if (topic.compare(synchronizer_topic_)==0) {
	  synchronizer_diagnostics_->tick();
	  ROS_DEBUG("[MessageMonitor] Updated tick for publisher diagnostic: %s", topic.c_str());
	  return;
  }

  for (size_t i=0; i<sensor_logger_topics_.size(); i++) {
    if (topic.compare(sensor_logger_topics_[i])==0) {
      sensor_logger_diagnostics_[i]->tick();
      ROS_DEBUG("[MessageMonitor] Updated tick for publisher diagnostic: %s", topic.c_str());
      return;
    }
  }
};

void MessageMonitor::updateDiagnostics(const ros::TimerEvent& t) {
  //Send diagnostics out
  logger_updater_.update();
  synchronizer_updater_.update();
};

void MessageMonitor::loadParameters(){
  pnh_->param<std::string>("synchronizer_topic", synchronizer_topic_, default_synchronizer_topic_);
  pnh_->param<std::vector<std::string>>("sensor_logger_topics", sensor_logger_topics_, default_sensor_logger_topics_);
  pnh_->param<int>("frame_rate", frame_rate_, default_frame_rate_);
  pnh_->param<double>("min_frequency", min_frequency_, default_frame_rate_);
  pnh_->param<double>("max_frequency", max_frequency_, default_frame_rate_);
  pnh_->param<int>("window_size", window_size_, default_window_size_);
  pnh_->param<double>("frequency_tolerance", frequency_tolerance_, default_frequency_tolerance_);
  pnh_->param<double>("min_acceptable_delay", min_acceptable_delay_, default_min_acceptable_delay_);
  pnh_->param<double>("max_acceptable_delay", max_acceptable_delay_, default_max_acceptable_delay_);
  pnh_->param<double>("update_interval", update_interval_, default_update_interval_);

  ROS_INFO_STREAM("[MessageMonitor] synchronizer topic: "<<synchronizer_topic_);
  for(const auto &s: sensor_logger_topics_) {
    ROS_INFO_STREAM("[MessageMonitor] sensor topic: "<<s);
  }
  ROS_INFO_STREAM("[MessageMonitor] frame rate: "<<frame_rate_);
  ROS_INFO_STREAM("[MessageMonitor] window size: "<<window_size_);
  ROS_INFO_STREAM("[MessageMonitor] frequency tolerance: "<<frequency_tolerance_);
  ROS_INFO_STREAM("[MessageMonitor] min acceptable delay: "<<min_acceptable_delay_);
  ROS_INFO_STREAM("[MessageMonitor] max acceptable delay: "<<max_acceptable_delay_);
  ROS_INFO_STREAM("[MessageMonitor] update interval: "<<update_interval_);
};

} // Namespace gear_dignostics

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "message_monitor");

  // Create a message monitor
  gear_diagnostics::MessageMonitor monitor;

  // Spin
  ros::spin ();
  return 0;
}
