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
    default_update_interval_(1){
  default_data_topics_ = {"/k1/color_image_count",
                          "/k2/color_image_count",
                          "/k3/color_image_count",
                          "/p1/color_image_count",
                          "/p2/color_image_count",
                          "/k1/depth_image_count",
                          "/k2/depth_image_count",
                          "/k3/depth_image_count",
                          "/synchronizer/image_count"};

  //load parameters from parameter server
  loadParameters();
};

void MessageMonitor::initialize(const std::string &hw_id) {
  // Setup subscribers
  setupSubscribers();

  //Setup diagnostics
  setupDiagnostics(hw_id);

  //Setup diagnostics update timer
  diagnostics_timer_ = pnh_->createTimer(ros::Duration(update_interval_), &MessageMonitor::updateDiagnostics, this);
}

void MessageMonitor::setupDiagnostics(const std::string &hw_id) {
  // Setup diagnostic updaters
  data_updater_.setHardwareID(hw_id);

  for(size_t i=0; i<data_topics_.size(); i++){
    boost::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic>
      temp(new diagnostic_updater::HeaderlessTopicDiagnostic(data_topics_[i],
                                                             data_updater_,
                                                             diagnostic_updater::FrequencyStatusParam(&min_frequency_,
                                                                                                      &max_frequency_,
                                                                                                      frequency_tolerance_,
                                                                                                      window_size_)));
    data_diagnostics_.push_back(temp);
  }
};

void MessageMonitor::updateDiagnostics(const ros::TimerEvent& t) {
  //Send diagnostics out
  data_updater_.update();
};

void MessageMonitor::loadParameters(){
  if (pnh_->hasParam("data_topics")) {
    pnh_->getParam("data_topics", data_topics_);
  } else {
    data_topics_ = default_data_topics_;
  }
  pnh_->param<double>("min_frequency", min_frequency_, default_frame_rate_);
  pnh_->param<double>("max_frequency", max_frequency_, default_frame_rate_);
  pnh_->param<int>("window_size", window_size_, default_window_size_);
  pnh_->param<double>("frequency_tolerance", frequency_tolerance_, default_frequency_tolerance_);
  pnh_->param<double>("min_acceptable_delay", min_acceptable_delay_, default_min_acceptable_delay_);
  pnh_->param<double>("max_acceptable_delay", max_acceptable_delay_, default_max_acceptable_delay_);
  pnh_->param<double>("update_interval", update_interval_, default_update_interval_);

  ROS_INFO_STREAM("[MessageMonitor] Num sensor topics: "<<data_topics_.size());
  for(const auto &s: data_topics_) {
    ROS_INFO_STREAM("[MessageMonitor] sensor topic: "<<s);
  }
  ROS_INFO_STREAM("[MessageMonitor] window size: "<<window_size_);
  ROS_INFO_STREAM("[MessageMonitor] frequency tolerance: "<<frequency_tolerance_);
  ROS_INFO_STREAM("[MessageMonitor] min acceptable delay: "<<min_acceptable_delay_);
  ROS_INFO_STREAM("[MessageMonitor] max acceptable delay: "<<max_acceptable_delay_);
  ROS_INFO_STREAM("[MessageMonitor] update interval: "<<update_interval_);
};

} // Namespace gear_dignostics
