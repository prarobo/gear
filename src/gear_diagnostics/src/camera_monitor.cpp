/*
 * camera_monitor.cpp
 *
 *  Created on: Feb 2, 2017
 *      Author: gear
 */
#include <gear_diagnostics/message_monitor.h>

namespace gear_diagnostics {

CameraMonitor::CameraMonitor(): MessageMonitor(){};

void CameraMonitor::registerEvent(const ros::MessageEvent<sensor_msgs::CameraInfo const>& event) {

  const ros::M_string& header = event.getConnectionHeader();
  std::string topic = header.at("topic");
  ROS_DEBUG("[MessageMonitor] Received message from publisher: %s", topic.c_str());

  // Update tick
  for (size_t i=0; i<data_topics_.size(); i++) {
    if (topic.compare(data_topics_[i])==0) {
      data_diagnostics_[i]->tick();
      ROS_DEBUG("[MessageMonitor] Updated tick for publisher diagnostic: %s", topic.c_str());
      return;
    }
  }
};

void CameraMonitor::setupSubscribers() {
  for(size_t i=0; i<data_topics_.size(); i++){
    data_sub_.emplace_back(boost::shared_ptr<ros::Subscriber>(new ros::Subscriber(pnh_->subscribe(data_topics_[i],
                                                                                  5, &CameraMonitor::registerEvent, this))));
  }
};

} // Namespace gear_dignostics

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "camera_monitor");

  // Create a message monitor
  gear_diagnostics::CameraMonitor monitor;

  // Initialize camera monitor
  monitor.initialize("camera");

  // Spin
  ros::spin ();
  return 0;
}


