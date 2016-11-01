/*
 * message_monitor.h
 *
 *  Created on: Oct 31, 2016
 *      Author: cooplab-gear
 */

#ifndef MESSAGE_MONITOR_H_
#define MESSAGE_MONITOR_H_

// c++ Dependencies
#include <string>
#include <vector>

// ROS dependencies
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

// Boost Dependencies
#include <boost/shared_ptr.hpp>

namespace gear_diagnostics {

class MessageMonitor {
public:
  /*
   * Constructor
   */
  MessageMonitor();

  /*
   * Destructor
   */
  ~MessageMonitor(){};

private:
  /*
   * Load parameters from parameter server
   */
  void loadParameters();

  /*
   * Setup diagnostics components
   */
  void setupDiagnostics();

  /*
   * Update diagnostics function
   */
  void updateDiagnostics(const ros::TimerEvent& t);

  // Default parameters
  std::string default_synchronizer_topic_;
  std::vector<std::string> default_sensor_logger_topics_;
  int default_frame_rate_, default_window_size_;
  double default_frequency_tolerance_, default_min_acceptable_delay_, default_max_acceptable_delay_;
  double default_update_interval_;

  // Parameters
  std::string synchronizer_topic_;
  std::vector<std::string> sensor_logger_topics_;
  int frame_rate_, window_size_;
  double min_frequency_, max_frequency_;
  double frequency_tolerance_, min_acceptable_delay_, max_acceptable_delay_;
  double update_interval_;

  // Node handles
  boost::shared_ptr<ros::NodeHandle> pnh_;

  // Diagnostics updaters
  diagnostic_updater::Updater synchronizer_updater_, logger_updater_;
  boost::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> synchronizer_diagnostics_;
  std::vector<boost::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic>> sensor_logger_diagnostics_;

  //Timer
  ros::Timer diagnostics_timer_;
};

} // Namespace gear_dignostics

#endif /* MESSAGE_MONITOR_H_ */
