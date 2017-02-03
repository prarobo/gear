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
#include <std_msgs/Int64.h>
#include <sensor_msgs/CameraInfo.h>

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
  virtual ~MessageMonitor(){};

  /*
   * Initalization function
   */
  void initialize(const std::string &hw_id);

protected:
  /*
   * Load parameters from parameter server
   */
  void loadParameters();

  /*
   * Setup diagnostics components
   */
  void setupDiagnostics(const std::string &hw_id);

  /*
   * Setup topic subscribers
   */
  virtual void setupSubscribers()=0;

  /*
   * Update diagnostics function
   */
  void updateDiagnostics(const ros::TimerEvent& t);

  // Default parameters
  std::vector<std::string> default_data_topics_;
  int default_frame_rate_, default_window_size_;
  double default_frequency_tolerance_, default_min_acceptable_delay_, default_max_acceptable_delay_;
  double default_update_interval_;

  // Parameters
  std::vector<std::string> data_topics_;
  int frame_rate_, window_size_;
  double min_frequency_, max_frequency_;
  double frequency_tolerance_, min_acceptable_delay_, max_acceptable_delay_;
  double update_interval_;

  // Node handles
  boost::shared_ptr<ros::NodeHandle> pnh_;

  // Diagnostics updaters
  diagnostic_updater::Updater data_updater_;
  std::vector<boost::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic>> data_diagnostics_;

  // Subscribers
  std::vector<boost::shared_ptr<ros::Subscriber>> data_sub_;

  //Timer
  ros::Timer diagnostics_timer_;
};

class LoggerMonitor: public MessageMonitor {
public:
  /*
   * Constructor
   */
  LoggerMonitor();

  /*
   * Destructor
   */
  ~LoggerMonitor(){};

private:
  /*
   * Setup topic subscribers
   */
  void setupSubscribers();

  /*
   * Register the arrival of a image count message
   */
  void registerEvent(const ros::MessageEvent<std_msgs::Int64 const>& event);

};

class CameraMonitor: public MessageMonitor {
public:
  /*
   * Constructor
   */
  CameraMonitor();

  /*
   * Destructor
   */
  ~CameraMonitor(){};

private:
  /*
   * Register the arrival of a image count message
   */
  void registerEvent(const ros::MessageEvent<sensor_msgs::CameraInfo const>& event);

  /*
   * Setup topic subscribers
   */
  void setupSubscribers();

};
} // Namespace gear_dignostics

#endif /* MESSAGE_MONITOR_H_ */
