#ifndef LOG_AUDIO_H_
#define LOG_AUDIO_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <gear_data_handler/log_bags.h>
#include <rosbag/bag.h>
#include <audio_common_msgs/StampedAudioData.h>

namespace gear_data_handler {

class AudioLogger : public nodelet::Nodelet, public BagLogger {

public:
  AudioLogger();

  ~AudioLogger() {};

  /**
   * Implements the Nodelet interface
   **/
  void onInit();

  /**
   * April-tag tracking callback function
   **/
  void audioCallback(const audio_common_msgs::StampedAudioDataConstPtr &msg);

}; //class PointcloudLogger

PLUGINLIB_DECLARE_CLASS(gear_data_handler, AudioLogger,
                        gear_data_handler::AudioLogger,
                        nodelet::Nodelet);

} // namespace gear_data_handler

#endif // LOG_AUDIO_H_
