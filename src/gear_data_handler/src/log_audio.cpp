#include <gear_data_handler/log_audio.h>

// Boost Dependencies
#include <boost/format.hpp>

namespace gear_data_handler {

AudioLogger::AudioLogger(): BagLogger(){};

void AudioLogger::onInit(){

  // Initialize
  initialize(getNodeHandle(), getPrivateNodeHandle());

  // Loading tag type
  sub_ = getNodeHandle().subscribe("/log_audio", 5, &AudioLogger::audioCallback, this);
}

void AudioLogger::audioCallback(const audio_common_msgs::StampedAudioDataConstPtr &msg) {
  boost::mutex::scoped_lock(session_param_lock_);
  boost::mutex::scoped_lock(enable_lock_);

  if (enable_) {

    // Saving to disk
    bag_.write("audio_info", ros::Time::now(), msg);

    // Publish running image counts
    publishCount();
  }
}

};

