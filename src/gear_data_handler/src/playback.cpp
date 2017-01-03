#include <gear_data_handler/playback.h>

// Boost dependencies
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

namespace fs=boost::filesystem;

namespace gear_data_handler {
Playback::Playback(ros::NodeHandle nh, ros::NodeHandle pnh) {
  // Get node handles
  nh_.reset(new ros::NodeHandle(nh));
  pnh_.reset(new ros::NodeHandle(pnh));

  // Enabled
  enabled_ = false;

  // Create service to enable playback
  enable_playback_ = nh_->advertiseService("enable_playback", &Playback::startPlayback, this);
}

void Playback::initializePlayback() {
  //Load associated parameters
  loadParams();

  //Load image information
  loadImageInfo();
}

void Playback::play() {
}

bool Playback::startPlayback(std_srvs::Trigger::Request  &req,
                             std_srvs::Trigger::Response &res) {

  if (!enabled_) {
    // Set the enable parameter based on service call value
    ROS_INFO("[Playback] Starting playback");
    res.message = "Playback ON";
    enabled_ = true;

    // Initialize playback
    initializePlayback();

    // Play messages
    play();

    res.success = true;
    return true;
  } else {
    res.message = "Playback already started, doing nothing!";
    res.success = false;
    return false;
  }
}

void Playback::loadParams() {
  // Getting session wide parameters
  nh_->param<std::string>("/subject_id", subject_id_,"x");
  nh_->param<std::string>("/session_id", session_id_,"session_1");
  nh_->param<std::string>("/activity_id", activity_id_,"act");
  nh_->param<std::string>("/condition_id", condition_id_,"in");
  nh_->param<int>("/trial_id", trial_id_, 1);

  // Getting node specific parameters
  pnh_->param<std::string>("data_dir", data_dir_, "/mnt/md0/gear_data");
  pnh_->param<std::string>("image_extn", image_extn_, ".jpg");
  pnh_->param<std::string>("image_prefix", image_prefix_, "im");
}

void Playback::loadImageInfo() {

  // Get all directories where images are present
  fs::path image_root_dir = fs::path(data_dir_)/fs::path(subject_id_)/fs::path(session_id_)/
                            fs::path(activity_id_+std::string("_")+condition_id_+std::string("_")+std::to_string(trial_id_))/
                            fs::path("image");

  // Iterate over all images over directories in image root directory
  if ( fs::exists(image_root_dir) && fs::is_directory(image_root_dir))  {
    for( fs::directory_iterator dir_iter(image_root_dir) ; dir_iter != fs::directory_iterator{} ; ++dir_iter) {
      if (fs::is_directory(dir_iter->status())) {
        for( fs::directory_iterator im_iter(*dir_iter) ; im_iter != fs::directory_iterator() ; ++im_iter) {
          if (fs::is_regular_file(im_iter->status()) ) {
            std::string image_name = im_iter->path().filename().string();
            image_info_.insert(std::multimap<double,std::string>::value_type(parseTimeStamp(image_name),
                                                                             dir_iter->path().filename().string()));
          }
        }
      }
    }
  }
}

double Playback::parseTimeStamp(const std::string &image_name) {
  std::vector<std::string> strs;
  boost::split(strs, image_name, boost::is_any_of("_."));
  return boost::lexical_cast<double>(strs[1]+std::string(".")+strs[2]);
}

};

int main(int argc, char **argv){
  ros::init(argc, argv, "apriltag_detector");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  gear_data_handler::Playback player(nh, pnh);
  ros::spin();
}
