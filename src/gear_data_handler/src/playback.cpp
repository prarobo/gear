#include <gear_data_handler/playback.h>

namespace gear_data_handler {
Playback::Playback(ros::NodeHandle nh, ros::NodeHandle pnh) {
  // Get node handles
  nh_.reset(new ros::NodeHandle(nh));
  pnh_.reset(new ros::NodeHandle(pnh));

  //Load associated parameters
  loadParams();

  //Load image information
  loadImageInfo();
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

}

};

int main(int argc, char **argv){
  ros::init(argc, argv, "apriltag_detector");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  gear_data_handler::Playback player(nh, pnh);
  ros::spin();
}
