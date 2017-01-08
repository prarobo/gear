#include <gear_data_handler/playback.h>

// ROS dependencies
#include <rosgraph_msgs/Clock.h>
#include <cv_bridge/cv_bridge.h>

// Boost dependencies
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

// OpenCV dependencies
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace gear_data_handler {
Playback::Playback(ros::NodeHandle nh, ros::NodeHandle pnh, int clock_frequency) {
  boost::mutex::scoped_lock(enable_lock_);

  // Get node handles
  nh_.reset(new ros::NodeHandle(nh));
  pnh_.reset(new ros::NodeHandle(pnh));

  // Enabled
  enabled_ = false;

  // Clock frequency
  clock_frequency_ = clock_frequency;

  // Time publisher
  clock_pub_ = nh_->advertise<rosgraph_msgs::Clock>("clock",1);

  // Image transport
  it_.reset(new image_transport::ImageTransport(*nh_));

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
  boost::mutex::scoped_lock(enable_lock_);
  if(enabled_) {
    std::multimap<ros::Time, std::string>::iterator it=image_info_.begin();
    ros::Time sim_t_current = it->first;
    ros::Time sim_t_next = it->first+ros::Duration(1/clock_frequency_);
    ros::Duration t_diff(0);

    while (it != image_info_.end()) {
      ros::Time sim_next_msg_t = it->first;
      bool msg_published = false;

      if (sim_next_msg_t<=sim_t_next) {
        t_diff = ros::Duration((sim_next_msg_t.toSec()-sim_t_current.toSec())/rate_);
        sim_t_current = sim_next_msg_t;

        // Publish message
        publishImage(it);
        it++;

        msg_published = true;
      }

      if(!msg_published || sim_t_current == sim_t_next) {
        t_diff = ros::Duration((sim_t_next.toSec()-sim_t_current.toSec())/rate_);
        sim_t_current = sim_t_next;
        sim_t_next = sim_t_current+ros::Duration(1/clock_frequency_);
      }

      // Wait for some time
      ros::Time::sleepUntil(ros::Time::now()+t_diff);

      // Publish current time
      rosgraph_msgs::Clock clk;
      clk.clock = sim_t_current;
      clock_pub_.publish(clk);
    }
  }
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
  pnh_->param<std::string>("image_def", image_def_, "hd");
}

void Playback::loadImageInfo() {

  // Get all directories where images are present
  image_root_dir_ = fs::path(data_dir_)/fs::path(subject_id_)/fs::path(session_id_)/
                    fs::path(activity_id_+std::string("_")+condition_id_+std::string("_")+std::to_string(trial_id_))/
                    fs::path("image");

  // Iterate over all images over directories in image root directory
  if ( fs::exists(image_root_dir_) && fs::is_directory(image_root_dir_))  {
    for( fs::directory_iterator dir_iter(image_root_dir_) ; dir_iter != fs::directory_iterator{} ; ++dir_iter) {
      if (fs::is_directory(dir_iter->status())) {
        std::string dir_name = dir_iter->path().filename().string();
        createPublisher(dir_name);
        for( fs::directory_iterator im_iter(*dir_iter) ; im_iter != fs::directory_iterator() ; ++im_iter) {
          if (fs::is_regular_file(im_iter->status()) ) {
            std::string image_name = im_iter->path().filename().string();
            image_info_.insert(std::multimap<ros::Time,std::string>::value_type(parseTimeStamp(image_name),
                                                                                dir_name));
          }
        }
      }
    }
  }
}

ros::Time Playback::parseTimeStamp(const std::string &image_name) {
  std::vector<std::string> strs;
  boost::split(strs, image_name, boost::is_any_of("_."));
  ros::Time t(boost::lexical_cast<uint32_t>(strs[1]), boost::lexical_cast<uint32_t>(strs[2]));
  return t;
}

void Playback::publishImage(const std::multimap <ros::Time, std::string>::iterator &image_it) {
  // Get image file path
  std::string image_name = image_prefix_+std::to_string(image_it->first.sec)
                           +std::string("_")+std::to_string(image_it->first.nsec)
                           +image_extn_;
  fs::path image_path = image_root_dir_/fs::path(image_it->second)/fs::path(image_name);

  // Load image
  cv_bridge::CvImage cv_image;
  cv_image.image = cv::imread(image_path.string(),CV_LOAD_IMAGE_COLOR);
  cv_image.encoding = "bgr8";
  sensor_msgs::Image ros_image;
  cv_image.toImageMsg(ros_image);

  // Publish
  image_pub_.at(image_it->second)->publish(ros_image);
}

void Playback::createPublisher(const std::string &dir_name) {
  // Check if the specified publisher exists
  if(image_pub_.find(dir_name) == image_pub_.end()) {

    // Parse directory name to get publisher details
    std::vector<std::string> strs;
    boost::split(strs, dir_name, boost::is_any_of("_"));
    std::string msg_name = std::string("/")+strs[0]+std::string("/")+image_def_
                           +std::string("/")+std::string("image_")+strs[1];
    boost::shared_ptr<image_transport::Publisher> pub(new image_transport::Publisher(it_->advertise(msg_name, 1)));
    image_pub_[dir_name] = pub;
  }
}

};

int main(int argc, char **argv){
  ros::init(argc, argv, "apriltag_detector");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  gear_data_handler::Playback player(nh, pnh);
  while(ros::ok()) {
    player.play();
    ros::spinOnce();
  }
}
