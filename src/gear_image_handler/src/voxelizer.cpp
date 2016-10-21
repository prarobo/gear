// C++ Dependencies
#include <iostream>

// ROS Dependencies
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/point_cloud.h>

// PCL Dependencies
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>

namespace gear_image_handler {

class Voxelizer : public nodelet::Nodelet{
public:
  Voxelizer(){};
  void onInit(){
    nh_ = getNodeHandle();
    pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("voxel/points", 1);
    sub_ = nh_.subscribe ("rgb/points", 1, &Voxelizer::cloudCb, this);
  };

  void cloudCb(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud){

    pcl::PointCloud<pcl::PointXYZRGB> voxel_cloud;

    // Create the filtering object
    pcl::VoxelGrid<pcl::PointCloud<pcl::PointXYZ>> sor;
    /*sor.setInputCloud (*cloud);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (voxel_cloud);*/

    pub_.publish(voxel_cloud);
  }
private:
  double voxel_size_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::NodeHandle nh_;
}; //Class Voxelizer

PLUGINLIB_DECLARE_CLASS(gear_image_handler, Voxelizer,
                        gear_image_handler::Voxelizer,
                        nodelet::Nodelet);

} // Namespace gear_image_handler
