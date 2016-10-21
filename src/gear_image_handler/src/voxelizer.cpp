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
    pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("voxel/points", 1);
    sub_ = nh_.subscribe ("rgb/points", 1, &Voxelizer::cloudCb, this);
  };

  void cloudCb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (0.1, 0.1, 0.1);
    sor.filter (cloud_filtered);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_filtered, output);

    // Publish the data
    pub_.publish (output);
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
