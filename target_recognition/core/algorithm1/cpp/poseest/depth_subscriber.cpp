#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//添加
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>

using namespace std;
 
ros::Publisher pub;
 

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;
  //相机获得原始点云文件
  pcl::io::savePCDFile("./pointcloud_init.pcd", *cloud_msg);
  cout<<"publish point_cloud height = "<<cloud_msg->height<<endl;
  cout<<"publish point_cloud width = "<<cloud_msg->width<<endl;
}
 

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "VoxelGrid");
  ros::NodeHandle nh;
 
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 1, cloud_cb);
 
  // Spin
  ros::spin ();
}
