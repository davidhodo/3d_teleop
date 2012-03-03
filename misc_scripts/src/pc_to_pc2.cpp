#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"

ros::Publisher *pub;

void pcCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
  sensor_msgs::PointCloud2 pc2;
  sensor_msgs::convertPointCloudToPointCloud2(*msg, pc2);
  pub->publish(pc2);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pc_to_pc2");

  ros::NodeHandle n;

  ros::Publisher temp = n.advertise<sensor_msgs::PointCloud2>("velodyne_pc2", 1000);
  pub = &temp;
  ros::Subscriber sub = n.subscribe("/Truck/Velodyne", 1000, pcCallback);

  ros::spin();

  return 0;
}
