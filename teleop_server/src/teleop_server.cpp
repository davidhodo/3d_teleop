#include <string>
#include <sstream>
#include <time.h>

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "nav_msgs/GridCells.h"
#include "geometry_msgs/Point.h"

#include <octomap/octomap.h>
#include <octomap_ros/OctomapROS.h>
#include <octomap_ros/conversions.h>

using namespace std;

bool is_first_time = true;
bool running = false;
octomap::OcTreeROS *ros_tree;
tf::TransformListener *listener;
ros::Publisher *grid_cell_pub;
ros::Publisher *point_cloud_pub;
double resolution = 0.1;

string pointcloud2_to_str(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  stringstream ss;
  ss << "PointCloud2 received:" << endl;
  ss << msg->header;
  ss << msg->height << endl;
  ss << msg->width << endl;
  ss << msg->fields[0].count;
  return ss.str();
}

bool first_time_setup() {
  ros_tree = new octomap::OcTreeROS(resolution);
  
  tf::StampedTransform transform;
  
  try {
    listener->lookupTransform("/odom", "/openni_rgb_optical_frame", ros::Time(0), transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
    return false;
  }
  
  ROS_INFO("Origin at: %f, %f, %f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
  
  // Maybe set an origin here
  
  is_first_time = false;
  return true;
}

void publishOccupiedGridCells() {
  nav_msgs::GridCells msg;
  
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "/odom";
  
  msg.cell_height = resolution;
  msg.cell_width = resolution;
  
  list<octomap::OcTreeVolume> occupied_leafs;
  
  ros_tree->octree.getOccupied(occupied_leafs);
  
  list<octomap::OcTreeVolume>::const_iterator it = occupied_leafs.begin(), end = occupied_leafs.end();
  
  msg.cells.resize(occupied_leafs.size());
  
  size_t count = 0;
  for(; it != end; ++it) {
    geometry_msgs::Point pt;
    pt.y = it->first.x();
    pt.z = it->first.y();
    pt.x = it->first.z();
    
    msg.cells.push_back(pt);
    
    ++count;
  }
  
  grid_cell_pub->publish(msg);
}

void publishOccupiedPointCloud() {
  ros::Time start, end_t;
  start = ros::Time::now();
  sensor_msgs::PointCloud msg;
  
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "/odom";
  
  list<octomap::OcTreeVolume> occupied_leafs;
  
  ros_tree->octree.getOccupied(occupied_leafs);
  
  list<octomap::OcTreeVolume>::const_iterator it = occupied_leafs.begin(), end = occupied_leafs.end();
  
  msg.points.resize(occupied_leafs.size());
  
  for(; it != end; ++it) {
    geometry_msgs::Point32 pt;
    pt.y = it->first.x();
    pt.z = it->first.y();
    pt.x = it->first.z();
    msg.points.push_back(pt);
  }
  
  point_cloud_pub->publish(msg);
  
  end_t = ros::Time::now();
  ROS_INFO("Publishing the PointCloud took: %f seconds", (end_t-start).toSec());
}

void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  if (!running)
    return;
  ros::Time start, end;
  start = ros::Time::now();
  if (is_first_time)
    if (!first_time_setup())
      return;
  
  tf::StampedTransform transform;
  try {
    listener->lookupTransform("/base_link", "/openni_rgb_optical_frame", msg->header.stamp, transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
    return;
  }
  
  octomap::point3d origin(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
  
  ros_tree->insertScan(*msg, octomap::pointOctomapToMsg(origin));
  end = ros::Time::now();
  
  // publishOccupiedGridCells();
  publishOccupiedPointCloud();
  
  ROS_INFO("Pointcloud2 processing took: %f seconds", (end-start).toSec());
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "teleop_server");
  
  ros::NodeHandle n;
  
  listener = new tf::TransformListener();
  
  ros::Subscriber sub = n.subscribe("cloud_throttled", 1, pointcloudCallback);
  
  ros::Publisher temp = n.advertise<nav_msgs::GridCells>("/octomap", 1000);
  grid_cell_pub = &temp;
  
  temp = n.advertise<sensor_msgs::PointCloud>("/octomap_points", 1000);
  point_cloud_pub = &temp;
  
  running = true;
  
  ros::spin();
  
  running = false;
  
  ROS_INFO("Writing binary map file.");
  ros_tree->octree.writeBinary("map.bt");
  
  return 0;
}