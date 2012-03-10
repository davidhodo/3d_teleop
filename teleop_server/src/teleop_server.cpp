#include <string>
#include <sstream>
#include <time.h>

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "nav_msgs/GridCells.h"
#include "geometry_msgs/Point.h"
#include "pcl_ros/transforms.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "std_msgs/ColorRGBA.h"
#include "visualization_msgs/MarkerArray.h"

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
double resolution = 0.025;

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
    listener->lookupTransform("/map", "/openni_camera", ros::Time(0), transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
    return false;
  }
  
  ROS_INFO("Origin at: %f, %f, %f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
  
  // Maybe set an origin here
  
  is_first_time = false;
  return true;
}

// void publishOccupiedGridCells() {
//   nav_msgs::GridCells msg;
//   
//   msg.header.stamp = ros::Time::now();
//   msg.header.frame_id = "/map";
//   
//   msg.cell_height = resolution;
//   msg.cell_width = resolution;
//   
//   list<octomap::OcTreeVolume> occupied_leafs;
//   
//   ros_tree->octree.getOccupied(occupied_leafs);
//   
//   list<octomap::OcTreeVolume>::const_iterator it = occupied_leafs.begin(), end = occupied_leafs.end();
//   
//   msg.cells.resize(occupied_leafs.size());
//   
//   size_t count = 0;
//   for(; it != end; ++it) {
//     geometry_msgs::Point pt;
//     pt.y = it->first.x();
//     pt.z = it->first.y();
//     pt.x = it->first.z();
//     
//     msg.cells.push_back(pt);
//     
//     ++count;
//   }
//   
//   grid_cell_pub->publish(msg);
// }

// void publishOccupiedPointCloud() {
//   ros::Time start, end_t;
//   start = ros::Time::now();
//   sensor_msgs::PointCloud msg, tf_msg;
  
//   msg.header.stamp = ros::Time::now();
//   msg.header.frame_id = "/octomap";
  
//   list<octomap::OcTreeVolume> occupied_leafs;
  
//   ros_tree->octree.getOccupied(occupied_leafs);
  
//   list<octomap::OcTreeVolume>::const_iterator it = occupied_leafs.begin(), end = occupied_leafs.end();
  
//   msg.points.resize(occupied_leafs.size());
  
//   for(; it != end; ++it) {
//     geometry_msgs::Point32 pt;
//     pt.y = it->first.x();
//     pt.z = it->first.y();
//     pt.x = it->first.z();
//     msg.points.push_back(pt);
//   }
  
// //  tf::Transform transform(btTransform(btQuaternion(0.0,90.0,0.0)));
  
//   point_cloud_pub->publish(msg);
  
//   end_t = ros::Time::now();
//   ROS_INFO("Publishing the PointCloud took: %f seconds", (end_t-start).toSec());
// }

std_msgs::ColorRGBA marker_color;

void publishMarkers() {
  list<octomap::OcTreeVolume> occupied_leafs;
  visualization_msgs::MarkerArray marker_array;
  
  ros_tree->octree.getOccupied(occupied_leafs);
  
  list<octomap::OcTreeVolume>::const_iterator it = occupied_leafs.begin(), end = occupied_leafs.end();
  
  marker_array.markers.resize(occupied_leafs.size());

  int i = 0;
  for(; it != end; ++it) {
    double cell_size = ros_tree->octree.getNodeSize(i);

    geometry_msgs::Point pt;
    pt.y = it->first.x();
    pt.z = it->first.y();
    pt.x = it->first.z();
    marker_array.markers[i].points.push_back(pt);

    marker_array.markers[i].header.frame_id = "/map";
    marker_array.markers[i].header.stamp = ros::Time::now();
    marker_array.markers[i].ns = "map";
    marker_array.markers[i].id = i;
    marker_array.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    marker_array.markers[i].scale.x = cell_size;
    marker_array.markers[i].scale.y = cell_size;
    marker_array.markers[i].scale.z = cell_size;
    marker_array.markers[i].color = marker_color;

    i++;
  }
}

void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  if (!running)
    return;
  ros::Time start, end;
  start = ros::Time::now();
  ROS_INFO("The time is: %f", start.toSec());
  if (is_first_time)
    if (!first_time_setup())
      return;
  
  tf::StampedTransform transform;
  try {
    listener->lookupTransform("/map", "/openni_rgb_optical_frame", msg->header.stamp, transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
    return;
  }
  ROS_INFO("/openni_rgb_optical_frame -> /map: %f, %f, %f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
  
  sensor_msgs::PointCloud2 transformed_cloud;
  try {
    pcl_ros::transformPointCloud("/map", *msg, transformed_cloud, *listener);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
    return;
  }
  
  octomap::point3d origin(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
  
  ros_tree->insertScan(transformed_cloud, octomap::pointOctomapToMsg(origin));
  end = ros::Time::now();
  
  // publishOccupiedGridCells();
  // publishOccupiedPointCloud();
  publishMarkers();
  
  ROS_INFO("Pointcloud2 processing took: %f seconds", (end-start).toSec());
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "teleop_server");
  
  ros::NodeHandle n;
  
  listener = new tf::TransformListener();
  
  ros::Subscriber sub = n.subscribe("/camera/rgb/points", 1, pointcloudCallback);

  marker_color.r = 1.0;
  marker_color.g = 1.0;
  marker_color.b = 1.0;
  marker_color.a = 1.0;


  
  running = true;
  
  ros::spin();
  
  running = false;
  
  ROS_INFO("Writing binary map file.");
  ros_tree->octree.writeBinary("/tmp/map.bt");
  
  return 0;
}
