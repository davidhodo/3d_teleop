#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include "teleop_server/mapper_nodelet.h"

// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(teleop_server, MapperNodelet, teleop_server::MapperNodelet, nodelet::Nodelet) 

namespace teleop_server {

void MapperNodelet::onInit() {
  NODELET_INFO("Initializing nodelet...");
  // Get the node handle
  this->nh_ = this->getNodeHandle();

  // Get the world frame id
  this->nh_.param<string>("world_frame_id", this->world_frame_, "/base_footprint");
  this->nh_.param<double>("z_min", this->z_min_, 0.2);
  this->nh_.param<double>("z_max", this->z_max_, 3.0);
  this->nh_.param<double>("resolution", this->resolution_, 0.05);

  // Setup the octree
  this->ros_octree_ = new octomap::OcTreeROS(this->resolution_);

  // Setup ROS communications
  this->pc2_sub_ = this->nh_.subscribe<PCLPointCloud>("input", 1, &MapperNodelet::onInputPC2, this);
}

void MapperNodelet::onInputPC2(const PCLPointCloud::ConstPtr& cloud) {
  // Get the latest tf and transform to the "world" frame
  tf::StampedTransform sensorToWorldTf;
  try {
    this->listener_.lookupTransform(this->world_frame_, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
  } catch(tf::TransformException& ex){
    ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
    return;
  }

  // Convert the tf transform to eigen
  Eigen::Matrix4f sensorToWorld;
  pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

  // Use that eigen transform to transform the pc into the world frame
  PCLPointCloud pc;
  pcl::transformPointCloud(*cloud, pc, sensorToWorld);

  // Filter by the world's z axis
  pcl::PassThrough<PCLPoint> pass;
  pass.setFilterFieldName("z");
  pass.setFilterLimits(this->z_min_, this->z_max_);
  pass.setInputCloud(pc.makeShared());
  pass.filter(pc);

  insertScan(sensorToWorldTf.getOrigin(), pc);

  // publishAll(cloud->header.stamp);
}

void MapperNodelet::insertScan(const tf::Point& sensorOriginTf, const PCLPointCloud& pc) {
  // Convert the sensor origin to a PCL point
  octomap::point3d sensorOrigin = octomap::pointTfToOctomap(sensorOriginTf);
  pcl::PointXYZ p(sensorOrigin.x(), sensorOrigin.y(), sensorOrigin.z());

  // Insert into the octomap octree
  this->ros_octree_->insertScan(pc, p);
}

}
