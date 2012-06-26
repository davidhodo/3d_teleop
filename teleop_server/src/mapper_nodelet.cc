#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>

#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include "teleop_server/mapper_nodelet.h"

// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(teleop_server,
                        MapperNodelet,
                        teleop_server::MapperNodelet,
                        nodelet::Nodelet)

namespace teleop_server {

void MapperNodelet::onInit() {
  NODELET_INFO("Initializing nodelet...");
  // Get the node handle
  this->nh_ = this->getNodeHandle();

  // Get the world frame id
  this->nh_.param<string>("world_frame_id", this->world_frame_,
                          "/map");
  this->nh_.param<double>("z_min", this->z_min_, 0.2);
  this->nh_.param<double>("z_max", this->z_max_, 2.25);
  this->nh_.param<double>("resolution", this->resolution_, 0.025);

  // Setup the octree
  this->octree_ = new octomap::ColorOcTree(this->resolution_);
  this->tree_depth_ = this->octree_->getTreeDepth();

  // Setup ROS communications
  this->pc2_sub_ =
    this->nh_.subscribe<PCLPointCloud>("input", 1,
                                       &MapperNodelet::onInputPC2, this);
  this->map_pub_ = this->nh_.advertise<
    visualization_msgs::MarkerArray>("visualization_markers", 10);
  // this->map_bin_pub_ = this->nh_.advertise<
    // teleop_server::OctreeBinWithPoseAndColor>("octree", 10);
}

void MapperNodelet::onInputPC2(const PCLPointCloud::ConstPtr& cloud) {
  // Get the latest tf and transform to the "world" frame
  tf::StampedTransform sensor_to_world_tf;
  try {
    this->listener_.lookupTransform(this->world_frame_,
                                    cloud->header.frame_id,
                                    cloud->header.stamp,
                                    sensor_to_world_tf);
  } catch(tf::TransformException& ex){
    ROS_ERROR_STREAM("Transform error of sensor data: "
                  << ex.what() << ", quitting callback");
    return;
  }

  // Convert the tf transform to eigen
  Eigen::Matrix4f sensor_to_world;
  pcl_ros::transformAsMatrix(sensor_to_world_tf, sensor_to_world);

  // Use that eigen transform to transform the pc into the world frame
  PCLPointCloud pc;
  pcl::transformPointCloud(*cloud, pc, sensor_to_world);

  // Filter by the world's z axis
  pcl::PassThrough<PCLPoint> pass;
  pass.setFilterFieldName("z");
  pass.setFilterLimits(this->z_min_, this->z_max_);
  pass.setInputCloud(pc.makeShared());
  pass.filter(pc);

  // Downsample to reduce the number of raytraces
  pcl::VoxelGrid<PCLPoint> downsample;
  downsample.setInputCloud(pc.makeShared());
  double ds_resolution = 0.025f;
  downsample.setLeafSize(ds_resolution, ds_resolution, ds_resolution);
  downsample.filter(pc);

  insertScan(sensor_to_world_tf.getOrigin(), pc);

  publishMap();
}

void MapperNodelet::insertScan(const tf::Point& sensor_origin_tf,
                               const PCLPointCloud& pc)
{
  // Convert the sensor origin to a PCL point
  octomap::point3d origin = octomap::pointTfToOctomap(sensor_origin_tf);
  // pcl::PointXYZ sensor_origin(p.x(), p.y(), p.z());

  // For each point in the point cloud
  octomap::KeySet free_cells, occupied_cells;
  octomap::KeyRay keyray;
  for (size_t i = 0; i < pc.points.size(); ++i) {
    // Get the point in octomap type
    octomap::point3d p = octomap::pointPCLToOctomap(pc.points[i]);
    // Calculate the free cells
    if (this->octree_->computeRayKeys(origin, p, keyray)){
      free_cells.insert(keyray.begin(), keyray.end());
    }
    // Calculate the occupied cell
    octomap::OcTreeKey key;
    if (this->octree_->genKey(p, key)) {
      occupied_cells.insert(key);
    }
  }
  // Update the tree
  for (octomap::KeySet::iterator it = free_cells.begin();
       it != free_cells.end(); ++it)
  {
    this->octree_->updateNode(*it, false, false);
  }
  for (octomap::KeySet::iterator it = occupied_cells.begin();
       it != occupied_cells.end(); ++it)
  {
    this->octree_->updateNode(*it, true, false);
  }
  // this->octree_->prune();
}

void MapperNodelet::publishMap() {
  if (this->map_pub_.getNumSubscribers() > 0) {
    this->publishMapAsMarkers();
  }
}

void MapperNodelet::publishMapAsMarkers() {
  visualization_msgs::MarkerArray msg;
  msg.markers.resize(tree_depth_+1);

  typedef octomap::ColorOcTree::iterator it_t;
  it_t it = this->octree_->begin();
  it_t end = this->octree_->end();
  // For leaf in leaves
  for (; it != end; ++it) {
    // If occupied
    if (this->octree_->isNodeOccupied(*it)) {
      // Get some info about the leaf
      double x = it.getX();
      double y = it.getY();
      double z = it.getZ();
      size_t depth = it.getDepth();
      // Insert a point for the leaf's cube
      geometry_msgs::Point leaf_origin;
      leaf_origin.x = x;
      leaf_origin.y = y;
      leaf_origin.z = z;
      msg.markers[depth].points.push_back(leaf_origin);
      // Determine and set the leaf's color by height
      msg.markers[depth].colors.push_back(getColorByHeight(leaf_origin.z));
    }
  }
  // Finish the marker array setup
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  for (size_t i = 0; i < msg.markers.size(); ++i) {
    double size = this->octree_->getNodeSize(i);

    msg.markers[i].header.frame_id = "/map";
    msg.markers[i].header.stamp = ros::Time::now();
    msg.markers[i].ns = "map";
    msg.markers[i].id = i;
    msg.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    msg.markers[i].scale.x = size;
    msg.markers[i].scale.y = size;
    msg.markers[i].scale.z = size;
    msg.markers[i].action = visualization_msgs::Marker::ADD;
    msg.markers[i].color = color;
  }
  // Publish the marker array
  this->map_pub_.publish(msg);
}

// This function comes from the octomap_server pkg
std_msgs::ColorRGBA MapperNodelet::getColorByHeight(double h) {
  double range = this->z_max_ - this->z_min_;
  h = 1.0 - std::min(std::max(h/range, 0.0), 1.0);
  h *= 0.8;
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  // blend over HSV-values (more colors)

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1))
    f = 1 - f; // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:
      color.r = v; color.g = n; color.b = m;
      break;
    case 1:
      color.r = n; color.g = v; color.b = m;
      break;
    case 2:
      color.r = m; color.g = v; color.b = n;
      break;
    case 3:
      color.r = m; color.g = n; color.b = v;
      break;
    case 4:
      color.r = n; color.g = m; color.b = v;
      break;
    case 5:
      color.r = v; color.g = m; color.b = n;
      break;
    default:
      color.r = 1; color.g = 0.5; color.b = 0.5;
      break;
  }

  return color;
}

}
