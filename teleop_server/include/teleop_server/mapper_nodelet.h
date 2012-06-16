#include <string>

#include <octomap_ros/OctomapROS.h>
#include <octomap/OcTreeKey.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <tf/tf.h>

#include <pcl_ros/point_cloud.h>

using std::string;

namespace teleop_server
{
    typedef pcl::PointXYZRGB PCLPoint;
    typedef pcl::PointCloud<PCLPoint> PCLPointCloud;

    class MapperNodelet : public nodelet::Nodelet
    {
        public:
            virtual void onInit();

            void onInputPC2(const PCLPointCloud::ConstPtr& cloud);
            void insertScan(const tf::Point& sensorOriginTf, const PCLPointCloud& pc);
        private:
            string world_frame_;
            double z_min_, z_max_;
            double resolution_;

            tf::TransformListener listener_;

            octomap::OcTreeROS * ros_octree_;

            ros::NodeHandle nh_;
            ros::Publisher str_pub_;
            ros::Subscriber pc2_sub_;
    };
}