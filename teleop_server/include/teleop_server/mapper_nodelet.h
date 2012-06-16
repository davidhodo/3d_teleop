#include <nodelet/nodelet.h>

namespace teleop_server
{
    class MapperNodelet : public nodelet::Nodelet
    {
        public:
            virtual void onInit();

            void onInputPC2(const sensor_msgs::PointCloud2::ConstPtr& msg);
        private:
            ros::NodeHandle nh_;
            ros::Publisher str_pub_;
            ros::Subscriber pc2_sub_;
    };
}