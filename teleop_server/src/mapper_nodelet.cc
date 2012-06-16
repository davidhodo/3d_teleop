#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include "teleop_server/mapper_nodelet.h"

// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(teleop_server, MapperNodelet, teleop_server::MapperNodelet, nodelet::Nodelet) 

namespace teleop_server
{
    void MapperNodelet::onInit() 
    {
        NODELET_INFO("Initializing nodelet...");
        this->nh_ = this->getNodeHandle();
        this->str_pub_ = this->nh_.advertise<std_msgs::String>("chatter", 5);
        this->pc2_sub_ = this->nh_.subscribe("input", 1, &MapperNodelet::onInputPC2, this);
    }

    void MapperNodelet::onInputPC2(const sensor_msgs::PointCloud2::ConstPtr& pc2) {
        std_msgs::String msg;
        msg.data = "Got a point cloud 2!";
        this->str_pub_.publish(msg);
    }
}
