#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "teleop_server");
  ros::NodeHandle n;
  
  int use_sim_time = false;
  if (!n.getParam("/use_sim_time", use_sim_time))
    ROS_ERROR("did not get use_sim_time");
  
  if (use_sim_time)
    ROS_INFO("use_sim_time is True");
  else
    ROS_INFO("use_sim_time is False");
  
  ros::Rate r (5);
  while (ros::ok()) {
    ROS_INFO("The time is: %f", ros::Time::now().toSec());
    if (ros::Time::isSimTime())
        ROS_INFO("The output of isSimTime(): True");
    else
        ROS_INFO("The output of isSimTime(): False");
    ros::spinOnce();
    r.sleep();
  }
  
}
