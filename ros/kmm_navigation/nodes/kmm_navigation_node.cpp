#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>
#include "kmm_navigation/Navigation.hpp"

//The reconfigure callback for navigation velocities
void reconfigureCallback(kmm_navigation::NavigationConfig &config, uint32_t level){
  ROS_INFO("Reconfigure Request: (error velocity: %.2f, forward velocity: %.2f)",
    config.error_vel_, config.forward_vel_);
  set_error_vel(config.error_vel_);
  set_forward_vel(config.forward_vel_);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "kmm_navigation");
  ros::NodeHandle nh;
  kmm_navigation::Navigation n(nh);
  ros::spin();
}
