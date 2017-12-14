#include "ros/ros.h"
#include "kmm_position/Position.hpp"

/*
 * The kmm_position_node is responsible for calculating the position of the robot.
 * This is done by using a Kalman Filter and ILS.
 */

int main(int argc, char **argv) {
  ros::init(argc, argv, "kmm_position");
  ros::NodeHandle nh;
  kmm_position::Position p(nh);
  ros::spin();
}
