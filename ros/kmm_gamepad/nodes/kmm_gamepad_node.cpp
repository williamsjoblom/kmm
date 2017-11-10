#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <SFML/Window.hpp>
#include <iostream>

int main(int argc, char **argv) {

  ros::init(argc, argv, "kmm_gamepad");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("gamepad", 1);

  ros::Rate rate(30);

  const int DEVICE_ID = 0;

  while (ros::ok()) {

    sf::Joystick::update();

    geometry_msgs::Twist msg;
    msg.linear.x = sf::Joystick::getAxisPosition(DEVICE_ID, sf::Joystick::Y) / 100.0;
    msg.linear.y = sf::Joystick::getAxisPosition(DEVICE_ID, sf::Joystick::X) / -100.0;
    msg.angular.z = sf::Joystick::getAxisPosition(DEVICE_ID, sf::Joystick::U) / -100.0;
    pub.publish(msg);

    ros::spinOnce();
    rate.sleep();
  }
}
