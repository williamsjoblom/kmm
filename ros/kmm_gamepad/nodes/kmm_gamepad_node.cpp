#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <SFML/Window.hpp>
#include <iostream>

int main(int argc, char **argv) {

  ros::init(argc, argv, "kmm_gamepad");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("gamepad_vel", 1);

  ros::Rate rate(10);

  const int DEVICE_ID = 0;

  while (ros::ok()) {

    sf::Joystick::update();

    float x = sf::Joystick::getAxisPosition(DEVICE_ID, sf::Joystick::X) / 100.0;
    float y = sf::Joystick::getAxisPosition(DEVICE_ID, sf::Joystick::Y) / 100.0;
    float u = sf::Joystick::getAxisPosition(DEVICE_ID, sf::Joystick::U) / 100.0;

    const float MAX_LINEAR = 0.5; // m/s
    const float MAX_ANGLUAR = 0.1; // rad/s

    geometry_msgs::Twist msg;
    msg.linear.x = -y * MAX_LINEAR;
    msg.linear.y = -x * MAX_LINEAR;
    msg.angular.z = u * MAX_ANGLUAR;
    pub.publish(msg);

    ros::spinOnce();
    rate.sleep();
  }
}
