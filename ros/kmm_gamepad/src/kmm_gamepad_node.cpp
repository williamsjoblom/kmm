#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <SFML/Window.hpp>
#include <iostream>

int main(int argc, char **argv) {

  ros::init(argc, argv, "kmm_gamepad");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<std_msgs::Float64>("joy_x", 1);

  ros::Rate rate(10);

  const int DEVICE_ID = 0;

  while (ros::ok()) {
    sf::Joystick::update();

    float x = sf::Joystick::getAxisPosition(DEVICE_ID, sf::Joystick::X);
    float y = sf::Joystick::getAxisPosition(DEVICE_ID, sf::Joystick::Y);
    float u = sf::Joystick::getAxisPosition(DEVICE_ID, sf::Joystick::U);

    ROS_INFO("x: %.2f, y: %.2f, u: %.2f", x, y, u);

    std_msgs::Float64 msg;
    msg.data = x;
    pub.publish(msg);

    ros::spinOnce();

    rate.sleep();

    float R = 0.046;
    float L = 0.103; // [m]
    float sigma = 0;
    float a1 = (1.0 / R) * (-sin(sigma + u)*x + cos(sigma + u)*y + L*u);
    // float a2 = ...
    // float a3 = ...

  }

}
