#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <SFML/Window.hpp>
#include <iostream>

float deadzone(float value) {
  const float DEADZONE = 0.05;
  if (value > -DEADZONE && value < DEADZONE) {
    return 0;
  } else {
    return value;
  }
}

float lowpass(float x, float y0, float dt, float T) {
  return y0 + (x - y0) * (dt/(dt+T));
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "kmm_gamepad");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("gamepad_vel", 1);

  ros::Rate rate(30);

  float T = 0.2;
  ros::Time ts = ros::Time::now();
  float l_x = 0;
  float l_y = 0;
  float l_z = 0;

  const int DEVICE_ID = 0;

  while (ros::ok()) {

    sf::Joystick::update();

    float x = deadzone(sf::Joystick::getAxisPosition(DEVICE_ID, sf::Joystick::Y) / 100.0);
    float y = deadzone(sf::Joystick::getAxisPosition(DEVICE_ID, sf::Joystick::X) / -100.0);
    float z = deadzone(sf::Joystick::getAxisPosition(DEVICE_ID, sf::Joystick::U) / -100.0);

    double dt = (ros::Time::now() - ts).toSec();
    ts = ros::Time::now();
    l_x = lowpass(x, l_x, dt, T);
    l_y = lowpass(y, l_y, dt, T);
    l_z = lowpass(z, l_z, dt, T);

    const float MAX_LINEAR = 0.25; // m/s
    const float MAX_ANGLUAR = 1.0; // rad/s

    geometry_msgs::Twist msg;
    msg.linear.x = l_x * MAX_LINEAR;
    msg.linear.y = l_y * MAX_LINEAR;
    msg.angular.z = l_z * MAX_ANGLUAR;
    pub.publish(msg);

    ros::spinOnce();
    rate.sleep();
  }
}
