#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <SFML/Window.hpp>
#include <iostream>
#include <dynamic_reconfigure/server.h>
#include <kmm_gamepad/GamepadConfig.h>

float max_linear_vel = 0.0; // m/s
float max_angular_vel = 0.0; // rad/s

void callback(kmm_gamepad::GamepadConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: (New max linear velocity: %.2f, \
    New max angular velocity: %.2f)",
    config.max_linear_vel, config.max_angular_vel
  );
  max_linear_vel = config.max_linear_vel;
  max_angular_vel = config.max_angular_vel;
}

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

  dynamic_reconfigure::Server<kmm_gamepad::GamepadConfig> server;
  dynamic_reconfigure::Server<kmm_gamepad::GamepadConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

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

    geometry_msgs::Twist msg;
    msg.linear.x = l_x * max_linear_vel;
    msg.linear.y = l_y * max_linear_vel;
    msg.angular.z = l_z * max_angular_vel;
    pub.publish(msg);

    ros::spinOnce();
    rate.sleep();
  }
}
