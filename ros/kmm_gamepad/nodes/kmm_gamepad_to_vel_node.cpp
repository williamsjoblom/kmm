#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <SFML/Window.hpp>
#include <iostream>
#include <dynamic_reconfigure/server.h>
#include <kmm_gamepad/GamepadConfig.h>

ros::Publisher cmd_vel_pub;

float max_linear_vel = 0.0; // m/s
float max_angular_vel = 0.0; // rad/s
float deadzone_threshold = 0.0;

// Lowpass filter
float T = 0.2; // Constant
ros::Time ts;
float l_x = 0;
float l_y = 0;
float l_z = 0;

void reconfigureCallback(kmm_gamepad::GamepadConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: (linear: %.2f, angular: %.2f, deadzone: %.4f)",
    config.max_linear_vel, config.max_angular_vel, config.deadzone_threshold
  );
  max_linear_vel = config.max_linear_vel;
  max_angular_vel = config.max_angular_vel;
  deadzone_threshold = config.deadzone_threshold;
}

float deadzone(float value) {
  if (value > -deadzone_threshold && value < deadzone_threshold) {
    return 0;
  } else {
    return value;
  }
}

float lowpass(float x, float y0, float dt, float T) {
  return y0 + (x - y0) * (dt/(dt+T));
}

void gamepadCallback(geometry_msgs::Twist gamepad_msg) {

  float x = gamepad_msg.linear.x;
  float y = gamepad_msg.linear.y;
  float z = gamepad_msg.angular.z;

  double dt = (ros::Time::now() - ts).toSec();
  ts = ros::Time::now();
  l_x = lowpass(x, l_x, dt, T);
  l_y = lowpass(y, l_y, dt, T);
  l_z = lowpass(z, l_z, dt, T);

  geometry_msgs::Twist cmd_vel_msg;
  cmd_vel_msg.linear.x = l_x * max_linear_vel;
  cmd_vel_msg.linear.y = l_y * max_linear_vel;
  cmd_vel_msg.angular.z = l_z * max_angular_vel;
  cmd_vel_pub.publish(cmd_vel_msg);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "kmm_gamepad");
  ros::NodeHandle nh;

  dynamic_reconfigure::Server<kmm_gamepad::GamepadConfig> server;
  dynamic_reconfigure::Server<kmm_gamepad::GamepadConfig>::CallbackType f;

  f = boost::bind(&reconfigureCallback, _1, _2);
  server.setCallback(f);

  ts = ros::Time::now();

  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Subscriber sub = nh.subscribe("gamepad", 1, &gamepadCallback);

  ros::spin();
}
