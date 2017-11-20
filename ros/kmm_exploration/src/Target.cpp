#include "kmm_exploration/Target.hpp"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>
#include <float.h>

namespace kmm_exploration{

  Target::Target(ros::NodeHandle nh) : nh_(nh){
    // Publishers
    target_pub_ = nh_.advertise<geometry_msgs::Twist>("target_position", 1);

    // Subscribers
    end_points_sub_ = nh_.subscribe<sensor_msgs::PointCloud>("end_points", 1, &Target::end_points_callback, this);
    position_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("position", 1, &Target::position_callback, this);
  }

  Target::~Target(){}

  void Target::end_points_callback(sensor_msgs::PointCloud msg){
    geometry_msgs::Point32 closest;
    closest.x = 0;
    closest.y = 0;
    float min_distance = FLT_MAX;
    for (geometry_msgs::Point32 point : msg.points){
      float distance = std::sqrt(std::pow(point.x - pos_x_, 2) + std::pow(point.y - pos_y_ , 2));
      if (distance < min_distance){
        closest = point;
        min_distance = distance;
      }
    }
    if (closest.x == 0 && closest.y == 0){
      publish_target(0.2, 0.2);
    }
    else{
      float new_x = ((closest.x - pos_x_) / min_distance) * 0.4;
      float new_y = ((closest.y - pos_y_) / min_distance) * 0.4;
      publish_target(closest.x + new_x, closest.y + new_y);
    }
  }

  void Target::position_callback(geometry_msgs::PoseWithCovarianceStamped msg){
    pos_x_ = msg.pose.pose.position.x;
    pos_y_ = msg.pose.pose.position.y;
    angle_ = msg.pose.pose.orientation.z;
  }

  void Target::publish_target(float x, float y){
    geometry_msgs::Twist msg;
    msg.linear.x = x;
    msg.linear.y = y;
    float angle = 0; //calculate
    msg.angular.z = angle;
    target_pub_.publish(msg);
  }
}
