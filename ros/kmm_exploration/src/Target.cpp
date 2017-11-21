#include "kmm_exploration/Target.hpp"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>
#include <float.h>

namespace kmm_exploration{

  Target::Target(ros::NodeHandle nh)
  : nh_(nh), navigation_client_("navigation", true)
  {
    // Publishers
    target_pub_ = nh_.advertise<geometry_msgs::Twist>("target_position", 1);

    // Subscribers
    end_points_sub_ = nh_.subscribe("end_points", 1, &Target::end_points_callback, this);
    position_sub_ = nh_.subscribe("position", 1, &Target::position_callback, this);
  }

  Target::~Target(){}

/*
  Callback for end_points. Eventually updates target and publishes and sets goal based on if previous target is explored.
*/
  void Target::end_points_callback(sensor_msgs::PointCloud msg){
    geometry_msgs::Point32 closest;
    bool not_empty = false;
    float min_distance = FLT_MAX;
    for (geometry_msgs::Point32 point : msg.points){
      not_empty = true;
      float distance = std::sqrt(std::pow(point.x - pos_x_, 2) + std::pow(point.y - pos_y_ , 2));
      //If point is equal to the previous, there shouldn't be a new target
      if (point.x == target_.x && point.y == target_.y){
        closest = point;
        break;
      }
      else if (distance < min_distance){
        closest = point;
        min_distance = distance;
      }
    }
    //If list was empty, return to start
    if (!not_empty){
      float new_x = 0.2;
      float new_y = 0.2;
      update_target(new_x, new_y);
    }
    else{
      target_ = closest;
      //float new_x = ((closest.x - pos_x_) / min_distance) * 0.4;
      float new_x = closest.x + (closest.x - pos_x_ > 0 ? 0.2 : - 0.2);
      //float new_y = ((closest.y - pos_y_) / min_distance) * 0.4;
      float new_y = closest.y + (closest.y - pos_y_ > 0 ? 0.2 : - 0.2);
      update_target(new_x, new_y);
    }
  }

/*
  Checks if value is new and in that case publishes and sends new goal.
*/
  void Target::update_target(float new_x, float new_y){
    if (!(new_x == x_ && new_y == y_)){
      x_ = new_x;
      y_ = new_y;
      send_goal();
      publish_target();
    }
  }

/*
  Sends a new travelling goal to action server.
*/
  void Target::send_goal() {
    kmm_navigation::MoveToGoal goal;
    goal.x = x_;
    goal.y = y_;
    goal.angle = 0;
    navigation_client_.sendGoal(goal);
  }

  void Target::position_callback(geometry_msgs::PoseWithCovarianceStamped msg){
    pos_x_ = msg.pose.pose.position.x;
    pos_y_ = msg.pose.pose.position.y;
    angle_ = msg.pose.pose.orientation.z;
  }

  /*
  Publishes the target coordinates and angle.
  */
  void Target::publish_target(){
    geometry_msgs::Twist msg;
    msg.linear.x = x_;
    msg.linear.y = y_;
    float angle = 0; //calculate
    msg.angular.z = angle;
    target_pub_.publish(msg);
  }
}
