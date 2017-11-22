#include "kmm_navigation/Smooth.hpp"

namespace kmm_navigation {

  Smooth::Smooth(ros::NodeHandle nh)
  : nh_(nh), map_(53) {

    // Fill map here...
    // map_.....

    path_pub_ = nh_.advertise<nav_msgs::Path>("path", 1);
    smooth_pub_ = nh_.advertise<nav_msgs::Path>("smooth", 1);
    timer_ = nh_.createTimer(ros::Duration(1), &Smooth::timer_callback, this);
  }

  Smooth::~Smooth() {

  }

  void Smooth::timer_callback(const ros::TimerEvent&) {



    std::vector<Eigen::Vector2f> not_smooth;
    not_smooth.push_back(Eigen::Vector2f(0.2, 0.2));
    not_smooth.push_back(Eigen::Vector2f(0.2 + 0.4, 0.2));
    not_smooth.push_back(Eigen::Vector2f(0.2 + 0.4, 0.2 + 0.4));

    // Do shit
    std::vector<Eigen::Vector2f> smooth = make_smooth(not_smooth);

    path_pub_.publish(path_to_msg(not_smooth));
    smooth_pub_.publish(path_to_msg(smooth));




  }

  std::vector<Eigen::Vector2f> Smooth::make_smooth(const std::vector<Eigen::Vector2f>& path) {

    if (path.size() < 3) {
      return path;
    }

    int resolution = 10;
    std::vector<Eigen::Vector2f> smooth;

    smooth.push_back(path[0]);
    for (int i = 0; i < path.size() - 2; i++) {
      for (int t = 0; t < resolution; t++) {
        float s = 0.2 * t / resolution;
        Eigen::Vector2f first = path[i+1] + (path[i] - path[i+1]).normalized() * (0.2 - s);
        Eigen::Vector2f second = path[i+1] + (path[i+2] - path[i+1]).normalized() * s;
        Eigen::Vector2f point = first + (second - first) * s / 0.2;
        smooth.push_back(point);
      }
    }
    smooth.push_back(path[path.size()-1]);

    return smooth;

  }

  nav_msgs::Path Smooth::path_to_msg(std::vector<Eigen::Vector2f> path) {
    nav_msgs::Path msg;
    msg.header.frame_id = "map";
    for (const Eigen::Vector2f point : path) {
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.pose.position.x = point.x();
      pose_stamped.pose.position.y = point.y();
      msg.poses.push_back(pose_stamped);
    }
    return msg;
  }



}
