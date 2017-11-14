#include "kmm_position/Position.hpp"
#include <visualization_msgs/Marker.h>
#include "kmm_position/ils.h"

namespace kmm_position {

  Position::Position(ros::NodeHandle nh)
  : nh_(nh)
  {
    laser_scan_sub_ = nh_.subscribe("scan", 1, &Position::laser_scan_callback, this);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("pair", 1);
  }

  Position::~Position() {

  }

  void Position::laser_scan_callback(sensor_msgs::LaserScan msg) {
    ROS_INFO("GOT SCAN!");

    std::vector<Eigen::Vector2f> scan;
    for (int i = 0; i < msg.ranges.size(); i++) {
      float range = msg.ranges[i];
      if (range > 0.16) {
        float angle = i * 3.1415 / 180;
        float x = cos(angle) * range + 0.2;
        float y = sin(angle) * range + 0.2;
        scan.push_back(Eigen::Vector2f(x, y));
      }
    }

    std::vector<Eigen::Vector2f> a, b;
    build_pair(scan, a, b);

    publish_points("a", a, 1, 0, 0);
    publish_points("b", b, 0, 1, 0);
  }

  void Position::publish_points(std::string ns, std::vector<Eigen::Vector2f>& points, float r, float g, float b) {
    visualization_msgs::Marker markers;
    markers.type = visualization_msgs::Marker::POINTS;
    markers.header.frame_id = "neato_laser";
    markers.ns = ns;
    markers.scale.x = 0.01;
    markers.scale.y = 0.01;
    markers.color.r = r;
    markers.color.g = g;
    markers.color.b = b;
    markers.color.a = 1.0;

    for (Eigen::Vector2f point : points) {
      geometry_msgs::Point p;
      p.x = point[0];
      p.y = point[1];
      markers.points.push_back(p);
    }

    marker_pub_.publish(markers);
  }

}
