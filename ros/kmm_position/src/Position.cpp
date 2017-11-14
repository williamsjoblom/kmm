#include "kmm_position/Position.hpp"
#include <visualization_msgs/Marker.h>
#include "kmm_position/ils.h"

namespace kmm_position {

  Position::Position(ros::NodeHandle nh)
  : nh_(nh),
    lidar_messurement_(0.2, 0.2, 0)
  {
    // Publishers
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("pair", 1);
    broadcast_timer_ = nh_.createTimer(ros::Duration(0.01), &Position::broadcast_position, this);

    // Subscribers
    laser_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, "scan", 10);
    laser_notifier_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_sub_, tf_listener_, "map", 10);
    laser_notifier_->registerCallback(boost::bind(&Position::laser_scan_callback, this, _1));
    laser_notifier_->setTolerance(ros::Duration(0.1));
  }

  Position::~Position() {
    delete laser_sub_;
    delete laser_notifier_;
  }

  void Position::laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // Convert scan in polar coordinates robot frame (base_link)
    // to cartesian coordinates global frame (map).
    sensor_msgs::PointCloud cloud;
    try {
      projector_.transformLaserScanToPointCloud("map", *msg, cloud, tf_listener_);
    }
    catch (tf::TransformException ex) {
       ROS_WARN("%s", ex.what());
       return;
     }

    // Create list of Eigen vectors.
    std::vector<Eigen::Vector2f> scan;
    scan.resize(cloud.points.size());
    for (int i = 0; i < scan.size(); i++) {
      scan[i] = Eigen::Vector2f(cloud.points[i].x, cloud.points[i].y);
    }

    std::vector<Eigen::Vector2f> a, b;
    build_pair(scan, a, b);

    publish_points("a", a, 1, 0, 0);
    publish_points("b", b, 0, 1, 0);
  }

  void Position::publish_points(std::string ns, std::vector<Eigen::Vector2f>& points, float r, float g, float b) {
    visualization_msgs::Marker markers;
    markers.type = visualization_msgs::Marker::POINTS;
    markers.header.frame_id = "map";
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

  void Position::broadcast_position(const ros::TimerEvent&) {
    tf::Vector3 position(lidar_messurement_.pos[0], lidar_messurement_.pos[1], 0);
    tf::Quaternion orientation;
    orientation.setEuler(0, 0, lidar_messurement_.angle);
    tf_broadcaster_.sendTransform(
      tf::StampedTransform(
        tf::Transform(orientation, position),
        ros::Time::now(), "map", "base_link"));
  }

}
