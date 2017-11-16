#include "kmm_position/Position.hpp"
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "kmm_position/ils.h"

namespace kmm_position {

  Position::Position(ros::NodeHandle nh)
  : nh_(nh),
    lidar_measurement_(0.2, 0.2, 0)
  {
    // Publishers
    aligned_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud>("aligned_scan", 1);
    position_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("position", 1);
    broadcast_timer_ = nh_.createTimer(ros::Duration(0.05), &Position::broadcast_position, this);

    // Subscribers
    laser_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, "scan", 10);
    laser_notifier_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_sub_, tf_listener_, "map", 10);
    laser_notifier_->registerCallback(boost::bind(&Position::laser_scan_callback, this, _1));
    laser_notifier_->setTolerance(ros::Duration(0.1));
    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &Position::cmd_vel_callback, this);
  }

  Position::~Position() {
    delete laser_sub_;
    delete laser_notifier_;
  }

  void Position::cmd_vel_callback(geometry_msgs::Twist msg) {
    //ROS_INFO("%f", msg.linear.x);
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
    for (int i = 0; i < cloud.points.size(); i++) {
      float x = cloud.points[i].x;
      float y = cloud.points[i].y;
      scan[i] = Eigen::Vector2f(x, y);
    }

    std::vector<Eigen::Vector2f> aligned;
    Pose result = get_transform_pose(scan, aligned, 5);
    lidar_measurement_.accumulate(result);
    publish_aligned_scan(aligned);
  }

  void Position::publish_aligned_scan(std::vector<Eigen::Vector2f>& aligned) {
    sensor_msgs::PointCloud cloud;
    cloud.header.frame_id = "map";

    for (Eigen::Vector2f point : aligned) {
      geometry_msgs::Point32 p;
      p.x = point[0];
      p.y = point[1];
      cloud.points.push_back(p);
    }

    aligned_scan_pub_.publish(cloud);
  }

  void Position::broadcast_position(const ros::TimerEvent&) {
    ROS_INFO("angle %f", lidar_measurement_.angle);
    tf::Vector3 position(lidar_measurement_.pos[0], lidar_measurement_.pos[1], 0);
    tf::Quaternion orientation;
    orientation.setEuler(0, 0, lidar_measurement_.angle);
    tf_broadcaster_.sendTransform(
      tf::StampedTransform(
        tf::Transform(orientation, position),
        ros::Time::now(), "map", "base_link"));

    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.frame_id = "map";
    msg.pose.pose.position.x = lidar_measurement_.pos[0];
    msg.pose.pose.position.y= lidar_measurement_.pos[1];
    msg.pose.pose.orientation.z = lidar_measurement_.angle;
    position_pub_.publish(msg);
  }

}
