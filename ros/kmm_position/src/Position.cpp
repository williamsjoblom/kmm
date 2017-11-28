#include "kmm_position/Position.hpp"
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "kmm_position/ils.h"

namespace kmm_position {

  Position::Position(ros::NodeHandle nh)
  : nh_(nh),
    kalman_(0.2, 0.2, 0)
  {
    // Publishers
    aligned_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud>("aligned_scan", 1);
    position_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("position", 1);
    broadcast_robot_pose_timer_ = nh_.createTimer(ros::Duration(1. / 50), &Position::broadcast_robot_pose, this);
    publish_robot_pose_timer_ = nh_.createTimer(ros::Duration(1. / 20), &Position::publish_robot_pose, this);
    scan_point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("scan_point_cloud", 1);

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
    Eigen::Vector3f state = kalman_.get_state();
    Eigen::Vector3f u(msg.linear.x, msg.linear.y, msg.angular.z * 0.85); // TODO:: Compensate elsewhere
    Eigen::Transform<float, 3, Eigen::Affine> t(Eigen::AngleAxis<float>(state[2], Eigen::Vector3f(0, 0, 1)));
    u = t * u; // Rotate into global frame.
    kalman_.predict(u);
  }

  void Position::laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // Convert scan in polar coordinates robot frame (base_link)
    // to cartesian coordinates global frame (map).
    sensor_msgs::PointCloud cloud;
    try {
      projector_.transformLaserScanToPointCloud("map", *msg, cloud, tf_listener_);
      publish_scan_cloud(cloud);
    }
    catch (tf::TransformException ex) {
       ROS_WARN("%s", ex.what());
       return;
     }

    Eigen::Vector3f state = kalman_.get_state();
    Eigen::Vector2f pos(state[0], state[1]);

    // Create list of Eigen vectors.
    std::vector<Eigen::Vector2f> scan;
    for (int i = 0; i < cloud.points.size(); i++) {
      float x = cloud.points[i].x;
      float y = cloud.points[i].y;
      Eigen::Vector2f p(x, y);
      // Only use data points that are within a certain proximity.
      float proximity = 6;
      if ((p - pos).norm() < proximity) {
        scan.push_back(p);
      }
    }

    std::vector<Eigen::Vector2f> aligned;
    Pose result = get_transform_pose(scan, aligned, 5);
    state[0] = result.pos[0];
    state[1] = result.pos[1];
    state[2] = result.angle;
    kalman_.lidar_measurement(state);
    publish_aligned_scan(aligned);
  }

  void Position::publish_scan_cloud(sensor_msgs::PointCloud& cloud){
    scan_point_cloud_pub_.publish(cloud);
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

  void Position::broadcast_robot_pose(const ros::TimerEvent&) {
    // Get robot state (x, y, angle).
    Eigen::Vector3f state = kalman_.get_state();
    tf::Vector3 position(state[0], state[1], 0);
    tf::Quaternion orientation;
    orientation.setEuler(0, 0, state[2]);

    // Broadcast to TF.
    tf_broadcaster_.sendTransform(
      tf::StampedTransform(
        tf::Transform(orientation, position),
        ros::Time::now(), "map", "base_link"));
  }

  void Position::publish_robot_pose(const ros::TimerEvent&) {
    // Get robot state (x, y, angle).
    Eigen::Vector3f state = kalman_.get_state();
    Eigen::Matrix3f state_cov = kalman_.get_state_cov();
    tf::Vector3 position(state[0], state[1], 0);
    tf::Quaternion orientation;
    orientation.setEuler(0, 0, state[2]);

    // Create message and publish.
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.frame_id = "map";
    msg.pose.pose.position.x = state[0];
    msg.pose.pose.position.y= state[1];
    msg.pose.pose.orientation.x = orientation.x();
    msg.pose.pose.orientation.y = orientation.y();
    msg.pose.pose.orientation.z = orientation.z();
    msg.pose.pose.orientation.w = orientation.w();
    msg.pose.covariance[0] = state_cov(0,0);
    msg.pose.covariance[1] = state_cov(0,1);
    msg.pose.covariance[5] = state_cov(0,2);
    msg.pose.covariance[6] = state_cov(1,0);
    msg.pose.covariance[7] = state_cov(1,1);
    msg.pose.covariance[11] = state_cov(1,2);
    msg.pose.covariance[30] = state_cov(2,0);
    msg.pose.covariance[31] = state_cov(2,1);
    msg.pose.covariance[35] = state_cov(2,2);
    position_pub_.publish(msg);
  }
}
