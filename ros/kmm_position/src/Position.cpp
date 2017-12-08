#include "kmm_position/Position.hpp"
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "kmm_position/ils.h"

namespace kmm_position {

  Position::Position(ros::NodeHandle nh)
  : nh_(nh),
    kalman_(0.2, 0.2, 0)
  {
    // Dynamic reconfigure
    dynamic_reconfigure::Server<PositionConfig>::CallbackType f;
    f = boost::bind(&Position::reconfigure_callback, this, _1, _2);
    reconfigure_server_.setCallback(f);

    // Publishers
    mapping_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud>("mapping_scan", 1);
    position_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud>("position_scan", 1);
    position_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("position", 1);
    broadcast_robot_pose_timer_ = nh_.createTimer(ros::Duration(1. / 50), &Position::broadcast_robot_pose, this);
    publish_robot_pose_timer_ = nh_.createTimer(ros::Duration(1. / 20), &Position::publish_robot_pose, this);

    // Subscribers
    laser_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, "scan", 10);
    laser_notifier_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_sub_, tf_listener_, "map", 10);
    laser_notifier_->registerCallback(boost::bind(&Position::laser_scan_callback, this, _1));
    laser_notifier_->setTolerance(ros::Duration(0.1));
    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &Position::cmd_vel_callback, this);
    imu_sub_ = nh_.subscribe("imu", 1, &Position::imu_callback, this);

    // Services
    reset_position_service_ = nh_.advertiseService("reset_position", &Position::reset_position, this);
  }

  Position::~Position() {
    delete laser_sub_;
    delete laser_notifier_;
  }

  void Position::reconfigure_callback(PositionConfig& config, int level) {
    config_ = config;
    kalman_.set_predict_noise(config_.predict_noise_linear, config_.predict_noise_angular);
    kalman_.set_lidar_noise(config_.lidar_noise_linear, config_.lidar_noise_angular);
    use_predictions_ = config_.use_predictions;
    use_lidar_ = config_.use_lidar;
    use_accelerometer_ = config_.use_accelerometer;
    use_gyroscope_ = config_.use_gyroscope;
  }

  void Position::cmd_vel_callback(geometry_msgs::Twist msg) {
    if (use_predictions_) {
      Eigen::Vector3f u(msg.linear.x, msg.linear.y, msg.angular.z);
      kalman_.predict(u);
    }
  }

  void Position::imu_callback(sensor_msgs::Imu msg){
    if (use_gyroscope_) {
      // Create matrices for gyro and accelerometer.
      Eigen::Matrix3f gyro_m;

      gyro_m <<
        0, 0, 0,
        0, 0, 0,
        0, 0, config_.gyro_noise;//(float)msg.angular_velocity_covariance[8];

      Eigen::Matrix3f accel_m;

      accel_m <<
        (float)msg.linear_acceleration_covariance[0], (float)msg.linear_acceleration_covariance[1], 0,
        (float)msg.linear_acceleration_covariance[3], (float)msg.linear_acceleration_covariance[4], 0,
        0,                                            0,                                            0;

      kalman_.set_gyro_noise(gyro_m);
      //kalman_.set_accel_noise(accel_m)

      // Create a vector with present measurements from sensor
      Eigen::Vector3f gyro_vector(
        0,
        0,
        msg.angular_velocity.z);

        // Create a vector with present measurements from sensor
      Eigen::Vector3f accel_vector(
        msg.linear_acceleration.x,
        msg.linear_acceleration.y,
        0);

      kalman_.gyro_accel_measurement(gyro_vector, accel_vector);
    }
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
    for (int i = 0; i < cloud.points.size(); i++) {
      float x = cloud.points[i].x;
      float y = cloud.points[i].y;
      scan.push_back(Eigen::Vector2f(x, y));
    }

    std::vector<Eigen::Vector2f> position_scan;
    std::vector<Eigen::Vector2f> mapping_scan;

    Eigen::Vector3f state = kalman_.get_state();
    Eigen::Vector2f robot_position(state[0], state[1]);

    Pose result = calculate_robot_movement(
      robot_position,
      scan,
      position_scan,
      mapping_scan,
      config_.iterations,
      config_.position_proximity,
      config_.mapping_proximity,
      config_.position_ignore,
      config_.mapping_ignore
    );

    Eigen::Vector3f messurement(
      result.pos[0],
      result.pos[1],
      result.angle
    );

    if (use_lidar_) {
      kalman_.lidar_measurement(messurement);
    }

    publish_scan(mapping_scan_pub_, mapping_scan);
    publish_scan(position_scan_pub_, position_scan);
  }

  void Position::publish_scan(ros::Publisher& pub, std::vector<Eigen::Vector2f>& scan) {
    sensor_msgs::PointCloud cloud;
    cloud.header.frame_id = "map";

    for (Eigen::Vector2f point : scan) {
      geometry_msgs::Point32 p;
      p.x = point[0];
      p.y = point[1];
      cloud.points.push_back(p);
    }

    pub.publish(cloud);
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

  /*
   * Callback for service requests to reset position.
   */
  bool Position::reset_position(std_srvs::SetBool::Request &req,
    std_srvs::SetBool::Response &res) {
    kalman_.reset_state();

    return true;
  }
}
