#include "ros/ros.h"
#include <Eigen/Dense>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include "kmm_position/Pose.h"
#include "kmm_position/ils.h"

void publish_points(ros::Publisher& pub, std::vector<Eigen::Vector2f>& points, float green, float red) {
  visualization_msgs::Marker markers;
  markers.type = visualization_msgs::Marker::POINTS;
  markers.header.frame_id = "global";
  markers.scale.x = 0.1;
  markers.scale.y = 0.1;
  markers.color.g = green;
  markers.color.r = red;
  markers.color.a = 1.0;

  for (Eigen::Vector2f point : points) {
    geometry_msgs::Point p;
    p.x = point[0];
    p.y = point[1];
    markers.points.push_back(p);
  }

  pub.publish(markers);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "test_ils_node");
  ros::NodeHandle nh;

  ros::Publisher pub_a = nh.advertise<visualization_msgs::Marker>("test_a", 1);
  ros::Publisher pub_b = nh.advertise<visualization_msgs::Marker>("test_b", 1);

  ros::Rate rate(1);

  while (ros::ok()) {

    std::vector<Eigen::Vector2f> a;
    a.push_back(Eigen::Vector2f(-1, 1));
    a.push_back(Eigen::Vector2f(1, 1));
    a.push_back(Eigen::Vector2f(1, -1));
    a.push_back(Eigen::Vector2f(-1, -1));

    std::vector<Eigen::Vector2f> b;
    b.push_back(Eigen::Vector2f(-0.9, 1));
    b.push_back(Eigen::Vector2f(1, 1.02));
    b.push_back(Eigen::Vector2f(1.05, -0.9));
    b.push_back(Eigen::Vector2f(-1.2, -1.05));

    Pose offset;
    offset.pos[0] = 0.4;
    offset.pos[1] = 0.5;
    offset.angle = 3.14 / 8;
    ROS_INFO_STREAM("offset: " << offset);

    offset.transform(&b);

    publish_points(pub_a, a, 1, 0);
    publish_points(pub_b, b, 0, 1);
    rate.sleep();

    Pose result = least_squares(a, b);
    ROS_INFO_STREAM("result: " << result);

    result.invert();
    result.transform(&b);

    publish_points(pub_b, b, 0, 1);
    rate.sleep();
  }
}
