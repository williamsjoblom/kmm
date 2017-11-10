#include "ros/ros.h"
#include <Eigen/Dense>
#include <sensor_msgs/LaserScan.h>
#include "kmm_position/Pose.h"
#include "kmm_position/ils.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "test_ils_node");
  ros::NodeHandle nh;

  ros::Publisher pub_a = nh.advertise<sensor_msgs::LaserScan>("test_a", 1);
  ros::Publisher pub_b = nh.advertise<sensor_msgs::LaserScan>("test_b", 1);

  ros::Rate rate(1);

  while (ros::ok()) {

    std::vector<Eigen::Vector2f> a;
    a.push_back(Eigen::Vector2f(-1, 1));
    a.push_back(Eigen::Vector2f(1, 1));
    a.push_back(Eigen::Vector2f(1, -1));
    a.push_back(Eigen::Vector2f(-1, -1));

    std::vector<Eigen::Vector2f> b;
    b.push_back(Eigen::Vector2f(-1, 1));
    b.push_back(Eigen::Vector2f(1, 1));
    b.push_back(Eigen::Vector2f(1, -1));
    b.push_back(Eigen::Vector2f(-1, -1));

    Pose offset;
    offset.pos[0] = 0.3;
    offset.pos[1] = 0.2;
    offset.angle = 3.14 / 8;
    std::cout << "offset: " << offset << std::endl;

    offset.transform(&b);

    //pub_a.publish(a);
    //pub_b.publish(b);
    rate.sleep();

    Pose result = least_squares(a, b);
    std::cout << "result: " << result << std::endl;

    result.transform(&b);

    //pub_b.publish(b);
    rate.sleep();
  }
}
