#include "ros/ros.h"
#include <Eigen/Dense>

int main(int argc, char **argv) {

  ros::init(argc, argv, "test_ils_node");
  ros::NodeHandle nh;

  ros::Publisher pub_a = nh.advertise<sensor_msgs::LaserScan>("test_a", 1);
  ros::Publisher pub_b = nh.advertise<sensor_msgs::LaserScan>("test_b", 1);

  ros::Rate rate(1);

  while (ros::ok()) {

    std::vector<Eigen::Vector2f> a;
    scan.push_back(Eigen::Vector2f(-1, 1));
    scan.push_back(Eigen::Vector2f(1, 1));
    scan.push_back(Eigen::Vector2f(1, -1));
    scan.push_back(Eigen::Vector2f(-1, -1));

    std::vector<Eigen::Vector2f> b;
    scan.push_back(Eigen::Vector2f(-1, 1));
    scan.push_back(Eigen::Vector2f(1, 1));
    scan.push_back(Eigen::Vector2f(1, -1));
    scan.push_back(Eigen::Vector2f(-1, -1));

    Pose offset;
    offset.pos.x = 0.3;
    offset.pos.y = 0.2;
    offset.angle = 3.14 / 8;
    std::cout << "offset: " << offset << std::endl;

    offset.transform(b);

    pub_a.publish(a);
    pub_b.publish(b);
    rate.sleep();

    Pose result = ils_relative_pose(a, b);
    std::cout << "result: " << result << std::endl;

    result.transform(b);

    pub_b.publish(b);
    rate.sleep();
  }
}
