#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "kmm_mapping/wall_positions.h"
#include <iostream>
#include <random>
#include <vector>

using namespace std;

struct Point {
    float x;
    float y;
};

// Returns random values uniformly distributed in the range [a, b]
int random(int a, int b) {
  thread_local std::mt19937 eng{std::random_device{}()};
  std::uniform_int_distribution<int> dist(a, b);
  return dist(eng);
}

int main(int argc, char **argv)
{
  // ROS objects
  ros::init(argc, argv, "wall_positions_node");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<kmm_mapping::wall_positions>("wall_positions", 1);
  ros::Rate loop_rate(10);

  // the message to be published
  kmm_mapping::wall_positions wall_positions_msg;
  wall_positions_msg.cnt = 0;

  // loop control
  int count = 0;
  while (ros::ok())
  {
    wall_positions_msg.horizontal_walls.clear();
    wall_positions_msg.vertical_walls.clear();
    wall_positions_msg.cnt = count;

    for (int i = 0; i < 25; i++) {
        geometry_msgs::Point point_horizontal;
        point_horizontal.x = random(0,15);
        point_horizontal.y = random(-15,15); // can never be 0
        while (point_horizontal.y == 0) { // make sure y != 0
          point_horizontal.y = random(-15,15);
        }
        point_horizontal.z = 0;
        wall_positions_msg.horizontal_walls.push_back(point_horizontal);
    }

    for (int i = 0; i < 17; i++) {
        geometry_msgs::Point point_vertical;
        point_vertical.x = random(1,15); // can never be 0
        point_vertical.y = random(-15,15);
        point_vertical.z = 0;
        wall_positions_msg.vertical_walls.push_back(point_vertical);
    }

    ROS_INFO("%d", wall_positions_msg.cnt);

    pub.publish(wall_positions_msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
