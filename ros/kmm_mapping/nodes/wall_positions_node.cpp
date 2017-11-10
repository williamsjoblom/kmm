#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "kmm_mapping/wall_positions.h"

#include <vector>

struct Point {
    float x;
    float y;
};

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

  // creating the vector of horizontal walls
  Point horizontal_walls_array[10];
  Point point_horizontal;
  for (int i=0; i < 10; i++) {
    point_horizontal.x = i;
    point_horizontal.y = i;
    horizontal_walls_array[i] = point_horizontal;
  }
  std::vector<Point> horizontal_walls (horizontal_walls_array,
    horizontal_walls_array + sizeof(horizontal_walls_array) / sizeof(Point));

  // creating the vector of vertical walls
  Point vertical_walls_array[10];
  Point point_vertical;
  for (int i=0; i < 10; i++) {
    point_vertical.x = i;
    point_vertical.y = i;
    vertical_walls_array[i] = point_vertical;
  }
  std::vector<Point> vertical_walls (vertical_walls_array,
    vertical_walls_array + sizeof(vertical_walls_array) / sizeof(Point));


  // loop control
  int count = 0;
  while (ros::ok())
  {
    wall_positions_msg.horizontal_walls.clear();
    wall_positions_msg.vertical_walls.clear();
    wall_positions_msg.cnt = count;

    int i = 0;
    for (std::vector<Point>::iterator it = horizontal_walls.begin();
      it != horizontal_walls.end(); ++it) {
        geometry_msgs::Point point_horizontal;
        point_horizontal.x = (*it).x;
        point_horizontal.y = (*it).y;
        point_horizontal.z = 0;
        wall_positions_msg.horizontal_walls.push_back(point_horizontal);
        i++;
    }

    int j = 0;
    for (std::vector<Point>::iterator it = vertical_walls.begin();
      it != vertical_walls.end(); ++it) {
        geometry_msgs::Point point_vertical;
        point_vertical.x = (*it).x;
        point_vertical.y = (*it).y;
        point_vertical.z = 0;
        wall_positions_msg.vertical_walls.push_back(point_vertical);
        j++;
    }

    ROS_INFO("%d", wall_positions_msg.cnt);

    pub.publish(wall_positions_msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
