#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include "kmm_mapping/wall_positions.h"
#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <vector>
#include <math.h>
#include <algorithm>

namespace kmm_mapping{

  Class Occupancy {
  public:
    Occupancy(ros::NodeHandle nh);

    //Used to update position
    position_callback();
    //Gather scan data and calculate un/occupied cells and publish
    scan_callback();
  private:
    ros::NodeHandle nh_;
    //Robots position
    float x;
    float y;
  }
}
