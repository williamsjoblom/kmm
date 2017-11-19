#include "kmm_mapping/Occupancy.hpp"
#include "geometry_msgs/Point.h"

namespace kmm_mapping {

  Occupancy::Occupancy(ros::NodeHandle nh)
  : nh_(nh)
  {
    // Publishers
    occupancy_pub_ = nh_.advertise<nav_msgs::OccupanyGrid>("occupied_cells", 1);
    // Subscribers
    scan_sub_ = nh_.subscribe("aligned_scan", 1, &Occupancy::scan_callback, this);
    position_sub_ = nh_.subscribe("position", 1, &Occupancy::position_callback, this);
  }

  Occupancy::scan_callback(){

  }

  Occupancy::position_callback(){

  }
}
