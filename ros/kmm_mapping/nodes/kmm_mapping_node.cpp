#include "ros/ros.h"
#include "kmm_mapping/Mapping.hpp"
#include <dynamic_reconfigure/server.h>
#include <kmm_mapping/MappingConfig.h>

/*
 * kmm_mapping_node determines where there are walls and end points and
 * publishes that data to the topics /walls and /end_points.
 * Also responsible for the mapping toggle in the client.
 */

int pnt_cnt_req_;
int times_req_;

void reconfigure_callback(kmm_mapping::MappingConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: (pnt_cnt_req: %d, times_req: %d)",
    config.pnt_cnt_req, config.times_req
  );
  pnt_cnt_req_ = config.pnt_cnt_req;
  times_req_ = config.times_req;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "kmm_mapping");
  ros::NodeHandle nh;
  kmm_mapping::Mapping m(nh);

  // Dynamic reconfigure
  dynamic_reconfigure::Server<kmm_mapping::MappingConfig> server;
  dynamic_reconfigure::Server<kmm_mapping::MappingConfig>::CallbackType f;

  f = boost::bind(&reconfigure_callback, _1, _2);
  server.setCallback(f);

  ros::Rate r(10); // 10 hz
  while (ros::ok())
  {
    m.set_times_req(times_req_);
    m.set_pnt_cnt_req(pnt_cnt_req_);
    ros::spinOnce();
    r.sleep();
  }
}
