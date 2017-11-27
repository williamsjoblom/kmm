#include "kmm_navigation/PathFollower.hpp"

namespace kmm_navigation {

  PathFollower::PathFollower(){

  }


void PathFollower::get_velocity(
    const std::vector<Eigen::Vector2f>& path,
    const Eigen::Vector2f& robot_position,
    Eigen::Vector2f& vel,
    bool& has_reached_target
  )
  {
    double offset_distance = std::numeric_limits<double>::infinity();
    Eigen::Vector2f offset_vector(0,0);
    Eigen::Vector2f curr_offset_vector;
    Eigen::Vector2f path_vector;
    Eigen::Vector2f robot_vector;
    Eigen::Vector2f forward_vector(0, 0);
    double proj_factor;
    for (int i = 0 ; i < path.size(); i++){
      robot_vector = robot_position - path[i];
      if (i != path.size() - 1) {
        path_vector = path[i+1] - path[i];
        proj_factor = robot_vector.dot(path_vector)/path_vector.squaredNorm();
        if (proj_factor < 1 && proj_factor >= 0) {
          curr_offset_vector = proj_factor*path_vector - robot_vector;
          if (curr_offset_vector.norm() < offset_distance){
            offset_distance = curr_offset_vector.norm();
            offset_vector = curr_offset_vector;
            forward_vector = path_vector.normalized();
          }
        }
      }
      if (robot_vector.norm() < offset_distance) {
          offset_distance = robot_vector.norm();
          offset_vector = robot_vector * -1;
      }
    }

    has_reached_target = false;
    if (path.size() > 0) {
      Eigen::Vector2f destination = path[path.size() - 1];
      float distance_to_destination = (destination - robot_position).norm();
      if (distance_to_destination < 0.05) {
        has_reached_target = true;
      }
    }

    if (has_reached_target) {
      vel[0] = 0;
      vel[1] = 0;
    } else {
      vel = offset_vector * 3. + forward_vector * 0.15;
    }
  }
}
