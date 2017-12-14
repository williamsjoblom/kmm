/*
ITERATE LEAST SQUARES
*/

#pragma once

#include <Eigen/Dense>
#include <vector>
#include "Pose.h"

Pose calculate_robot_movement(
  Eigen::Vector2f robot_position,
  std::vector<Eigen::Vector2f> &scan,
  std::vector<Eigen::Vector2f> &position_scan,
  std::vector<Eigen::Vector2f> &mapping_scan,
  int iterations,
  float position_proximity,
  float mapping_proximity,
  float position_ignore,
  float mapping_ignore,
  float cell_size
);

Pose least_squares(
  std::vector<Eigen::Vector2f> &a,
  std::vector<Eigen::Vector2f> &b
);

void build_pairs(
  const std::vector<Eigen::Vector2f> &scan,
  std::vector<Eigen::Vector2f> &a,
  std::vector<Eigen::Vector2f> &b,
  float ignore,
  float cell_size
);
