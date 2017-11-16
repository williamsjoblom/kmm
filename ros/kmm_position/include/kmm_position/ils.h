/*
ITERATE LEAST SQUARES
*/

#pragma once

#include <Eigen/Dense>
#include <vector>
#include "Pose.h"

Pose get_transform_pose(
  std::vector<Eigen::Vector2f> &scan,
  std::vector<Eigen::Vector2f> &aligned,
  int iterations
);

Pose least_squares(
  std::vector<Eigen::Vector2f> &a,
  std::vector<Eigen::Vector2f> &b
);

void build_pairs(
  const std::vector<Eigen::Vector2f> &scan,
  std::vector<Eigen::Vector2f> &a,
  std::vector<Eigen::Vector2f> &b
);
