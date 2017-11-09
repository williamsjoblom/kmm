/*
ITERATE LEAST SQUARES
*/

#pragma once

#include <Eigen/Dense>
#include <vector>
#include "Pose.h"

Pose get_transform_pose(std::vector<Eigen::vector2f> &scan, int iterations = 1);

Pose least_squares(std::vector<Eigen::Vector3f> &a, std::vector<Eigen::Vector3f> &b);
