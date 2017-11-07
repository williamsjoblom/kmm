#pragma once

#include <Eigen/Dense>
#include <vector>

ils_fit(std::vector<Eigen::Vector3f> &a, std::vector<Eigen::Vector3f> &b);

ils_relative_pose(std::vector<Eigen::Vector3f> &a, std::vector<Eigen::Vector3f> &b);
