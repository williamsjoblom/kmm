#pragma once

#include <Eigen/Dense>
#include <vector>
#include <iostream>

class Pose {
public:
    Eigen::Vector3f pos;
    float angle = 0;

    void transform(std::vector<Eigen::Vector3f> *points);
    void accumulate(const Pose &pose);
    static Pose relativePose(std::vector<const Eigen::Vector3f*> &a, std::vector<const Eigen::Vector3f*> &b);
};

std::ostream &operator<<(std::ostream &os, const Pose &p);
