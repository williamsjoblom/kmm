#pragma once

#include <Eigen/Dense>
#include <vector>
#include <iostream>

class Pose {
public:
    Pose();
    Pose(float x, float y, float angle);

    Eigen::Vector2f pos;
    float angle = 0;

    void transform(std::vector<Eigen::Vector2f> *points);
    void accumulate(const Pose &pose);
    void invert();
};

std::ostream &operator<<(std::ostream &os, const Pose &p);
