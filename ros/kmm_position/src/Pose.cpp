#include "kmm_position/Pose.h"
#include <cmath>
#include <assert.h>
#include <ros/ros.h>

Pose::Pose() : pos(0, 0), angle(0) { };

Pose::Pose(float x, float y, float angle) : pos(x, y), angle(angle) { };

/**
  Moves points in point cloud accordingly to pose
  (applies the transform matrix to points).
*/
void Pose::transform(std::vector<Eigen::Vector2f> *points)
{
    double c = std::cos(angle);
    double s = std::sin(angle);

    for (std::vector<Eigen::Vector2f>::iterator i = points->begin(); i != points->end(); i++)
    {
        Eigen::Vector2f &point = *i;
        point[0] += pos[0];
        point[1] += pos[1];
        double x = c * point[0] - s * point[1];
        double y = s * point[0] + c * point[1];
        point[0] = x;
        point[1] = y;
    }
}

/**
  Add pose to this pose.
*/
void Pose::accumulate(const Pose &pose)
{
    double c = std::cos(pose.angle);
    double s = std::sin(pose.angle);
    pos[0] += pose.pos[0];
    pos[1] += pose.pos[1];
    double x = c * pos[0] - s * pos[1];
    double y = s * pos[0] + c * pos[1];
    pos[0] = x;
    pos[1] = y;
    angle += pose.angle;
    while (angle >= M_PI) angle -= 2.0 * M_PI;
    while (angle <  M_PI) angle += 2.0 * M_PI;
}

void Pose::invert() {
  pos[0] *= -1;
  pos[1] *= -1;
  angle *= -1;
  while (angle >= M_PI) angle -= 2.0 * M_PI;
  while (angle <  M_PI) angle += 2.0 * M_PI;
}

/**
  Overloading stream operator.
*/
std::ostream &operator<<(std::ostream &os, const Pose &pose)
{
	os  << "("
        << pose.pos[0] << ","
        << pose.pos[1] << ","
        << pose.angle << ")";
	return os;
}
