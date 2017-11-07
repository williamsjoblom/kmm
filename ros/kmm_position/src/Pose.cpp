#include "Pose.hpp"
#include <cmath>
#include <assert.h>

void Pose::transform(std::vector<Eigen::Vector3f> *points)
{
    double c = std::cos(angle);
    double s = std::sin(angle);

    for (std::vector<Vec2>::iterator i = points->begin(); i != points->end(); i++)
    {
        Vec2 &point = *i;
        point.x += pos.x;
        point.y += pos.y;
        double x = c * point.x - s * point.y;
        double y = s * point.x + c * point.y;
        point.x = x;
        point.y = y;
    }
}

void Pose::accumulate(const Pose &pose)
{
    double c = std::cos(pose.angle);
    double s = std::sin(pose.angle);
    pos.x += pose.pos.x;
    pos.y += pose.pos.y;
    double x = c * pos.x - s * pos.y;
    double y = s * pos.x + c * pos.y;
    pos.x = x;
    pos.y = y;
    angle += pose.angle;
    while (angle >= M_PI) angle -= 2.0 * M_PI;
    while (angle <  M_PI) angle += 2.0 * M_PI;
}

std::ostream &operator<<(std::ostream &os, const Pose &pose)
{
	os  << "("
        << pose.pos.x << ","
        << pose.pos.y << ","
        << pose.angle << ")";
	return os;
}
