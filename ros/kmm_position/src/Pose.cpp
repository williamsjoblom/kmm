#include "Pose.hpp"
#include <cmath>
#include <assert.h>

void Pose::transform(std::vector<Vec2> *points)
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

Pose Pose::relativePose(std::vector<const Vec2*> &a, std::vector<const Vec2*> &b)
{
    assert(a.size() == b.size());

    int n = a.size();
    double x1 = 0.0, x2 = 0.0, y1 = 0.0, y2 = 0.0;
    double xx = 0.0, yy = 0.0, xy = 0.0, yx = 0.0;


    for (int i = 0; i < n; i++)
    {
        const Vec2 &p1 = *a[i];
        const Vec2 &p2 = *b[i];
        x1 += p1.x;
        x2 += p2.x;
        y1 += p1.y;
        y2 += p2.y;
        xx += p1.x * p2.x;
        yy += p1.y * p2.y;
        xy += p1.x * p2.y;
        yx += p1.y * p2.x;
    }


    double N = (double)n;

    double Sxx = xx - x1 * x2 / N;
    double Syy = yy - y1 * y2 / N;
    double Sxy = xy - x1 * y2 / N;
    double Syx = yx - y1 * x2 / N;

    double xm1 = x1 / N;
    double xm2 = x2 / N;
    double ym1 = y1 / N;
    double ym2 = y2 / N;

    double angle = atan2(Sxy - Syx, Sxx + Syy);

    double c = cos(angle);
    double s = sin(angle);

    Pose pose;
    pose.pos.x = xm2 - (xm1 * c - ym1 * s);
    pose.pos.y = ym2 - (xm1 * s + ym1 * c);
    pose.angle = angle;


    return pose;
}


std::ostream &operator<<(std::ostream &os, const Pose &pose)
{
	os  << "("
        << pose.pos.x << ","
        << pose.pos.y << ","
        << pose.angle << ")";
	return os;
}
