#include "ils.h"

ils_fit(
  std::vector<Eigen::Vector3f> &a,
  std::vector<Eigen::Vector3f> &b,
  int iterations)
{
  for (int i = 0; i < iterations; i++) {
    // call on relative pose iteratively
  }
}

/*
  Iterative Least Squares Relative Pose
  Finds a relative pose that minimizes the distances between
  the pairs of points in a and b using the least squares method.

  Pairs:
  a[0] <-> b[0]
  a[1] <-> b[1]
  a[2] <-> b[2]
  ...

*/

ils_relative_pose(
  std::vector<Eigen::Vector3f> &a,
  std::vector<Eigen::Vector3f> &b)
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
