#include "kmm_position/ils.h"
#include "kmm_position/Pose.h"
#include <cmath>

/**
  Creates a Pose that aligns points in laser scan to grid. Also modifies scan with Pose.
*/
Pose get_transform_pose(
  std::vector<Eigen::Vector2f> &scan,
  int iterations)
{
  Pose total;
  std::vector<Eigen::Vector2f> newScan; //Scan whithout points in crossings.

  for (int i = 0; i < iterations; i++){
    std::vector<Eigen::Vector2f> scanPair; //Kuriosa: Namnet för en enhet i ett par är "Pair"
    std::vector<Eigen::Vector2f> gridPair;

    //Copies scan
    newScan = scan;
    total.transform(&newScan);
    //Creates pairs after previous transformation
    build_pair(newScan, scanPair, gridPair);
    //Saves scan without points in crossings
    newScan = scanPair;
    // Calculates difference
    Pose diff = least_squares(scanPair, gridPair);
    //Updates newScan so that it can be used to replace laser scan point cloud.
    diff.transform(&newScan);
    total.accumulate(diff);
  }
  scan = newScan;
  return total;
}

/*
Takes the laser scan and adds scan points which are outside diff_percentage's
bounds to vector a. The closest point to the grid from that point is added to
vector b.
*/
void build_pair(
  const std::vector<Eigen::Vector2f> &scan,
  std::vector<Eigen::Vector2f> &a,
  std::vector<Eigen::Vector2f> &b)
{
  const float diff_percentage = 0.1;
  for (Eigen::Vector2f v : scan){
    float x = std::round(v[0]/0.4)*0.4;
    float y = std::round(v[1]/0.4)*0.4;

    if (!(std::abs(x - v[0]) < 0.4*diff_percentage &&
          std::abs(y - v[1]) < 0.4*diff_percentage)){
      a.push_back(v);
      Eigen::Vector2f optimal(x, y);
      b.push_back(optimal);
    }
  }
}

/*
  Iterative Least Squares Relative Pose
  Finds a relative pose that minimizes the distances between
  the pairs of points in a and b using the least squares method.

  Pairs:, a[0] <-> b[0], a[1] <-> b[1]...
*/

Pose least_squares(
  std::vector<Eigen::Vector2f> &a,
  std::vector<Eigen::Vector2f> &b)
{
    assert(a.size() == b.size());

    int n = a.size();

    double x1 = 0.0, x2 = 0.0, y1 = 0.0, y2 = 0.0;
    double xx = 0.0, yy = 0.0, xy = 0.0, yx = 0.0;

    for (int i = 0; i < n; i++)
    {
        const Eigen::Vector2f &p1 = a[i];
        const Eigen::Vector2f &p2 = b[i];
        x1 += p1[0];
        x2 += p2[0];
        y1 += p1[1];
        y2 += p2[1];
        xx += p1[0] * p2[0];
        yy += p1[1] * p2[1];
        xy += p1[0] * p2[1];
        yx += p1[1] * p2[0];
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
    pose.pos[0] = xm2 - (xm1 * c - ym1 * s);
    pose.pos[1] = ym2 - (xm1 * s + ym1 * c);
    pose.angle = angle;

    return pose;
}
