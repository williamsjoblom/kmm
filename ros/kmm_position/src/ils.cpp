#include "kmm_position/ils.h"
#include "kmm_position/Pose.h"
#include <cmath>
#include <ros/ros.h>

/**
  Creates a Pose that aligns points in laser scan to grid.
*/
Pose calculate_robot_movement(
  Eigen::Vector2f robot_position,
  std::vector<Eigen::Vector2f> &scan,
  std::vector<Eigen::Vector2f> &position_scan,
  std::vector<Eigen::Vector2f> &mapping_scan,
  int iterations,
  float position_proximity,
  float mapping_proximity,
  float position_ignore,
  float mapping_ignore
)
{
  Pose total;

  if (scan.size() == 0) {
    ROS_WARN("Scan size is zero, this is not good!");
    return total;
  }

  for (Eigen::Vector2f p : scan) {
    float squaredDistance = (p - robot_position).squaredNorm();

    // Only use scan data within a certain proximity to position the robot.
    if (squaredDistance < std::pow(position_proximity, 2)) {
      position_scan.push_back(p);
    }

    // Only use scan data within a certain proximity to map the room.
    if (squaredDistance < std::pow(mapping_proximity, 2)) {
      mapping_scan.push_back(p);
    }
  }

  std::vector<Eigen::Vector2f> scanCopy;

  for (int i = 0; i < iterations; i++) {
    scanCopy = position_scan;
    total.transform(&scanCopy);

    std::vector<Eigen::Vector2f> scanPair;
    std::vector<Eigen::Vector2f> gridPair;
    build_pairs(scanCopy, scanPair, gridPair, position_ignore);

    Pose diff = least_squares(scanPair, gridPair);
    total.accumulate(diff);
  }

  total.transform(&position_scan);
  total.transform(&mapping_scan);

  // Used to vizualise what part of the scan was
  // used to position to position the robot.
  std::vector<Eigen::Vector2f> scanPair;
  std::vector<Eigen::Vector2f> gridPair;
  build_pairs(position_scan, scanPair, gridPair, position_ignore);
  position_scan = gridPair;

  // Used to do mapping of walls.
  scanPair.clear();
  gridPair.clear();
  build_pairs(mapping_scan, scanPair, gridPair, mapping_ignore);
  mapping_scan = gridPair;

  return total;
}

/*
Takes the laser scan and adds scan points which are outside diff_percentage's
bounds to vector a. The closest point to the grid from that point is added to
vector b.
*/
void build_pairs(
  const std::vector<Eigen::Vector2f> &scan,
  std::vector<Eigen::Vector2f> &a,
  std::vector<Eigen::Vector2f> &b,
  float ignore
)
{
  for (Eigen::Vector2f point : scan) {
    float x_round = std::round(point[0] / 0.4) * 0.4;
    float y_round = std::round(point[1] / 0.4) * 0.4;
    float x_diff = std::abs(x_round - point[0]);
    float y_diff = std::abs(y_round - point[1]);

    const float max_diff = 0.1;

    bool not_near_corner = !(x_diff < ignore && y_diff < ignore);
    bool is_close_to_grid = (x_diff < max_diff) || (y_diff < max_diff);

    if (not_near_corner && is_close_to_grid) {
        a.push_back(point);
        if (x_diff < y_diff) {
          b.push_back(Eigen::Vector2f(x_round, point[1]));
        } else {
          b.push_back(Eigen::Vector2f(point[0], y_round));
        }
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

    if (n == 0) {
      return Pose();
    }

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
