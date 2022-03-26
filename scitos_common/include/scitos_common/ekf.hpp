#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <tuple>
#include <vector>

#include <eigen3/Eigen/Eigen>

#include "scitos_common/map/line.hpp"
#include "scitos_common/map/map.hpp"
#include "scitos_common/polar2.hpp"
#include "scitos_common/vec2.hpp"

namespace scitos_common {

using Eigen::Matrix;
using Eigen::Matrix2f;
using Eigen::Matrix3f;
using Eigen::MatrixXd;
using Eigen::Vector2f;
using Eigen::Vector3f;

class EKF {
public:
  EKF() : a1_{0.f}, a2_{0.f}, a3_{0.f}, a4_{0.f} {}
  EKF(float a1, float a2, float a3, float a4)
      : a1_{a1}, a2_{a2}, a3_{a3}, a4_{a4} {}

  /*!
   * EKF for differential drive robots
   *
   * @param v - linear velocity
   * @param w - angular velocity
   * @param t - time since last update
   */
  std::tuple<Vector3f, Matrix3f> predict(float v, float w,
                                         std::chrono::milliseconds t) {
    float dt = static_cast<float>(t.count()) / 1000.f;
    // Motion update
    Vector3f m{m_(0) + v * dt * std::cos(m_(2) + w * dt / 2.f),
               m_(1) + v * dt * std::sin(m_(2) + w * dt / 2.f), m_(2) + w * dt};

    Matrix2f M{{a1_ * v * v + a2_ * w * w, 0}, {0, a3_ * v * v + a4_ * w * w}};

    Matrix3f G{{1.f, 0.f, -v * dt * std::sin(m_(2) + w * dt / 2.f)},
               {0.f, 1.f, v * dt * std::cos(m_(2) + w * dt / 2.f)},
               {0.f, 0.f, 1.f}};

    Matrix<float, 3, 2> V{{dt * std::cos(m_(2) + w * dt / 2.f),
                           -0.5f * dt * dt * std::sin(m_(2) + w * dt / 2.f)},
                          {dt * std::sin(m_(2) + w * dt / 2.f),
                           0.5f * dt * dt * std::cos(m_(2) + w * dt / 2.f)},
                          {0.f, dt}};

    Matrix3f sigma = G * sigma_ * G.transpose() + V * M * V.transpose();

    m_ = m;
    sigma_ = sigma;

    return std::make_tuple(m, sigma);
  }

  std::tuple<Vector3f, Matrix3f>
  correct(const map::Map<float> &map,
          const std::vector<map::Line<float>> &lines) {
    auto mapLines = map.getLines();
    if (mapLines.size() < 1) {
      return std::make_tuple(m_, sigma_);
    }

    Vec2<float> pos(m_(0), m_(1));
    ::Polar2<float> rot(1.f, m_(2)); // Unit vector

    for (const auto &line : lines) {
      // TODO: map measured p1 and p2 to x and y and map p1 and p2 to mx and my
      float x1, y1, x2, y2;
      float mx1, my1, mx2, my2;
      float q1 = std::pow(x1 - mx1, 2) + std::pow(y1 - my1, 2);
      float q2 = std::pow(x2 - mx2, 2) + std::pow(y2 - my2, 2);
      float rq1 = sqrt(q1);
      float rq2 = sqrt(q2);
      Matrix<float, 4, 3> H_curr{{(-x1 - mx1) / rq1, (-y1 - my1) / rq1, 0.f},
                                 {(y1 - my1) / q1, (x1 - mx1) / q1, -1.f},
                                 {(-x2 - mx2) / rq2, (-y2 - my2) / rq2, 0.f},
                                 {(y2 - my2) / q2, (x2 - mx2) / q2, -1.f}};
      // TODO:calculate likelyhood and compare with best so far
    }

    // TODO: Find Kalman gain and do the update

    return std::make_tuple(m_, sigma_);
  }

  void setState(Vector3f m) { m_ = m; }
  void setVariances(Matrix3f sigma) { sigma_ = sigma; }

private:
  float a1_;
  float a2_;
  float a3_;
  float a4_;
  // [x, y, phi]'
  Vector3f m_;
  Matrix3f sigma_;
  Matrix3f sensorCovs_;

  // Needs at least 1 line to exist in map
  map::Line<float>
  findMatchingLine(const map::Line<float> &line,
                   const std::vector<map::Line<float>> &mapLines) {
    // NOTE: Somefancy stuff could be used such as max likelyhood.
    // This however, would need more complect evaluation as all of the line
    // is not always visible.
    // Line end points can be used but the one that is in the middle of nowhere
    // would likely be mapped wrongly.
    //
    // Currenlty this is approximation of Mahalanobis distance
    return *std::min_element(mapLines.begin(), mapLines.end(),
                             [&](const auto &lhs, const auto &rhs) {
                               return line.perpendicularDistance(lhs) <
                                      line.perpendicularDistance(rhs);
                             });
  }
};

} // namespace scitos_common
