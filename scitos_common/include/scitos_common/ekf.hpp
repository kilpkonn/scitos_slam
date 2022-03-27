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
      float bestLikelyhood = std::numeric_limits<float>::min();
      // NOTE: Maybe only one set of H and S could be used
      Matrix<float, 2, 3> H1;
      // Matrix<float, 2, 3> H2;
      Matrix<float, 2, 2> S1;
      // Matrix<float, 2, 2> S2;
      Matrix<float, 2, 1> dz1;
      // Matrix<float, 2, 1> dz2;
      for (const auto &mapLine : mapLines) {
        float mx1 = mapLine.p1.x, my1 = mapLine.p1.y, mx2 = mapLine.p2.x,
              my2 = mapLine.p2.y;
        for (int i = 0; i < 2; i++) {
          float x1, y1, x2, y2;
          Vec2<float> p1, p2;
          if (i == 1) {
            x1 = line.p1.x, y1 = line.p1.y, x2 = line.p2.x, y2 = line.p2.y;
            p1 = line.p1, p2 = line.p2;
          } else {
            x1 = line.p2.x, y1 = line.p2.y, x2 = line.p1.x, y2 = line.p1.y;
            p1 = line.p2, p2 = line.p1;
          }
          float q1 = std::pow(x1 - mx1, 2) + std::pow(y1 - my1, 2);
          // float q2 = std::pow(x2 - mx2, 2) + std::pow(y2 - my2, 2);
          float rq1 = sqrt(q1);
          // float rq2 = sqrt(q2);
          Matrix<float, 2, 3> H1curr{
              {(-x1 - mx1) / rq1, (-y1 - my1) / rq1, 0.f},
              {(y1 - my1) / q1, (x1 - mx1) / q1, -1.f}};
          // Matrix<float, 2, 3> H2curr{
          //     {(-x2 - mx2) / rq2, (-y2 - my2) / rq2, 0.f},
          //     {(y2 - my2) / q2, (x2 - mx2) / q2, -1.f}};
          // ROS_INFO("H: %f, %f, %f\n   %f, %f, %f", H1curr(0, 0), H1curr(0, 1), H1curr(0, 2), H1curr(1,0), H1curr(1,1), H1curr(1, 2));
          // ROS_INFO("Sig: %f, %f, %f\n   %f, %f, %f", sigma_(0, 0), sigma_(0, 1), sigma_(0, 2), sigma_(1,0), sigma_(1,1), sigma_(1, 2));
          Matrix<float, 2, 2> S1curr =
              H1curr * sigma_ * H1curr.transpose() + sensorCovs_;
          // Matrix<float, 2, 2> S2curr =
          //     H2curr * sigma_ * H2curr.transpose() + sensorCovs_;

          Matrix<float, 2, 1> dz1_curr{{rq1},
                                       {::Polar2<float>(p1 - pos).theta -
                                        ::Polar2<float>(mapLine.p1 - pos).theta}};
          // Matrix<float, 2, 1> dz2_curr{{q2},
          //                              {::Polar2<float>(p2 - pos).theta -
          //                               ::Polar2<float>(mapLine.p2 - pos)}};
          // ROS_INFO("S: %f, %f\n   %f, %f", S1curr(0, 0), S1curr(0, 1), S1curr(1,0), S1curr(1,1));

          // ROS_INFO("S1: %f", S1curr.determinant());
          // ROS_INFO("S2: %f", S2.determinant());
          float likelyhood1 =
              1 / std::sqrt(2.f * M_PIf32 * S1curr.determinant()) *
              std::exp(-0.5f * static_cast<float>(dz1_curr.transpose() *
                                                  S1curr.inverse() * dz1_curr));
          // float likelyhood2 =
          //     1 / std::sqrt(2.f * M_PIf32 * S2curr.determinant()) *
          //     std::exp(-0.5f * static_cast<float>(dz2_curr.transpose() *
          //                                         S1curr.inverse() * dz2_curr));
          float likelyhood = likelyhood1; // * likelyhood2;
          // ROS_INFO("likelyhood: %f", bestLikelyhood);
          if (likelyhood > bestLikelyhood) {
            bestLikelyhood = likelyhood;
            H1 = H1curr;
            // H2 = H2curr;
            S1 = S1curr;
            // S2 = S2curr;
            dz1 = dz1_curr;
            // dz2 = dz2_curr;
          }
        }
      }
      auto K = sigma_ * H1.transpose() * S1.inverse();
      m_ = m_ - K * dz1;
      sigma_ = (Matrix3f::Identity() - K * H1) * sigma_;
    }

    return std::make_tuple(m_, sigma_);
  }

  void setState(Vector3f m) { m_ = m; }
  void setVariances(Matrix3f sigma) { sigma_ = sigma; }
  void setSensorVariances(Matrix2f covs) { sensorCovs_ = covs; }
  Vec2<float> getPos() const { return {m_(0), m_(1)}; }
  float getRotation() const { return m_(2); }

private:
  float a1_;
  float a2_;
  float a3_;
  float a4_;
  // [x, y, phi]'
  Vector3f m_;
  Matrix3f sigma_;
  Matrix2f sensorCovs_;

  // Needs at least 1 line to exist in map
  // TODO: Delete as not used
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
