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
          const std::vector<map::Line<float>> &lines,
          std::vector<std::pair<Vec2<float>, Vec2<float>>>* matchedLines=nullptr) {

    Vec2<float> pos(m_(0), m_(1));
    ::Polar2<float> rot(1.f, m_(2)); // Unit vector

    auto mapLines = map.getLines();
    if (mapLines.size() < 1) {
      return std::make_tuple(m_, sigma_);
    }

    for (const auto &line : lines) {
      float bestLikelyhood = -1.f;
      Matrix3f H1;
      Matrix3f S1;
      Vector3f dz1;
      bool matchFound = false;
      Vec2<float> matchP1, matchP2;

      // auto mapLine = findMatchingLine(line, mapLines);
      for (const auto &mapLine : mapLines) {
        if (line.perpendicularDistance(mapLine) > 1.f)
          continue;

        for (int i = 0; i < 2; i++) {
          Vec2<float> mp1, mp2;
          if (i == 1) {
            mp1 = mapLine.p1, mp2 = mapLine.p2;
          } else {
            mp1 = mapLine.p2, mp2 = mapLine.p1;
          }

          if (!map.isVisible(mp1, pos))
            continue;

          auto p =
              (line.p1 - mp1).length() < (line.p2 - mp1).length() ? line.p1 : line.p2;

          // Too dangerous, likely incorrectly mapped
          if ((p - mp1).length() > 2.f)
            continue;

          float rq1 = (mp1 - pos).length();
          float q1 = rq1 * rq1;
          Vector3f zHat1 = {(mp1 - pos).length(),
                            ::Polar2<float>(mp1 - pos).theta - rot.theta,
                            line.heading_nodir()};

          Matrix3f H1curr{{(mp1.x - pos.x) / -rq1, (mp1.y - pos.y) / -rq1, 0.f},
                          {(mp1.y - pos.y) / q1, (mp1.x - pos.x) / q1, -1.f},
                          {0.f, 0.f, 0.f}};

          // ROS_INFO("H: %f, %f, %f\n   %f, %f, %f", H1curr(0, 0), H1curr(0,
          // 1),
          //          H1curr(0, 2), H1curr(1, 0), H1curr(1, 1), H1curr(1, 2));
          // ROS_INFO("Sig: % f, % f, % f\n % f, % f, % f ", sigma_(0, 0),
          //          sigma_(0, 1), sigma_(0, 2), sigma_(1, 0), sigma_(1, 1),
          //          sigma_(1, 2));
          Matrix3f S1curr = H1curr * sigma_ * H1curr.transpose() + sensorCovs_;

          Vector3f z1 = {(p - pos).length(),
                         ::Polar2<float>(p - pos).theta - rot.theta,
                         line.heading_nodir()};
          auto dz1_curr = z1 - zHat1;

          // ROS_INFO("S1: %f", S1curr.determinant());
          // ROS_INFO("dz: (%f, %f)", dz1_curr(0), dz1_curr(1));
          float likelyhood1 =
              1 / std::sqrt(2.f * M_PIf32 * S1curr.determinant()) *
              std::exp(-0.5f * static_cast<float>(dz1_curr.transpose() *
                                                  S1curr.inverse() * dz1_curr));
          float likelyhood = likelyhood1; // * likelyhood2;
          ROS_INFO("likelyhood: %f", bestLikelyhood);
          if (likelyhood > bestLikelyhood) {
            bestLikelyhood = likelyhood;
            H1 = H1curr;
            S1 = S1curr;
            dz1 = dz1_curr;
            matchP1 = p;
            matchP2 = mp1;
            matchFound = true;
          }
        }
      }
      if (matchFound) {
        auto K1 = sigma_ * H1.transpose() * S1.inverse();
        m_ = m_ + K1 * dz1;
        sigma_ = (Matrix3f::Identity() - K1 * H1) * sigma_;
        if(matchedLines){
          matchedLines->push_back(std::make_pair(matchP1, matchP2));
        }
      }
    }

    return std::make_tuple(m_, sigma_);
  }

  void setState(Vector3f m) { m_ = m; }
  void setVariances(Matrix3f sigma) { sigma_ = sigma; }
  void setSensorVariances(Matrix3f covs) { sensorCovs_ = covs; }
  Vec2<float> getPos() const { return {m_(0), m_(1)}; }
  float getRotation() const { return m_(2); }
  Matrix3f getCovarince() const { return sigma_; }

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
