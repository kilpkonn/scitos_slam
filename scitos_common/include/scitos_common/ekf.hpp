#pragma once

#include <chrono>
#include <cmath>
#include <tuple>
#include <vector>

#include <eigen3/Eigen/Eigen>

#include "scitos_common/map/line.hpp"
#include "scitos_common/map/map.hpp"

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
   * u = [v, w]'
   */
  std::tuple<Vector3f, Matrix3f> step(Vector2f u, std::chrono::milliseconds t,
                                      std::vector<map::Line<float>> z,
                                      map::Map<float> map) {
    float dt = static_cast<float>(t.count()) / 1000.f;
    // Motion update
    Vector3f m{m_(0) + u(0) * dt * std::cos(m_(2) + u(1) * dt / 2.f),
               m_(1) + u(0) * dt * std::sin(m_(2) + u(1) * dt / 2.f),
               m_(2) + u(1) * dt};

    Matrix2f M{{a1_ * u(0) * u(0) + a2_ * u(1) * u(1), 0},
               {0, a3_ * u(0) * u(0) + a4_ * u(1) * u(1)}};

    Matrix3f G{{1.f, 0.f, -u(0) * dt * std::sin(m_(2) + u(1) * dt / 2.f)},
               {0.f, 1.f, u(0) * dt * std::cos(m_(2) + u(1) * dt / 2.f)},
               {0.f, 0.f, 1.f}};

    Matrix<float, 3, 2> V{
        {dt * std::cos(m_(2) + u(1) * dt / 2.f),
         -0.5f * dt * dt * std::sin(m_(2) + u(1) * dt / 2.f)},
        {dt * std::sin(m_(2) + u(1) * dt / 2.f),
         0.5f * dt * dt * std::cos(m_(2) + u(1) * dt / 2.f)},
        {0.f, dt}};

    Matrix3f sigma = G * sigma_* G.transpose() + V * M * V.transpose();

    // TODO: Features to position
    // TODO: EKF correction to m and sigma
    
    m_= m;
    sigma_= sigma;

    return std::make_tuple(m, sigma);
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
};

} // namespace scitos_common
