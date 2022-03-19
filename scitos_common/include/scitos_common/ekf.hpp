#pragma once

#include <cmath>
#include <tuple>
#include <vector>

#include <eigen3/Eigen/Eigen>

#include "scitos_common/map/line.hpp"
#include "scitos_common/map/map.hpp"

namespace scitos_common {

using Eigen::Matrix2f;
using Eigen::Matrix3f;
using Eigen::MatrixXd;
using Eigen::Vector3f;

template <typename T> class EKF {
  /*!
   * EKF for differential drive robots
   *
   * m = [x, y, phi]'
   * u = [v, w]'
   */
  void step(Vector3f m_old, MatrixXd sigma_old, Vector2f u, float dt,
            std::vector<map::Line<T>> z, map::Map<T> map) {
    // Motion update
    Vector3f m{m_old(0) + u(0) * dt * std::cos(m_old(2) + u(1) * dt / 2.f),
               m_old(1) + u(0) * dt * std::sin(m_old(2) + u(1) * dt / 2.f),
               m_old(2) + u(1) * dt};

    Matrix2f M{{a1 * u(0) * u(0) + a2 * u(1) * u(1), 0},
               {0, a3 * u(0) * u(0) + a4 * u(1) * u(1)}};

    Matrix3f G{{1.f, 0.f, -u(0) * dt * std::sin(m_old(2) + u(1) * dt / 2.f)},
               {0.f, 1.f, u(0) * dt * std::cos(m_old(2) + u(1) * dt / 2.f)},
               {0.f, 0.f, 1.f}};

    // TODO: This should be 3 row 2 col matrix
    Matrix3f V{{dt * std::cos(m_old(2) + u(1) * dt / 2.f),
                -0.5f * dt * dt * std::sin(m_old(2) + u(1) * dt / 2.f)},
               {dt * std::sin(m_old(2) + u(1) * dt / 2.f),
                0.5f * dt * dt * std::cos(m_old(2) + u(1) * dt / 2.f)},
               {0.f, dt}};

    Matrix3f sigma = G * sigma_old * G.transpose() + V * M * V.transpose();

    // TODO: Features to position
    // TODO: EKF update to m and sigma
    return std::make_tuple(m, sigma);
  }

private:
  // TODO: Sensor covariances, add to params
  float a1;
  float a2;
  float a3;
  float a4;
};

} // namespace scitos_common
