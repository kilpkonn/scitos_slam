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
    Vector3f m{m_old(0) + u(0) * dt * std::cos(m_old(2) + u(2) * dt / 2),
               m_old(1) + u(0) * dt * std::sin(m_old(2) + u(2) * dt / 2),
               m_old(2) + u(1) * dt};

    Matrix2f M{{a1 * u(0) * u(0) + a2 * u(1) * u(1), 0},
               {0, a3 * u(0) * u(0) + a4 * u(1) * u(1)}};

    float rot1, trans, rot2; // TODO: actual values, but from where?
    // HACK: Pretty sure this matrix is incorrect, need to take Jacobian on
    // paper to recheck...
    Matrix3f G{{1.f, 0.f, -trans * std::sin(m_old(2) + rot1)},
               {0.f, 1.f, trans * std::cos(m_old(2) + rot1)},
               {0.f, 0.f, 1.f}};
    Matrix3f V{{}, {}, {}};

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
