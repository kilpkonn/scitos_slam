#pragma once

#include <cmath>
#include <vector>

#include <tf/transform_datatypes.h>

#include "scitos_common/vec2.hpp"
#include "scitos_common/util.hpp"

#include "scitos_common/Polar2.h"

template <typename T> struct Polar2 {
  T r, theta;

  Polar2() : r{T()}, theta{T()} {}
  Polar2(T r, T theta) : r{r}, theta{Util::normalize_angle(theta)} {}
  Polar2(Vec2<T> v) : r{v.length()}, theta{atan2(v.y, v.x)} {}

  Polar2<T> operator+(const Polar2<T> &o) const { return {sqrt(r * r + o.r * o.r + 2 * r * o.r * cos(theta - o.theta))
                                                          , theta + atan2(o.r * sin(o.theta - theta), r + o.r * cos(o.theta - theta))}; }
  Polar2<T> operator-(const Polar2<T> &o) const { return *this + o.opposite(); }
  Polar2<T> operator*(float s) const { return {r * s, theta}; }
  Polar2<T> operator/(float s) const { return {r / s, theta}; }
  Polar2<T> operator+=(const Polar2<T> &o) {
    T new_r = sqrt(r * r + o.r * o.r + 2 * r * o.r * cos(theta - o.theta));
    theta += atan2(o.r * sin(o.theta - theta), r + o.r * cos(o.theta - theta));
    r = new_r;
    return *this;
  }
  Polar2<T> operator-=(const Polar2<T> &o) {
    T new_r = sqrt(r * r + o.r * o.r + 2 * r * o.r * cos(theta - o.theta - M_PI));
    theta += atan2(o.r * sin(M_PI + o.theta - theta), r + o.r * cos(M_PI + o.theta - theta));
    r = new_r;
    return *this;
  }
  inline bool operator==(const Polar2<T>& o) const { return r == o.r && theta == o.theta; }
  auto operator<(float n) const { return length() < n; }
  auto operator<=(float n) const { return length() <= n; }
  auto operator>(float n) const { return length() > n; }
  auto operator>=(float n) const { return length() >= n; }

  Polar2<T> opposite() const { return {r, theta + M_PI}; }
  scitos_common::Polar2 toMsg() {
    scitos_common::Polar2 msg;
    msg.r = r;
    msg.theta = theta;
    return msg;
  }

  operator float() const { return length(); }
  operator Vec2<T>() const { return {r * cos(theta), r * sin(theta)}; }
  operator tf::Vector3() const { return {r * cos(theta), r * sin(theta), 0}; }

  float dot() const { return r * r; }
  float length() const { return r; }
};
