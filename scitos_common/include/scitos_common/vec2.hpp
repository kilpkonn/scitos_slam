#pragma once

#include <cmath>
#include <iostream>
#include <vector>

#include <opencv2/core/types.hpp>
#include <tf/transform_datatypes.h>

template <typename T> struct Vec2 {
  T x, y;

  Vec2() : x{T()}, y{T()} {}
  Vec2(T x, T y) : x{x}, y{y} {}
  Vec2(tf::Vector3 v) : x{v.getX()}, y{v.getY()} {}

  Vec2<T> operator+(const Vec2<T> &o) const { return {x + o.x, y + o.y}; }
  Vec2<T> operator-(const Vec2<T> &o) const { return {x - o.x, y - o.y}; }
  Vec2<T> operator*(float r) const { return {r * x, r * y}; }
  Vec2<T> operator/(float r) const { return {x / r, y / r}; }
  Vec2<T> operator+=(const Vec2<T> &o) {
    x += o.x;
    y += o.y;
    return *this;
  }
  Vec2<T> operator-=(const Vec2<T> &o) {
    x -= o.x;
    y -= o.y;
    return *this;
  }
  auto operator<(float n) const { return length() < n; }
  auto operator<=(float n) const { return length() <= n; }
  auto operator>(float n) const { return length() > n; }
  auto operator>=(float n) const { return length() >= n; }
  auto operator==(Vec2<T> &o) const { return x == o.x && y == o.y; }
  friend std::ostream &operator<<(std::ostream &os, const Vec2<T> &v) {
    os << '{' << v.x << ',' << v.y << '}';
    return os;
  }

  operator float() const { return length(); }
  template <typename U> operator Vec2<U>() const {
    return {static_cast<U>(x), static_cast<U>(y)};
  }
  operator cv::Point_<T>() const { return {x, y}; }
  operator tf::Vector3() const { return {x, y, 0}; }

  float dot() const { return x * x + y * y; }
  float dot(const Vec2<T> &o) const { return x * o.x + y * o.y; }
  float cross(const Vec2<T> &o) const { return x * o.y - y * o.x; }
  float length() const { return std::sqrt(dot()); }
  Vec2<T> round() { return {std::round(x), std::round(y)}; }
  Vec2<T> ceil() { return {std::ceil(x), std::ceil(y)}; }
  Vec2<T> normalize() { return *this / length(); }
  // Inner angle
  float angle(const Vec2<T> &o) const {
    return std::acos(std::clamp(dot(o), -1.f, 1.f));
  }

  float angle_nodir(const Vec2<T> &o) const {
    return std::min(angle(o), static_cast<float>(M_PI) - angle(o));
  }
};
