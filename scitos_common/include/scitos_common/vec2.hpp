#pragma once

#include <cmath>
#include <compare>
#include <vector>
template <typename T> struct Vec2 {
  T x, y;

  Vec2() : x{T()}, y{T()} {}
  Vec2(T x, T y) : x{x}, y{y} {}

  Vec2<T> operator+(const Vec2<T> &o) const { return {x + o.x, y + o.y}; }
  Vec2<T> operator-(const Vec2<T> &o) const { return {x - o.x, y - o.y}; }
  Vec2<T> operator*(float r) const { return {r * x, r * y}; }
  Vec2<T> operator/(float r) const { return {x / r, y / r}; }
  Vec2<T> operator+=(const Vec2<T> &o) const {
    x += o.x;
    y += o.y;
    return *this;
  }
  Vec2<T> operator-=(const Vec2<T> &o) const {
    x -= o.x;
    y -= o.y;
    return *this;
  }
  auto operator<=>(float n) const {
    auto m = length();
    if (m < n)
      return std::weak_ordering::less;
    if (m > n)
      return std::weak_ordering::greater;
    return std::weak_ordering::equivalent;
  }

  operator float() const { return length(); }

  float dot() const { return x * x + y * y; }
  float length() const { return std::sqrt(dot()); }
};
