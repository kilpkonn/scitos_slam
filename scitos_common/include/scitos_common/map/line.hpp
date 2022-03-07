#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include "scitos_common/vec2.hpp"

namespace scitos_common::map {

template <typename T> struct Line {
  Line(const Vec2<T> &s, const Vec2<T> &e) : p1{s}, p2{e} {}
  Line(const Vec2<T> &s, const Vec2<T> &e, float conf)
      : p1{s}, p2{e}, confidence{conf} {}
  float slope() const { return (p2.y - p1.y) / (p2.x - p1.x); }
  float yIntersept() const { return p1.y - slope() * p1.x; }
  bool overlaps(const Line<T> &line, float padding) {
    Vec2<T> minSelf = Vec2(std::min(p1.x, p2.x), std::min(p1.y, p2.y));
    Vec2<T> maxSelf = Vec2(std::max(p1.x, p2.x), std::max(p1.y, p2.y));
    Vec2<T> minOther =
        Vec2(std::min(line.p1.x, line.p2.x), std::min(line.p1.y, line.p2.y));
    Vec2<T> maxOther =
        Vec2(std::max(line.p1.x, line.p2.x), std::max(line.p1.y, line.p2.y));

    bool xOverlap =
        !(minSelf.x - padding > maxOther.x || maxSelf.x + padding < minOther.x);
    bool yOverlap =
        !(minSelf.y - padding > maxOther.y || maxSelf.y + padding < minOther.y);

    return xOverlap && yOverlap;
  }

  Vec2<T> toHoughSpace() { return {slope(), yIntersept()}; }
  float perpendicularDistance(const Vec2<T> &p) {
    Vec2<T> vec1(p.x - p1.x, p.y - p1.y);
    Vec2<T> vec2(p2.x - p1.x, p2.y - p1.y);
    float d_vec2 = vec2.length();
    float cross_product = vec1.x * vec2.y - vec2.x * vec1.y;
    float d = abs(cross_product / d_vec2);
    return d;
  }

  Vec2<T> projectInf(const Vec2<T> &p) const {
    Vec2<T> heading = (p2 - p1).normalize();
    Vec2<T> lhs = p - p1;
    return p1 + heading * lhs.dot(heading);
  }

  Vec2<T> project(const Vec2<T> &p) const {
    Vec2<T> r = projectInf(p);
    float magMax = (p2 - p1).length();
    return p1 + (p2 - p1).normalize() * std::clamp(r.length(), 0.f, magMax);
  }

  /*!
   * @see: https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
   */
  std::optional<Vec2<T>> intersectInf(const Line<T> &o) const {
    const double Precision = std::sqrt(std::numeric_limits<double>::epsilon());
    double d = -v().cross(o.v());
    if (std::abs(d) < Precision) {
      return std::nullopt;
    } else {
      double x = (p1.cross(p2) * o.v().x - v().x * o.p1.cross(o.p2)) / d;
      double y = (p1.cross(p2) * o.v().y - v().y * o.p1.cross(o.p2)) / d;
      return std::make_optional(Vec2<T>(static_cast<T>(x), static_cast<T>(y)));
    }
  }

  std::optional<Vec2<T>> intersect(const Line<T> &o) const {
    auto p = intersectInf(o);
    if (!p.has_value())
      return std::nullopt;

    if (!contains(*p) || !o.contains(*p))
      return std::nullopt;

    return p;
  }

  bool contains(const Vec2<T> &p) const {
    float e = std::numeric_limits<float>::epsilon();
    bool onLine = std::abs(slope() * (p.x - p1.x) - (p.y - p1.y)) < e;
    return onLine && std::min(p1.x, p2.x) - e < p.x &&
           p.x < std::max(p1.x, p2.x) + e;
    // return std::abs(length() - (p - p1).length() - (p - p2).length()) < e;
  }

  float length() const { return (p1 - p2).length(); }

  Line<T> merge(const Line<T> &o) const {
    // See which way to match
    auto np1Closer = (o.p1 - p1).length() < (o.p2 - p1).length();
    auto np1 = np1Closer ? o.p1 : o.p2;
    auto np2 = !np1Closer ? o.p1 : o.p2;
    // Give less influence to lower confidence and low length
    float rpow2 = confidence * confidence * length();
    float npow2 = o.confidence * o.confidence * o.length();
    return {(p1 * rpow2 + np1 * npow2) / (rpow2 + npow2),
            (p2 * rpow2 + np2 * npow2) / (rpow2 + npow2),
            confidence + o.confidence - confidence * o.confidence};
  }
  Vec2<T> dir() const { return v().normalize(); }
  Vec2<T> v() const { return p2 - p1; }

  Vec2<T> p1;
  Vec2<T> p2;
  float confidence = 0;
};
} // namespace scitos_common::map
