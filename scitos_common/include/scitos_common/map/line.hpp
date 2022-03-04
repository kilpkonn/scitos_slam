#pragma once

#include <algorithm>
#include <vector>

#include "scitos_common/vec2.hpp"

namespace scitos_common::map {

template <typename T> class Line {
public:
  Line(const Vec2<T> &s, const Vec2<T> &e) : p1_{s}, p2_{e} {}
  float getConfidance() { return confidence_; }
  float slope() { return (p2_.y - p1_.y) / (p2_.x - p1_.x); }
  float yIntersept() { return p1_.y - slope() * p1_.x; }
  bool overlaps(const Line<T> &line) {
    Vec2<T> minSelf = Vec2(std::min(p1_.x, p2_.x), std::min(p1_.y, p2_.y));
    Vec2<T> maxSelf = Vec2(std::max(p1_.x, p2_.x), std::max(p1_.y, p2_.y));
    Vec2<T> minOther = Vec2(std::min(line.p1_.x, line.p2_.x),
                            std::min(line.p1_.y, line.p2_.y));
    Vec2<T> maxOther = Vec2(std::max(line.p1_.x, line.p2_.x),
                            std::max(line.p1_.y, line.p2_.y));

    bool xOverlap = !(minSelf.x > maxOther.x || maxSelf.x < minOther.x);
    bool yOverlap = !(minSelf.y > maxOther.y || maxSelf.y < minOther.y);

    return xOverlap && yOverlap;
  }

  Vec2<T> toHoughSpace() { return {slope(), yIntersept()}; }

private:
  Vec2<T> p1_;
  Vec2<T> p2_;
  float confidence_ = 0;
};
} // namespace scitos_common
