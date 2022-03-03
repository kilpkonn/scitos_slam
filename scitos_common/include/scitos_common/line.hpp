#pragma once

#include <algorithm>
#include <vector>

#include "scitos_common/vec2.hpp"

template <typename T> class Line {

public:
  Line(const Vec2<T> &s, const Vec2<T> &e) : start_{s}, end_{e} {}
  float getConfidance() { return confidance_; }
  // Chack if lines are parallel
  // @param line - line to compare to
  // @param r - maximum offset between lines
  // @param e - maximum direction difference
  bool isParallel(const Line<T> &line, float r, float e) {
    // TODO: Compare slopes, compare distances
    return false;
  }

  Line<T> merge(const Line<T> line) {
    // TODO merge two lines
    if (closeToLine(line) && isParallel(line)) {
      /* should merge

              this should take confidance into consideration?
              maybe confidance1 * confidance2 = newConfidance?

              then take two most far points to make new line
              what to do with old lines? should there be a destructor in the
         class?

      */
      return true;
    }
  }

  float slope() { return (end_.y - start_.y) / (end_.x - start_.x); }
  float yIntersept() { return start_.y - slope() * start_.x; }

  bool overlaps(const Line<T> &line) {
    Vec2<T> minSelf = Vec2(std::min(start_.x, end_.x), std::min(start_.y, end_.y));
    Vec2<T> maxSelf = Vec2(std::max(start_.x, end_.x), std::max(start_.y, end_.y));
    Vec2<T> minOther = Vec2(std::min(line.start_.x, line.end_.x), std::min(line.start_.y, line.end_.y));
    Vec2<T> maxOther = Vec2(std::max(line.start_.x, line.end_.x), std::max(line.start_.y, line.end_.y));

    bool xOverlap = !(minSelf.x > maxOther.x || maxSelf.x < minOther.x);
    bool yOverlap = !(minSelf.y > maxOther.y || maxSelf.y < minOther.y);

    return xOverlap && yOverlap;
  }

private:
  Vec2<T> start_;
  Vec2<T> end_;
  float confidance_ = 0;
};
