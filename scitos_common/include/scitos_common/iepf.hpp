#pragma once

#include <vector>

#include "scitos_common/vec2.hpp"

namespace scitos_common::douglas_peuker {

template <typename T>
float perpendicularDistance(const Vec2<T> &p, const Vec2<T> &line_p1,
                            const Vec2<T> &line_p2) {
  Vec2<T> vec1(p.x - line_p1.x, p.y - line_p1.y);
  Vec2<T> vec2(line_p2.x - line_p1.x, line_p2.y - line_p1.y);
  float d_vec2 = vec2.length();
  float cross_product = vec1.x * vec2.y - vec2.x * vec1.y;
  float d = abs(cross_product / d_vec2);
  return d;
}

/*!
 * @see:
 * https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
 */
template <typename T>
std::vector<Vec2<T>> simplify(const std::vector<Vec2<T>> &pointList,
                              float epsilon) {
  std::vector<Vec2<T>> resultList;

  // Find the point with the maximum distance
  float dmax = 0;
  int index = 0;
  for (int i = 1; i < pointList.size() - 1; ++i) {
    float d = perpendicularDistance(pointList[i], pointList[0],
                                    pointList[pointList.size() - 1]);
    if (d > dmax) {
      index = i;
      dmax = d;
    }
  }
  // If max distance is greater than epsilon, recursively simplify
  if (dmax > epsilon) {
    std::vector<Vec2<T>> pre_part, next_part;
    for (int i = 0; i <= index; ++i)
      pre_part.push_back(pointList[i]);
    for (int i = index; i < pointList.size(); ++i)
      next_part.push_back(pointList[i]);
    // Recursive call
    std::vector<Vec2<T>> resultList1 = simplify(pre_part, epsilon);
    std::vector<Vec2<T>> resultList2 = simplify(next_part, epsilon);

    // combine
    resultList.insert(resultList.end(), resultList1.begin(), resultList1.end());
    resultList.insert(resultList.end(), resultList2.begin() + 1,
                      resultList2.end());
  } else {
    resultList.push_back(pointList[0]);
    resultList.push_back(pointList[pointList.size() - 1]);
  }

  return resultList;
}

} // namespace scitos_common::douglas_peuker
