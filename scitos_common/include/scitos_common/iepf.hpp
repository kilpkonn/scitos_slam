#pragma once

#include <functional>
#include <vector>

#include "scitos_common/map/line.hpp"
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

/*!
 * @see:
 * https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
 *
 * @param pointList - points that define a line
 * @param epsilon - max distance for point from straight line
 * @param fc - Function to evaluate confidence on line
 */
template <typename T>
std::vector<map::Line<T>>
simplify2(const std::vector<Vec2<T>> &pointList, float epsilon,
          std::function<float(const std::vector<Vec2<T>> &)> fc) {
  std::vector<map::Line<T>> resultList;

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
    pre_part.reserve(index);
    next_part.reserve(pointList.size() - index);
    for (int i = 0; i <= index; ++i)
      pre_part.push_back(pointList[i]);
    for (int i = index; i < pointList.size(); ++i)
      next_part.push_back(pointList[i]);
    // Recursive call
    std::vector<map::Line<T>> resultList1 = simplify2(pre_part, epsilon, fc);
    std::vector<map::Line<T>> resultList2 = simplify2(next_part, epsilon, fc);

    // combine
    resultList.insert(resultList.end(), resultList1.begin(), resultList1.end());
    resultList.insert(resultList.end(), resultList2.begin(), resultList2.end());
  } else {
    resultList.push_back(
        {pointList[0], pointList[pointList.size() - 1], fc(pointList)});
  }

  return resultList;
}

} // namespace scitos_common::douglas_peuker
