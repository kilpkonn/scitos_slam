#pragma once

#include <ros/console.h>

#include <cstddef>
#include <functional>
#include <vector>

// NOTE: This is not a "real" algorithm

namespace scitos_common {
template <typename T>
std::vector<T> grow_point_cloud(std::vector<T> points,
                                const std::vector<T> &data,
                                std::function<float(T, T)> s, float epsilon) {
  if (points.size() == 0)
    return data;
  ROS_INFO("C - %d", points.size());

  const size_t points_size = points.size();

  for (size_t i = 0; i < data.size(); i++) {
    const auto c = data.at(i);
    bool close = false;
    for (size_t j = 0; j < points_size; j++) {
      if (s(points.at(j), c) < epsilon) {
        // points.at(j) =
        //     (points.at(j) + c) / 2.f; // TODO: More intelligent algorithm
        close = true;
        continue;
      }
    }
    if (!close) {
      points.push_back(c);
    }
  }

  // TODO: merge close points

  return points;
}
} // namespace scitos_common
