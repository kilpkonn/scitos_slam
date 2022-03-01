#pragma once

#include <optional>

#include "scitos_common/vec2.hpp"

namespace grid {

inline std::optional<Vec2<int>> world_to_grid(const Vec2<float> pos,
                                              const Vec2<float> origin,
                                              const Vec2<float> grid_size, 
                                              const float resolution) {
  auto transformed = pos - origin;
  if (transformed.x < 0 || transformed.x >= grid_size.x
      || transformed.y < 0 || transformed.y >= grid_size.y) {
    return std::nullopt;
  }
  return std::optional(transformed / resolution);
}

inline Vec2<float> grid_to_world(Vec2<int> pos, Vec2<float> origin, float resolution) {
  return (static_cast<Vec2<float>>(pos) + Vec2(0.5f, 0.5f)) * resolution +
         origin;
}
} // namespace grid
