#pragma once

#include <optional>

#include "scitos_common/vec2.hpp"

namespace grid {

inline std::optional<Vec2<int>> world_to_grid(Vec2<float> pos,
                                              Vec2<float> origin, float width,
                                              float height, float resolution) {
  auto transformed = pos - origin;
  if (transformed.x < 0 || transformed.x > width || transformed.y < 0 ||
      transformed.y > height) {
    return std::nullopt;
  }
  return std::optional(transformed / resolution);
}

inline Vec2<float> grid_to_world(Vec2<int> pos, Vec2<float> origin, float width,
                                 float height, float resolution) {
  auto transformed = pos * resolution + origin + Vec2<float>(0.5f, 0.5f);
  return transformed;  // TODO: remove variable initialization
}
} // namespace grid
