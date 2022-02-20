#include <gtest/gtest.h>
#include <optional>

#include "scitos_common/grid/transformations.hpp"
#include "scitos_common/vec2.hpp"

TEST(ZeroPosZeroOriginTest, WorldToGrid) {
  Vec2<float> pos = {0, 0};
  Vec2<float> origin = {0, 0};
  auto expected = std::optional(Vec2<int>(0, 0));
  ASSERT_EQ(grid::world_to_grid(pos, origin, 1.f, 1.f, 1.f), expected);
}

TEST(PositivePosZeroOriginTest, WorldToGrid) {
  Vec2<float> pos = {1, 1};
  Vec2<float> origin = {0, 0};
  auto expected = std::optional(Vec2<int>(1, 1));
  ASSERT_EQ(grid::world_to_grid(pos, origin, 2.f, 2.f, 1.f), expected);
}

TEST(NegativePosXZeroOriginTest, WorldToGrid) {
  Vec2<float> pos = {-1, 1};
  Vec2<float> origin = {0, 0};
  auto expected = std::nullopt;
  ASSERT_EQ(grid::world_to_grid(pos, origin, 2.f, 2.f, 1.f), expected);
}

TEST(NegativePosYZeroOriginTest, WorldToGrid) {
  Vec2<float> pos = {1, -1};
  Vec2<float> origin = {0, 0};
  auto expected = std::nullopt;
  ASSERT_EQ(grid::world_to_grid(pos, origin, 2.f, 2.f, 1.f), expected);
}

TEST(NegativePosSameOriginTest, WorldToGrid) {
  Vec2<float> pos = {-4, -1};
  Vec2<float> origin = {-4, -1};
  auto expected = std::optional(Vec2<int>(0, 0));
  ASSERT_EQ(grid::world_to_grid(pos, origin, 2.f, 2.f, 1.f), expected);
}

TEST(NegativePosMoreNegativeOriginTest, WorldToGrid) {
  Vec2<float> pos = {-4, -1};
  Vec2<float> origin = {-6, -2};
  auto expected = std::optional(Vec2<int>(2, 1));
  ASSERT_EQ(grid::world_to_grid(pos, origin, 2.f, 2.f, 1.f), expected);
}

TEST(Scale2XTest, WorldToGrid) {
  Vec2<float> pos = {-4, -1};
  Vec2<float> origin = {-6, -2};
  auto expected = std::optional(Vec2<int>(1, 0));
  ASSERT_EQ(grid::world_to_grid(pos, origin, 2.f, 2.f, 2.f), expected);
}

TEST(Scale0_5XTest, WorldToGrid) {
  Vec2<float> pos = {-4, -1};
  Vec2<float> origin = {-6, -2};
  auto expected = std::optional(Vec2<int>(4, 2));
  ASSERT_EQ(grid::world_to_grid(pos, origin, 2.f, 2.f, 0.5f), expected);
}

TEST(OutOfBoundsXTest, WorldToGrid) {
  Vec2<float> pos = {-3, -1};
  Vec2<float> origin = {-6, -2};
  auto expected = std::nullopt;
  ASSERT_EQ(grid::world_to_grid(pos, origin, 2.f, 2.f, 2.f), expected);
}

TEST(OutOfBoundsYTest, WorldToGrid) {
  Vec2<float> pos = {-4, 1};
  Vec2<float> origin = {-6, -2};
  auto expected = std::nullopt;
  ASSERT_EQ(grid::world_to_grid(pos, origin, 2.f, 2.f, 2.f), expected);
}

TEST(NoScaleTest, GridToWorld) {
  Vec2<int> pos = {4, 1};
  Vec2<float> origin = {-6, -2};
  auto expected = Vec2<float>(-1.5f, -0.5f);
  auto res = grid::grid_to_world(pos, origin, 4.f, 4.f, 1.f);
  ASSERT_FLOAT_EQ(res.x, expected.x);
  ASSERT_FLOAT_EQ(res.y, expected.y);
}

TEST(Scale2XTest, GridToWorld) {
  Vec2<int> pos = {4, 1};
  Vec2<float> origin = {-6, -2};
  auto expected = Vec2<int>(3.f, 1.f);
  auto res = grid::grid_to_world(pos, origin, 8.f, 8.f, 2.f);
  ASSERT_FLOAT_EQ(res.x, expected.x);
  ASSERT_FLOAT_EQ(res.y, expected.y);
}

TEST(Scale0_5XTest, GridToWorld) {
  Vec2<int> pos = {4, 2};
  Vec2<float> origin = {-6, -2};
  auto expected = Vec2<float>(-3.75, -0.75);
  auto res = grid::grid_to_world(pos, origin, 4.f, 4.f, 0.5f);
  ASSERT_FLOAT_EQ(res.x, expected.x);
  ASSERT_FLOAT_EQ(res.y, expected.y);
}

TEST(NoScaleTest, WorldToGridToWorld) {
  Vec2<float> pos = {4, 2};
  Vec2<float> origin = {-1, 2};
  auto expected = pos + Vec2(0.5f, 0.5f);
  auto tmp = grid::world_to_grid(pos, origin, 8.f, 8.f, 1.0);
  auto res = grid::grid_to_world(tmp.value(), origin, 8.f, 8.f, 1.0f);
  ASSERT_FLOAT_EQ(res.x, expected.x);
  ASSERT_FLOAT_EQ(res.y, expected.y);
}

TEST(Scale2XTest, WorldToGridToWorld) {
  Vec2<float> pos = {4, 2};
  Vec2<float> origin = {-1, 2};
  auto expected = pos + Vec2(0.f, 1.f);
  auto tmp = grid::world_to_grid(pos, origin, 8.f, 8.f, 2.0);
  auto res = grid::grid_to_world(tmp.value(), origin, 8.f, 8.f, 2.0f);
  ASSERT_FLOAT_EQ(res.x, expected.x);
  ASSERT_FLOAT_EQ(res.y, expected.y);
}

TEST(Scale0_5XTest, WorldToGridToWorld) {
  Vec2<float> pos = {4, 2};
  Vec2<float> origin = {-1, 2};
  auto expected = pos + Vec2(0.25f, 0.25f);
  auto tmp = grid::world_to_grid(pos, origin, 8.f, 8.f, 0.5f);
  auto res = grid::grid_to_world(tmp.value(), origin, 8.f, 8.f, 0.5f);
  ASSERT_FLOAT_EQ(res.x, expected.x);
  ASSERT_FLOAT_EQ(res.y, expected.y);
}
