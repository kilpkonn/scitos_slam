
#include <gtest/gtest.h>
#include <optional>

#include "scitos_common/map/line.hpp"
#include "scitos_common/vec2.hpp"

using namespace scitos_common::map;

TEST(Test1, Intersction) {
  Line<float> l1({0.f, 0.f}, {2.f, 2.f});
  Line<float> l2({0.f, 2.f}, {2.f, 0.f});
  auto res = l1.intersect(l2);
  ASSERT_FLOAT_EQ(res->x, 1.f);
  ASSERT_FLOAT_EQ(res->y, 1.f);
}

TEST(Test2, Intersction) {
  Line<float> l1({0.f, 0.f}, {2.f, 2.f});
  Line<float> l2({2.f, 0.f}, {0.f, 2.f});
  auto res = l1.intersect(l2);
  ASSERT_FLOAT_EQ(res->x, 1.f);
  ASSERT_FLOAT_EQ(res->y, 1.f);
}

TEST(Test3, Intersction) {
  Line<float> l1({2.f, 2.f}, {4.f, 3.f});
  Line<float> l2({2.1f, 0.f}, {3.9f, 5.f});
  auto res = l1.intersect(l2);
  ASSERT_FLOAT_EQ(res->x, 3.f);
  ASSERT_FLOAT_EQ(res->y, 2.5f);
}

TEST(Test1, NoIntersction) {
  Line<float> l1({0.f, 0.f}, {2.f, 2.f});
  Line<float> l2({1.f, 1.f}, {3.f, 3.f});
  auto res = l1.intersect(l2);
  ASSERT_TRUE(res == std::nullopt);
}
