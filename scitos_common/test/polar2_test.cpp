#include <gtest/gtest.h>

#include <scitos_common/polar2.hpp>

TEST(ZeroToZeroTest, Addition)
{
  Polar2<float> a = {0, 0};
  Polar2<float> b = {0, 0};
  Polar2<float> expected = {0, 0};
  ASSERT_EQ((a + b), expected);
}

TEST(ZeroToNonzeroTest, Addition)
{
  Polar2<float> a = {0, 0};
  Polar2<float> b = {1, 1};
  Polar2<float> expected = {1, 1};
  ASSERT_EQ((a + b), expected);
}

TEST(NonzeroToZeroTest, Addition)
{
  Polar2<float> a = {1, 1};
  Polar2<float> b = {0, 0};
  Polar2<float> expected = {1, 1};
  ASSERT_EQ((a + b), expected);
}

TEST(SelfToSelfTest, Addition)
{
  Polar2<float> a = {1, 1};
  Polar2<float> expected = {2, 1};
  ASSERT_EQ((a + a), expected);
}

TEST(SimpleTest, Addition)
{
  Polar2<float> a = {7, 1};
  Polar2<float> b = {5, 0.5};
  Polar2<float> sum = a + b;
  Polar2<float> expected = {11.6375, 0.792531};
  ASSERT_NEAR(sum.r, expected.r, 0.001);
  ASSERT_NEAR(sum.theta, expected.theta, 0.001);
}

TEST(SelfToSelfTest, Subtraction)
{
  Polar2<float> a = {1, 1};
  Polar2<float> sub = a - a;
  ASSERT_NEAR(sub.r, 0, 0.001);
}

TEST(SimpleTest, Subtraction)
{
  Polar2<float> a = {7, 1};
  Polar2<float> b = {5, 0.5};
  Polar2<float> sub = a - b;
  Polar2<float> expected = {3.54530967, 1.74251166};
  ASSERT_NEAR(sub.r, expected.r, 0.001);
  ASSERT_NEAR(sub.theta, expected.theta, 0.001);
}

TEST(AdditionTest, InPlace)
{
  Polar2<float> a = {7, 1};
  Polar2<float> b = {5, 0.5};
  a += b;
  Polar2<float> expected = {11.6375, 0.792531};
  ASSERT_NEAR(a.r, expected.r, 0.001);
  ASSERT_NEAR(a.theta, expected.theta, 0.001);
}

TEST(SubtractionTest, InPlace)
{
  Polar2<float> a = {7, 1};
  Polar2<float> b = {5, 0.5};
  a -= b;
  Polar2<float> expected = {3.54530967, 1.74251166};
  ASSERT_NEAR(a.r, expected.r, 0.001);
  ASSERT_NEAR(a.theta, expected.theta, 0.001);
}