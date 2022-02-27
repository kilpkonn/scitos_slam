#include <gtest/gtest.h>

#include "scitos_common/grid/morphology.hpp"

TEST(Open, Basic)
{
  grid::open({}, 1.0f);
}