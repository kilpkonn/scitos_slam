#include <gtest/gtest.h>

#include "scitos_common/grid/morphology.hpp"

TEST(Open, Basic)
{
  nav_msgs::OccupancyGrid tmp;
  grid::open({}, 1.0f, tmp);
}
