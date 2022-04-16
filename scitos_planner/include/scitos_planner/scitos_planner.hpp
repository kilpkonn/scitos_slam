#pragma once

#include <cstdint>
#include <vector>

#include <ros/ros.h>

#include "scitos_common/LineArray.h"

#include "scitos_common/map/map.hpp"
#include "scitos_common/vec2.hpp"

class Planner {
public:
  explicit Planner(ros::NodeHandle nh);
  void step(const ros::TimerEvent &event);

private:
  ros::NodeHandle nh_;

  ros::Subscriber mapSub_;


  uint32_t n_ = 500;
  float d_ = 0.1f;
  float endThreshold_ = 0.1f;
  Vec2<float> goal_{0.f, 0.f};
  scitos_common::map::Map<float> map_;

  void mapCallback(scitos_common::LineArray msg);
};
