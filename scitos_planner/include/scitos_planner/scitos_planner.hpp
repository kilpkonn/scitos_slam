#pragma once

#include <cstdint>
#include <vector>

#include <ros/ros.h>

#include "scitos_common/LineArray.h"

#include "scitos_common/map/map.hpp"

class Planner {
public:
  explicit Planner(ros::NodeHandle nh);
  void step(const ros::TimerEvent &event);

private:
  ros::NodeHandle nh_;

  ros::Subscriber mapSub_;


  uint32_t n_ = 500;
  float d_ = 0.1f;
  scitos_common::map::Map<float> map_;

  void mapCallback(scitos_common::LineArray msg);
};
