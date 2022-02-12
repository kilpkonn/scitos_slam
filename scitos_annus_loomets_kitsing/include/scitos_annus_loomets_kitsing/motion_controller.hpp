#pragma once

#include "ros/ros.h"
#include "ros/timer.h"
#include "scitos_common/vec2.hpp"
#include "scitos_common/pid.hpp"
#include <chrono>
#include <vector>
class MotionController {
public:
  explicit MotionController(ros::NodeHandle nh);

  void step(const ros::TimerEvent& event);

private:
  ros::NodeHandle nh_;
  ros::Rate rate_;

  ros::Publisher waypointsPub_;

  ros::Timer mainTimer_;

  std::vector<Vec2<float>> waypoints_;
  float pointMargin_ = 0.1;
  uint32_t waypointIndex_ = 0;

  PID<Vec2<float>> trayectoryPid_;

};
