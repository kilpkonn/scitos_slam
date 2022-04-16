#pragma once

#include <cstdint>

#include <ros/ros.h>

class Planner {
public:
  explicit Planner(ros::NodeHandle nh);
  void step(const ros::TimerEvent &event);

private:
  ros::NodeHandle nh_;
};
