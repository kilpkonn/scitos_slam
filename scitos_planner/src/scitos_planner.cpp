
#include <ros/package.h>

#include "scitos_planner/scitos_planner.hpp"

Planner::Planner(ros::NodeHandle nh) : nh_{nh} {}

void Planner::step(const ros::TimerEvent &event) {}
