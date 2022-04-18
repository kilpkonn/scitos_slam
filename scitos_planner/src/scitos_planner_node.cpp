#include <chrono>

#include <ros/ros.h>

#include "scitos_planner/scitos_planner.hpp"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "scitos_planner");
  ros::NodeHandle n;

  Planner planner(n);
  ros::Timer mapperTimer =
      n.createTimer(ros::Duration(1.0), &Planner::step, &planner);

  ros::spin();

  return 0;
}
