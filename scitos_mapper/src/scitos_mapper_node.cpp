#include <chrono>

#include <ros/ros.h>

#include "scitos_mapper/scitos_mapper.hpp"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "scitos_mapper");
  ros::NodeHandle n;

  Mapper mapper(n);
  ros::Timer mapperTimer =
      n.createTimer(ros::Duration(0.05), &Mapper::step, &mapper);

  ros::spin();

  return 0;
}
