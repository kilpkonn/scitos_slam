#include <chrono>
#include <ros/ros.h>

#include "scitos_annus_loomets_kitsing/motion_controller.hpp"
#include "scitos_annus_loomets_kitsing/mapper.hpp"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "motion_controller");
  ros::NodeHandle n;

  // MotionController controller(n);
  // ros::Timer controllerTimer =
  //     n.createTimer(ros::Duration(0.01), &MotionController::step, &controller);

  Mapper mapper(n);
  ros::Timer mapperTimer =
      n.createTimer(ros::Duration(0.2), &Mapper::step, &mapper);

  ros::spin();

  return 0;
}
