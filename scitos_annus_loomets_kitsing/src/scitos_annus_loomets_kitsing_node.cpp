#include <chrono>
#include <ros/ros.h>

#include "scitos_annus_loomets_kitsing/motion_controller.hpp"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "motion_controller");
  ros::NodeHandle n;

  // Uncomment to add predefined path etc.
  // MotionController controller(n);
  // ros::Timer controllerTimer =
  //     n.createTimer(ros::Duration(0.01), &MotionController::step, &controller);

  ros::spin();

  return 0;
}
