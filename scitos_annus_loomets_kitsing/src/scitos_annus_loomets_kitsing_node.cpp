#include <chrono>
#include <ros/ros.h>

#include "scitos_annus_loomets_kitsing/motion_controller.hpp"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "motion_controller");
  ros::NodeHandle n;

  MotionController controller(n);

  // ros::Rate r(100);

  // while (n.ok()) {
  //   ros::spinOnce();
  //   r.sleep();
  // }
  ros::spin();

  return 0;
}
