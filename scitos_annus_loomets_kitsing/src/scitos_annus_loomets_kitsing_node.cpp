#include <ros/ros.h>

#include "scitos_common/pid.hpp"
#include "scitos_common/vec2.hpp"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "pid_controller");
  // TODO: Actual node
  PID<float> p(0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
  Vec2<int> v{1, 2};
  ros::spin();
  return 0;
}
