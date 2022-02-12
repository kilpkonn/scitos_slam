#pragma once
#include <chrono>
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "scitos_common/pid.hpp"
#include "scitos_common/vec2.hpp"
class MotionController {
public:
  explicit MotionController(ros::NodeHandle nh);


private:
  ros::NodeHandle nh_;
  ros::Rate rate_;

  ros::Subscriber odometrySub_;

  ros::Publisher waypointsPub_;

  ros::Timer mainTimer_;

  nav_msgs::OdometryPtr odometry_;

  std::vector<Vec2<float>> waypoints_;
  float pointMargin_ = 0.1;
  uint32_t waypointIndex_ = 0;

  PID<Vec2<float>> trajectoryPid_;

  void step(const ros::TimerEvent &event);

  void odometryCallback(nav_msgs::OdometryPtr msg);

};
