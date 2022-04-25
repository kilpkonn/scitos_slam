#pragma once
#include <chrono>
#include <vector>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include "scitos_common/Vec2Array.h"
#include "scitos_common/pid.hpp"
#include "scitos_common/polar2.hpp"
#include "scitos_common/vec2.hpp"

class MotionController {
public:
  explicit MotionController(ros::NodeHandle nh);
  void step(const ros::TimerEvent &event);

private:
  ros::NodeHandle nh_;

  ros::Subscriber odometrySub_;
  ros::Subscriber waypointsSub_;

  ros::Publisher waypointsPub_;
  ros::Publisher controlPub_;
  ros::Publisher errorPub_;

  ros::Timer mainTimer_;

  nav_msgs::OdometryPtr odometry_;

  std::vector<Vec2<float>> waypoints_;
  float pointMargin_ = 0.1;
  float maxAngleToDrive_ = 0.2;
  float maxSpeed_ = 0.5;
  float maxAngle_ = 0.5;
  uint32_t waypointIndex_ = 0;

  PID<float> trajectoryPidDist_;
  PID<float> trajectoryPidAng_;

  void odometryCallback(nav_msgs::OdometryPtr msg);
  void waypointsCallback(scitos_common::Vec2Array msg);
  void publishWaypoints() const;
};
