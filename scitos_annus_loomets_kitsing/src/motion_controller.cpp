#include <chrono>
#include <vector>

#include <XmlRpcValue.h>
#include <tf/transform_datatypes.h>
#include "tf/LinearMath/Matrix3x3.h"

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Twist.h>

#include "scitos_common/pid.hpp"
#include "scitos_common/vec2.hpp"
#include "scitos_common/polar2.hpp"

#include "scitos_annus_loomets_kitsing/motion_controller.hpp"

MotionController::MotionController(ros::NodeHandle nh) : nh_{nh}, rate_{100} {

  XmlRpc::XmlRpcValue waypoints;
  if (nh_.getParam("mission/waypoints", waypoints)) {
    if (waypoints.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      waypoints_.reserve(waypoints_.size());
      for (int i = 0; i < waypoints.size(); i++) {
        XmlRpc::XmlRpcValue trajectoryObject = waypoints[i];

        if (trajectoryObject.getType() == XmlRpc::XmlRpcValue::TypeArray &&
            trajectoryObject.size() == 2 &&
            trajectoryObject[0].getType() == XmlRpc::XmlRpcValue::TypeDouble &&
            trajectoryObject[1].getType() == XmlRpc::XmlRpcValue::TypeDouble) {

          double x = trajectoryObject[0];
          double y = trajectoryObject[1];
          waypoints_.push_back({static_cast<float>(x), static_cast<float>(y)});
        }
      }
    }
  }
  pointMargin_ = nh_.param("mission/distance_margin", 0.1f);
  // TODO: Move these params to yaml
  trajectoryPid_ = PID<Polar2<float>>(0.1, 0.f, 0.f, 100.f, 0.5f);

  odometrySub_ = nh_.subscribe("/controller_diffdrive/odom", 1,
                               &MotionController::odometryCallback, this);

  waypointsPub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "/mission_control/waypoints", 10);
  controlPub_ = nh_.advertise<geometry_msgs::Twist>(
      "/controller_diffdrive/cmd_vel", 3);

  mainTimer_ = nh_.createTimer(rate_, &MotionController::step, this);
}

void MotionController::step(const ros::TimerEvent &event) {
  if (waypointIndex_ >= waypoints_.size())
  {
    return;
  }

  // TODO: send out waypoints
  double roll, pitch, yaw;

  tf::Quaternion quat;
  tf::quaternionMsgToTF(odometry_->pose.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  Vec2<float> current(odometry_->pose.pose.position.x,
                      odometry_->pose.pose.position.y);
  Vec2<float> target = waypoints_.at(waypointIndex_);
  Polar2<float> error = target - current;

  if (error < pointMargin_)
  {
    ++waypointIndex_;
    return;
  }

  std::chrono::nanoseconds dt(event.profile.last_duration.toNSec());
  auto pidOut = trajectoryPid_.accumulate(
      error, std::chrono::duration_cast<std::chrono::milliseconds>(dt));

  geometry_msgs::Twist control;
  control.linear.x = pidOut.r;
  control.angular.z = pidOut.theta;
  controlPub_.publish(control);
}

void MotionController::odometryCallback(nav_msgs::OdometryPtr msg) {
  odometry_ = msg;
}
