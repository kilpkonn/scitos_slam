#include <chrono>
#include <cstddef>
#include <vector>

#include <XmlRpcValue.h>
#include <geometry_msgs/Twist.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "scitos_common/pid.hpp"
#include "scitos_common/polar2.hpp"
#include "scitos_common/vec2.hpp"

#include <scitos_common/Polar2.h>

#include "scitos_annus_loomets_kitsing/motion_controller.hpp"

MotionController::MotionController(ros::NodeHandle nh) : nh_{nh} {
  XmlRpc::XmlRpcValue waypoints;
  if (nh_.getParam("/mission/waypoints", waypoints)) {
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
  pointMargin_ = nh_.param("/mission/distance_margin", 0.1f);

  // PID for distance
  float kpDist = nh_.param("/mission/dist_pid_values/kp", 1.0f);
  float kiDist = nh_.param("/mission/dist_pid_values/ki", 0.0f);
  float kdDist = nh_.param("/mission/dist_pid_values/kd", 0.0f);
  float diffErrAlphaDist = nh_.param("/mission/dist_pid_values/diff_err_alpha", 5.0f);
  float maxErrDist = nh_.param("/mission/dist_pid_values/max_err", 0.8f);
  trajectoryPidDist_ =
      PID<float>(kpDist, kiDist, kdDist, maxErrDist, diffErrAlphaDist);

  // PID for angular
  float kpAng = nh_.param("/mission/ang_pid_values/kp", 1.0f);
  float kiAng = nh_.param("/mission/ang_pid_values/ki", 0.0f);
  float kdAng = nh_.param("/mission/ang_pid_values/kd", 0.0f);
  float diffErrAlphaAng = nh_.param("/mission/ang_pid_values/diff_err_alpha", 5.0f);
  float maxErrAng = nh_.param("/mission/ang_pid_values/max_err", 0.8f);
  trajectoryPidAng_ =
      PID<float>(kpAng, kiAng, kdAng, maxErrAng, diffErrAlphaAng);

  odometrySub_ = nh_.subscribe("/controller_diffdrive/odom", 1,
                               &MotionController::odometryCallback, this);

  waypointsPub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "/mission_control/waypoints", 10);
  errorPub_ = nh_.advertise<scitos_common::Polar2>("/debug/PID_error", 10);
  controlPub_ =
      nh_.advertise<geometry_msgs::Twist>("/controller_diffdrive/cmd_vel", 3);

  // mainTimer_ = nh_.createTimer(rate_, &MotionController::step, this);
}

void MotionController::step(const ros::TimerEvent &event) {
  if (waypointIndex_ >= waypoints_.size()) {
    return;
  }

  if (odometry_ == nullptr) {
    return;
  }

  double roll, pitch, yaw;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(odometry_->pose.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  Vec2<float> current(odometry_->pose.pose.position.x,
                      odometry_->pose.pose.position.y);
  Vec2<float> target = waypoints_.at(waypointIndex_);
  Polar2<float> error = target - current;
  error.theta = Util::normalize_angle(error.theta - yaw);

  scitos_common::Polar2 errorMsg = error.toMsg();
  errorMsg.header.stamp = event.current_real;
  errorPub_.publish(errorMsg);

  if (error < pointMargin_) {
    ++waypointIndex_;
    return;
  }

  std::chrono::nanoseconds dt(event.profile.last_duration.toNSec());
  auto pidOutAngular = trajectoryPidAng_.accumulate(
      error.theta, std::chrono::duration_cast<std::chrono::milliseconds>(dt));
  auto pidOutLinear = trajectoryPidDist_.accumulate(
      error.r, std::chrono::duration_cast<std::chrono::milliseconds>(dt));

  geometry_msgs::Twist control;
  control.linear.x = std::max(std::min(pidOutLinear, 0.7f), -0.7f);
  control.angular.z = pidOutAngular;
  controlPub_.publish(control);
  publishWaypoints();
}

void MotionController::odometryCallback(nav_msgs::OdometryPtr msg) {
  odometry_ = msg;
}

void MotionController::publishWaypoints() const {
  visualization_msgs::MarkerArray markers;

  for (size_t i = 0; i < waypoints_.size(); i++) {
    auto p = waypoints_.at(i);
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.a = 1.0;
    marker.color.r = i >= waypointIndex_ ? 1.0 : 0.0;
    marker.color.g = i <= waypointIndex_ ? 1.0 : 0.0;
    marker.color.b = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.pose.position.x = p.x;
    marker.pose.position.y = p.y;
    marker.pose.position.z = 0.05;
    marker.id = i;
    markers.markers.push_back(marker);
  }

  waypointsPub_.publish(markers);
}
