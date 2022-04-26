#include <chrono>
#include <cstddef>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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
      controlCalculator_.waypoints_.reserve(waypoints.size());
      for (int i = 0; i < waypoints.size(); i++) {
        XmlRpc::XmlRpcValue trajectoryObject = waypoints[i];

        if (trajectoryObject.getType() == XmlRpc::XmlRpcValue::TypeArray &&
            trajectoryObject.size() == 2 &&
            trajectoryObject[0].getType() == XmlRpc::XmlRpcValue::TypeDouble &&
            trajectoryObject[1].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
          double x = trajectoryObject[0];
          double y = trajectoryObject[1];
          controlCalculator_.waypoints_.push_back({static_cast<float>(x), static_cast<float>(y)});
        }
      }
    }
  }
  controlCalculator_.pointMargin_ = nh_.param("/mission/distance_margin", 0.1f);

  // PID for distance
  float kpDist = nh_.param("/mission/dist_pid_values/kp", 1.0f);
  float kiDist = nh_.param("/mission/dist_pid_values/ki", 0.0f);
  float kdDist = nh_.param("/mission/dist_pid_values/kd", 0.0f);
  float diffErrAlphaDist =
      nh_.param("/mission/dist_pid_values/diff_err_alpha", 5.0f);
  float maxErrDist = nh_.param("/mission/dist_pid_values/max_err", 0.8f);
  controlCalculator_.trajectoryPidDist_ =
      PID<float>(kpDist, kiDist, kdDist, maxErrDist, diffErrAlphaDist);

  // PID for angular
  float kpAng = nh_.param("/mission/ang_pid_values/kp", 1.0f);
  float kiAng = nh_.param("/mission/ang_pid_values/ki", 0.0f);
  float kdAng = nh_.param("/mission/ang_pid_values/kd", 0.0f);
  float diffErrAlphaAng =
      nh_.param("/mission/ang_pid_values/diff_err_alpha", 5.0f);
  float maxErrAng = nh_.param("/mission/ang_pid_values/max_err", 0.8f);
  controlCalculator_.trajectoryPidAng_ =
      PID<float>(kpAng, kiAng, kdAng, maxErrAng, diffErrAlphaAng);

  odometrySub_ =
      nh_.subscribe("/ekf_odom", 1, &MotionController::odometryCallback, this);
  waypointsSub_ =
      nh_.subscribe("/path", 1, &MotionController::waypointsCallback, this);
  obstaclesSub_ =
      nh_.subscribe("/obstacles_to_avoid", 1, &MotionController::obstaclesCallback, this);

  waypointsPub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "/mission_control/waypoints", 10);
  pathPub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "/debug/path", 10);
  errorPub_ = nh_.advertise<scitos_common::Polar2>("/debug/PID_error", 10);
  controlPub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 3);
}

void MotionController::step(const ros::TimerEvent &event) {
  if (odometry_ == nullptr) {
    geometry_msgs::Twist control;
    controlPub_.publish(control);
    return;
  }

  double roll, pitch, yaw;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(odometry_->pose.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  const Vec2<float> current(odometry_->pose.pose.position.x,
                            odometry_->pose.pose.position.y);

  std::vector<Vec2<float>> simulatedPath;
  std::vector<float> simulatedHeadings;
  float pathLength = 0.0f;
  MotionController::PathEndReason pathEndReason = simulateFuturePath(500, pathLength, &simulatedPath, &simulatedHeadings);
  publishPath(simulatedPath, simulatedHeadings);

  scitos_common::Polar2 errorMsg;

  std::chrono::milliseconds dt(static_cast<long int>((event.current_real - event.last_real).toNSec() * 1e-6f));
  Polar2<float> calculatedControl = controlCalculator_.calculateControl(current, yaw, dt, &errorMsg);
  errorMsg.header.stamp = event.current_real;
  errorPub_.publish(errorMsg);

  geometry_msgs::Twist control;
  if (pathEndReason == MotionController::PathEndReason::FINISH
      || pathEndReason == MotionController::PathEndReason::OK
      || pathLength > controlCalculator_.pointMargin_)
    control.linear.x = calculatedControl.r;
  control.angular.z = calculatedControl.theta;
  controlPub_.publish(control);
  publishWaypoints();
}

MotionController::PathEndReason MotionController::simulateFuturePath(const int steps, float& pathLength, std::vector<Vec2<float>>* path, std::vector<float>* headings) const {
  Vec2<float> robotLocation(odometry_->pose.pose.position.x,
                            odometry_->pose.pose.position.y);
  double roll, pitch, yaw;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(odometry_->pose.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  float robotHeading = yaw;
  float angularSpeed = odometry_->twist.twist.angular.z;

  MotionController::ControlCalculator calculator = controlCalculator_;
  const std::chrono::milliseconds timeStep(10);

  if (path)
    path->reserve(steps);
  if (headings)
    headings->reserve(steps);

  for (int i=0; i<steps; i++) {
    if(calculator.isObstacleNearby(robotLocation))
      return MotionController::PathEndReason::OBSTACLES;
    const Polar2<float> control = calculator.calculateControl(robotLocation, robotHeading, timeStep);
    angularSpeed = std::min(std::max(angularSpeed * 0.5 + std::max(std::min(control.theta, 20.0f), -20.0f) * 0.5, -M_PI_2), M_PI_2);
    const float turn = angularSpeed * timeStep.count() / 1e3f;
    robotHeading += turn / 2;
    const float distance = control.r * timeStep.count() / 1e3f;
    robotLocation.x += cos(robotHeading) * distance;
    robotLocation.y += sin(robotHeading) * distance;
    robotHeading += turn / 2;
    pathLength += distance;

    if (path)
      path->push_back(robotLocation);

    if (headings)
      headings->push_back(robotHeading);

    if (calculator.isFinished())
      return MotionController::PathEndReason::FINISH;
  }

  return MotionController::PathEndReason::OK;
}

void MotionController::odometryCallback(nav_msgs::OdometryPtr msg) {
  odometry_ = msg;
}

void MotionController::waypointsCallback(scitos_common::Vec2Array msg) {
  controlCalculator_.waypoints_.clear();
  controlCalculator_.waypointIndex_ = 0;

  controlCalculator_.waypoints_.reserve(std::min(msg.x.size(), msg.y.size()));
  for (size_t i = 0; i < std::min(msg.x.size(), msg.y.size()); i++) {
    controlCalculator_.waypoints_.emplace_back(msg.x[i], msg.y[i]);
  }
}

void MotionController::obstaclesCallback(scitos_common::Vec2Array msg) {
  controlCalculator_.obstacles_.clear();

  controlCalculator_.obstacles_.reserve(std::min(msg.x.size(), msg.y.size()));
  for (size_t i = 0; i < std::min(msg.x.size(), msg.y.size()); i++) {
    controlCalculator_.obstacles_.emplace_back(msg.x[i], msg.y[i]);
  }
}

void MotionController::publishWaypoints() const {
  visualization_msgs::MarkerArray markers;

  for (size_t i = 0; i < controlCalculator_.waypoints_.size(); i++) {
    auto p = controlCalculator_.waypoints_.at(i);
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.a = 1.0;
    marker.color.r = i >= controlCalculator_.waypointIndex_ ? 1.0 : 0.0;
    marker.color.g = i <= controlCalculator_.waypointIndex_ ? 1.0 : 0.0;
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

void MotionController::publishPath(const std::vector<Vec2<float>>& path, const std::vector<float>& headings) const {
  visualization_msgs::MarkerArray markers;

  for (size_t i = 0; i < path.size(); i++) {
    auto p = path.at(i);
    tf2::Quaternion heading;
    heading.setRPY(0, 0, headings.at(i));

    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.pose.orientation = tf2::toMsg(heading);
    marker.pose.position.x = p.x;
    marker.pose.position.y = p.y;
    marker.pose.position.z = 0.05;
    marker.id = i;
    markers.markers.push_back(marker);
  }

  pathPub_.publish(markers);
}
