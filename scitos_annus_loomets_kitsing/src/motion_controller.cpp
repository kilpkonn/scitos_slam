
#include "scitos_annus_loomets_kitsing/motion_controller.hpp"
#include "XmlRpcValue.h"
#include "scitos_common/pid.hpp"
#include "scitos_common/vec2.hpp"
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/MarkerArray.h>
MotionController::MotionController(ros::NodeHandle nh) : nh_{nh}, rate_{100} {

  XmlRpc::XmlRpcValue waypoints;
  if (nh_.getParam("mission.waypoints", waypoints)) {
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
  pointMargin_ = nh_.param("mission.distance_margin", 0.1f);
  // TODO: Move these params to yaml
  trayectoryPid_ = PID<Vec2<float>>(0.1, 0.f, 0.f, 100.f, 0.5f);

  waypointsPub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "/mission_control/waypoints", 10);

  // TODO: Sub to localization etc.

  mainTimer_ = nh_.createTimer(rate_, &MotionController::step, this);
}


void MotionController::step(const ros::TimerEvent& event) {
  // TODO: step pid, send out control and logging etc.
}
