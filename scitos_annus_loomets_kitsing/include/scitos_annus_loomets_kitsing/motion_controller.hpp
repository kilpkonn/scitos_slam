#pragma once
#include <chrono>
#include <vector>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include "scitos_common/Vec2Array.h"
#include "scitos_common/pid.hpp"
#include "scitos_common/polar2.hpp"
#include "scitos_common/vec2.hpp"

#include "geometry_msgs/PoseArray.h"

class MotionController {
public:
  enum PathEndReason {UNKNOWN, OK, FINISH, OBSTACLES};

  explicit MotionController(ros::NodeHandle nh);
  void step(const ros::TimerEvent &event);

private:
  class ControlCalculator {
    public:
    std::vector<Vec2<float>> waypoints_;
    std::vector<Vec2<float>> obstacles_;
    uint32_t waypointIndex_ = 0;
    float pointMargin_ = 0.1f;
    float maxAngleToDrive_ = 0.2f;
    float maxSpeed_ = 0.5f;
    float maxAngle_ = 0.7f;
    const float minObstacleDistance_ = 0.75f;

    PID<float> trajectoryPidDist_;
    PID<float> trajectoryPidAng_;

    bool isFinished() const {
      return waypointIndex_ >= waypoints_.size();
    }

    Polar2<float> calculateControl(Vec2<float> robotLocation, float heading, std::chrono::milliseconds timeStep, ::scitos_common::Polar2* errorMsg = nullptr){
      for (size_t i = waypointIndex_; i < waypoints_.size(); i++) {
        if (waypoints_.at(i) - robotLocation < pointMargin_) {
          waypointIndex_ = i + 1;
        }
      }

      if (isFinished()) {
        return {0.0f, 0.0f};
      }

      const Vec2<float> target = waypoints_.at(waypointIndex_);
      Polar2<float> error = target - robotLocation;
      error.theta = util::normalize_angle(error.theta - heading);

      if (errorMsg) {
        *errorMsg = error.toMsg();
      }

      float pidOutAngular = trajectoryPidAng_.accumulate(error.theta, timeStep);
      float pidOutLinear = std::max(std::min(trajectoryPidDist_.accumulate(error.r, timeStep), 0.7f), -0.7f);

      if (abs(error.theta) > M_PI / 16) {
        pidOutLinear = std::clamp(pidOutLinear, -0.5f, 0.5f);
      }
      pidOutAngular = std::clamp(pidOutAngular, -0.4f, 0.4f);
      return {pidOutLinear, pidOutAngular};
    }

    bool isObstacleNearby(const Vec2<float> robotLocation) const {
      for (Vec2<float> obstacle: obstacles_) {
        if (robotLocation - obstacle < minObstacleDistance_)
          return true;
      }
      return false;
    }
  };

  ros::NodeHandle nh_;

  ControlCalculator controlCalculator_;

  ros::Subscriber odometrySub_;
  ros::Subscriber waypointsSub_;
  ros::Subscriber obstaclesSub_;

  ros::Publisher waypointsPub_;
  ros::Publisher controlPub_;
  ros::Publisher errorPub_;
  ros::Publisher pathPub_;

  ros::Timer mainTimer_;

  nav_msgs::OdometryPtr odometry_;

  void odometryCallback(nav_msgs::OdometryPtr msg);
  void waypointsCallback(scitos_common::Vec2Array msg);
  void obstaclesCallback(scitos_common::Vec2Array msg);
  void publishWaypoints() const;
  void publishPath(const std::vector<Vec2<float>>& path, const std::vector<float>& headings) const;
  PathEndReason simulateFuturePath(const int steps, float& pathLength, std::vector<Vec2<float>>* path=nullptr, std::vector<float>* headings=nullptr) const;
};
