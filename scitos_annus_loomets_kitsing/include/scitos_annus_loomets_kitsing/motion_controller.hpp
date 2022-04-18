#pragma once
#include <chrono>
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "scitos_common/pid.hpp"
#include "scitos_common/vec2.hpp"
#include "scitos_common/polar2.hpp"

#include "geometry_msgs/PoseArray.h"

class MotionController {
public:
  enum PathEndReason {UNKNOWN, SUCCESS, FINISH, WALL};

  explicit MotionController(ros::NodeHandle nh);
  void step(const ros::TimerEvent& event);


private:
  class ControlCalculator {
    public:
    std::vector<Vec2<float>> waypoints_;
    uint32_t waypointIndex_ = 0;
    float pointMargin_ = 0.1f;

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
      error.theta = Util::normalize_angle(error.theta - heading);

      if (errorMsg) {
        *errorMsg = error.toMsg();
      }

      float pidOutAngular = trajectoryPidAng_.accumulate(error.theta, timeStep);
      float pidOutLinear = std::max(std::min(trajectoryPidDist_.accumulate(error.r, timeStep), 0.7f), -0.7f);
      return {pidOutLinear, pidOutAngular};
    }
  };

  ros::NodeHandle nh_;

  ControlCalculator controlCalculator_;

  ros::Subscriber odometrySub_;
  ros::Subscriber waypointsSub_;

  ros::Publisher waypointsPub_;
  ros::Publisher controlPub_;
  ros::Publisher errorPub_;
  ros::Publisher pathPub_;

  ros::Timer mainTimer_;

  nav_msgs::OdometryPtr odometry_;

  void odometryCallback(nav_msgs::OdometryPtr msg);
  void waypointsCallback(geometry_msgs::PoseArray msg);
  void publishWaypoints() const;
  void publishPath(const std::vector<Vec2<float>>& path, const std::vector<float>& headings) const;
  PathEndReason simulateFuturePath(const int steps, std::vector<Vec2<float>>* path=nullptr, std::vector<float>* headings=nullptr) const;
};
