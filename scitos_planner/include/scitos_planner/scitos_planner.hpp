#pragma once

#include <cstdint>
#include <vector>

#include <ros/ros.h>

#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "scitos_common/LineArray.h"

#include "scitos_common/map/map.hpp"
#include "scitos_common/vec2.hpp"

class Planner {
public:
  explicit Planner(ros::NodeHandle nh);
  void step(const ros::TimerEvent &event);

private:
  ros::NodeHandle nh_;

  ros::Subscriber mapSub_;
  ros::Subscriber odomSub_;
  ros::Subscriber goalSub_;

  ros::Publisher rrtPub_;


  uint32_t n_ = 2000;
  float d_ = 0.1f;
  float endThreshold_ = 0.2f;
  float padding_ = 0.5f;
  Vec2<float> goal_{0.f, 0.f};
  nav_msgs::Odometry odometry_;
  scitos_common::map::Map<float> map_;
  
  struct Node {
    Vec2<float> loc;
    Node *from;
    float cost;
  };
  std::vector<Node> nodes_;

  void mapCallback(scitos_common::LineArray msg);
  void odomCallback(nav_msgs::Odometry msg);
  void goalCallback(geometry_msgs::PoseStamped msg);

  void publishRRT();
};
