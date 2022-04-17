
#include <limits>
#include <random>
#include <ros/package.h>
#include <utility>
#include <vector>

#include "scitos_common/LineArray.h"
#include "scitos_common/map/line.hpp"
#include "scitos_common/vec2.hpp"
#include "scitos_planner/scitos_planner.hpp"
#include "visualization_msgs/MarkerArray.h"

Planner::Planner(ros::NodeHandle nh) : nh_{nh} {
  mapSub_ = nh_.subscribe("/map", 1, &Planner::mapCallback, this);
  odomSub_ = nh_.subscribe("/ekf_odom", 1, &Planner::odomCallback, this);
  goalSub_ =
      nh_.subscribe("/move_base_simple/goal", 1, &Planner::goalCallback, this);
  rrtPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/debug/rrt", 3);
  // TODO: Params for n and d (amount of vertices and edge length)
}

void Planner::step(const ros::TimerEvent &event) {
  if (!needsUpdate_) {
    publishRRT();
    return;
  }
  ROS_INFO("Step");
  ROS_INFO("Goal: (%f, %f)", goal_.x, goal_.y);
  nodes_.clear();
  nodes_.reserve(n_);
  nodes_.push_back({{static_cast<float>(odometry_.pose.pose.position.x),
                     static_cast<float>(odometry_.pose.pose.position.y)},
                    nullptr,
                    0.0});
  uint32_t m = 0;
  const auto [min, max] = map_.findBounds();
  static std::default_random_engine e;
  std::uniform_real_distribution<float> disX(min.x, max.x);
  std::uniform_real_distribution<float> disY(min.y, max.y);
  while (m < n_) {
    Vec2<float> qRand{disX(e), disY(e)};
    float dist = std::numeric_limits<float>::max();

    Node *qNear = nullptr;
    // look for nenum classrest node
    for (size_t i = 0; i < nodes_.size(); i++) {
      Vec2<float> qNew = nodes_[i].loc + (qRand - nodes_[i].loc).normalize() * d_;
      if ((nodes_[i].loc - qRand).dot() < dist &&
          map_.isClearPath({nodes_[i].loc, qNew}, padding_)) {
        // BUG: Is clear seems broken
        qNear = &nodes_[i];
        dist = (nodes_[i].loc - qRand).dot();
      }
    }

    if (qNear != nullptr) {
      Vec2<float> qNew = qNear->loc + (qRand - qNear->loc).normalize() * d_;
      nodes_.push_back({qNew, qNear, qNear->cost + d_});
      if ((qNew - goal_).length() < endThreshold_) {
        break;
      }
      m++;
    }
  }
  ROS_INFO("Step Done");
  needsUpdate_ = false;
  publishRRT();
}

void Planner::mapCallback(scitos_common::LineArray msg) {
  std::vector<scitos_common::map::Line<float>> lines;
  lines.reserve(msg.lines.size());
  for (const auto l : msg.lines) {
    lines.push_back({{l.x1, l.y1}, {l.x2, l.y2}});
  }
  map_.loadFromLines(lines);
}

void Planner::odomCallback(nav_msgs::Odometry msg) { odometry_ = msg; }

void Planner::goalCallback(geometry_msgs::PoseStamped msg) {
  goal_ = {static_cast<float>(msg.pose.position.x),
           static_cast<float>(msg.pose.position.y)};
  needsUpdate_ = true;
}

void Planner::publishRRT() {
  visualization_msgs::MarkerArray markers;
  int i = 0;
  for (auto node : nodes_) {
    if (node.from == nullptr) {
      continue;
    }
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.8;
    marker.color.b = 0.1;
    marker.pose.orientation.w = 1.0;

    geometry_msgs::Point p;
    p.z = 0.05;
    p.x = node.loc.x;
    p.y = node.loc.y;
    marker.points.push_back(p);
    p.x = node.from->loc.x;
    p.y = node.from->loc.y;
    marker.points.push_back(p);

    marker.id = i++;
    markers.markers.push_back(marker);
  }
  rrtPub_.publish(markers);
}
