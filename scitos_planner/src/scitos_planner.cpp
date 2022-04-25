
#include <algorithm>
#include <limits>
#include <random>
#include <ros/package.h>
#include <utility>
#include <vector>

#include "scitos_common/LineArray.h"
#include "scitos_common/Vec2Array.h"
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
  waypointsPub_ = nh_.advertise<scitos_common::Vec2Array>("/path", 10);

  n_ = nh_.param("/planner/n", 1000);
  d_ = nh_.param("/planner/d", 0.1);
  endThreshold_ = nh_.param("/planner/end_threshold", 0.5);
  padding_ = nh_.param("/planner/padding", 0.5);
}

void Planner::step(const ros::TimerEvent &event) {
  refreshWaypoints();
  if (!needsUpdate_) {
    publishRRT();
    publishWaypoints();
    return;
  }
  ROS_INFO("Step");
  ROS_INFO("Goal: (%f, %f)", goal_.x, goal_.y);
  nodes_.clear();
  waypoints_.clear();
  nodes_.reserve(n_);
  lastNode_ = nullptr;
  goalAchieved_ = false;
  nodes_.push_back({{static_cast<float>(odometry_.pose.pose.position.x),
                     static_cast<float>(odometry_.pose.pose.position.y)},
                    nullptr});
  const auto [min, max] = map_.findBounds();
  static std::default_random_engine e;
  std::uniform_real_distribution<float> disX(min.x, max.x);
  std::uniform_real_distribution<float> disY(min.y, max.y);
  for (uint32_t m = 0; m < n_; m++) {
    Vec2<float> qRand{disX(e), disY(e)};
    float dist = std::numeric_limits<float>::max();

    Node *qNear = nullptr;
    // look for nenum classrest node
    for (size_t i = 0; i < nodes_.size(); i++) {
      if ((nodes_[i].loc - qRand).dot() < dist) {
        qNear = &nodes_[i];
        dist = (nodes_[i].loc - qRand).dot();
      }
    }

    if (qNear != nullptr) {
      Vec2<float> qNew = qNear->loc + (qRand - qNear->loc).normalize() * d_;
      // TODO: Handle case wehere robot already is too close to wall
      if (!map_.isClearPath({qNear->loc, qNew}, padding_)) {
        continue;
      }
      nodes_.push_back({qNew, qNear});
      if ((qNew - goal_).length() < endThreshold_) {
        nodes_.push_back({goal_, &nodes_[nodes_.size() - 1]});
        lastNode_ = &nodes_[nodes_.size() - 1];
        needsUpdate_ = false;
        break;
      }
    }
  }
  refreshWaypoints();

  ROS_INFO("Step Done");

  publishRRT();
  publishWaypoints();
}

void Planner::refreshWaypoints() {
  if (lastNode_ == nullptr || goalAchieved_)
    return;

  Vec2<float> loc{static_cast<float>(odometry_.pose.pose.position.x),
                  static_cast<float>(odometry_.pose.pose.position.y)};

  const auto lastIdx = waypoints_.size();
  waypoints_.clear();

  if ((loc - lastNode_->loc).length() < endThreshold_) {
    ROS_INFO("Path completed!");
    goalAchieved_ = true;
    return;
  }

  Node *node = lastNode_;
  while (node->from != nullptr) {
    waypoints_.push_back(node->loc);
    if (map_.isClearPath({loc, node->loc}, padding_)) {
      break;
    }
    node = node->from;
  }

  // Too dangerous to continue, replan
  if (lastIdx > 0 && waypoints_.size() > lastIdx + 1) {
    ROS_INFO("Replannig path...");
    needsUpdate_ = true;
    goalAchieved_ = false;
  }
  std::reverse(waypoints_.begin(), waypoints_.end());
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
  ROS_INFO("New goal!");
  needsUpdate_ = true;
  goalAchieved_ = false;
}

void Planner::publishRRT() const {
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker m;
  m.action = visualization_msgs::Marker::DELETEALL;
  markers.markers.push_back(m);

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

void Planner::publishWaypoints() const {
  scitos_common::Vec2Array msg;
  msg.header.stamp = ros::Time::now();

  for (const auto &wp : waypoints_) {
    msg.x.push_back(wp.x);
    msg.y.push_back(wp.y);
  }

  waypointsPub_.publish(msg);
}
