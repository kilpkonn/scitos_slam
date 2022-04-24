
#include <algorithm>
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
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"

Planner::Planner(ros::NodeHandle nh) : nh_{nh} {
  mapSub_ = nh_.subscribe("/map", 1, &Planner::mapCallback, this);
  odomSub_ = nh_.subscribe("/ekf_odom", 1, &Planner::odomCallback, this);
  goalSub_ =
      nh_.subscribe("/move_base_simple/goal", 1, &Planner::goalCallback, this);
  rrtPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/debug/rrt", 3);
  waypointsPub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "/mission_control/waypoints", 10);
  drivingWaypointsPub_ = nh_.advertise<geometry_msgs::PoseArray>(
      "/waypoints", 10);

  n_ = nh_.param("/planner/n", 1000);
  d_ = nh_.param("/planner/d", 0.1);
  endThreshold_ = nh_.param("/planner/end_threshold", 0.5);
  padding_ = nh_.param("/planner/padding", 0.5);
}

void Planner::step(const ros::TimerEvent &event) {
  if (!needsUpdate_) {
    publishRRT();
    refreshWaypoints();
    //publishWaypoints();
    return;
  }
  ROS_INFO("Step");
  ROS_INFO("Goal: (%f, %f)", goal_.x, goal_.y);
  nodes_.clear();
  nodes_.reserve(n_);
  lastNode_ = nullptr;
  nodes_.push_back({{static_cast<float>(odometry_.pose.pose.position.x),
                     static_cast<float>(odometry_.pose.pose.position.y)},
                    nullptr});
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
      if ((nodes_[i].loc - qRand).dot() < dist) {
        qNear = &nodes_[i];
        dist = (nodes_[i].loc - qRand).dot();
      }
    }

    if (qNear != nullptr) {
      Vec2<float> qNew = qNear->loc + (qRand - qNear->loc).normalize() * d_;
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
      m++;
    }
  }
  refreshWaypoints();

  ROS_INFO("Step Done");

  publishRRT();
  publishWaypoints();
}

void Planner::refreshWaypoints() {
  if (lastNode_ == nullptr)
    return;

  Vec2<float> loc{static_cast<float>(odometry_.pose.pose.position.x),
                  static_cast<float>(odometry_.pose.pose.position.y)};

  waypoints_.clear();
  Node *node = lastNode_;
  while (node->from != nullptr) {
    waypoints_.push_back(node->loc);
    if (map_.isClearPath({loc, node->loc}, padding_)) {
      break;
    }
    node = node->from;
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
  needsUpdate_ = true;
}

void Planner::publishRRT() const {
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

void Planner::publishWaypoints() const {
  visualization_msgs::MarkerArray markers;
  geometry_msgs::PoseArray waypoints;

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
    marker.color.r = 1.0;
    marker.color.g = i == 0 ? 1.0 : 0.0;
    marker.color.b = 0.0;

    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = p.x;
    pose.position.y = p.y;
    pose.position.z = 0.05;

    marker.pose = pose;
    marker.id = i;
    markers.markers.push_back(marker);

    waypoints.poses.push_back(pose);
  }

  //waypointsPub_.publish(markers);
  drivingWaypointsPub_.publish(waypoints);
}
