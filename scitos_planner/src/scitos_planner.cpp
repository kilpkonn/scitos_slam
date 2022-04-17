
#include <random>
#include <ros/package.h>
#include <utility>
#include <vector>

#include "scitos_common/LineArray.h"
#include "scitos_common/map/line.hpp"
#include "scitos_common/vec2.hpp"
#include "scitos_planner/scitos_planner.hpp"

Planner::Planner(ros::NodeHandle nh) : nh_{nh} {
  mapSub_ = nh_.subscribe("/map", 1, &Planner::mapCallback, this);
  // TODO: Sub for goal and position (odom)
  // TODO: Params for n and d (amount of vertices and edge length)
}

void Planner::step(const ros::TimerEvent &event) {
  uint32_t m = 0;
  const auto [min, max] = map_.findBounds();
  static std::default_random_engine e;
  static std::uniform_real_distribution<float> disX(min.x, max.x);
  static std::uniform_real_distribution<float> disY(min.y, max.y);
  while (m < n_) {
    Vec2<float> qRand{disX(e), disY(e)};
    float dis = 0;

    Node *qNear;
    // look for nearest node
    for (size_t i = 0; i < nodes_.size(); i++) {
      const auto end = nodes_[i].loc + (nodes_[i].loc - qRand).normalize() * d_;
      if ((nodes_[i].loc - qRand).length() < dis &&
          map_.isClearPath({nodes_[i].loc, end}, padding_)) {
        qNear = &nodes_[i];
        dis = (nodes_[i].loc - qRand).length();
      }
    }

    // TODO check obstcle - if obstacle between then no go?

    if (qNear) {
      Vec2<float> qNew = (qRand - qNear->loc).normalize() * d_;
      nodes_.push_back({qNew, qNear, qNear->cost + d_});

      if ((qNew - goal_).length() < endThreshold_) {
        break;
      }
      m++;
    }
  }
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
}

void Planner::publishRRT() {
  // TODO:
}
