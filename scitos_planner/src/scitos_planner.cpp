
#include <ros/package.h>
#include <utility>
#include <vector>
#include <random>

#include "scitos_common/LineArray.h"
#include "scitos_common/map/line.hpp"
#include "scitos_common/vec2.hpp"
#include "scitos_planner/scitos_planner.hpp"

Planner::Planner(ros::NodeHandle nh) : nh_{nh} {
    mapSub_ =
      nh_.subscribe("/map", 1, &Planner::mapCallback, this);
    // TODO: Sub for goal and position (odom)
    // TODO: Params for n and d (amount of vertices and edge length)
}

void Planner::step(const ros::TimerEvent &event) {
  uint32_t m = 0;
  struct Node {
    Vec2<float> loc;
    Node* from;
    float cost;
  };
  std::vector<Node> nodes;
  const auto [min, max] = map_.findBounds();
  static std::default_random_engine e;
  static std::uniform_real_distribution<float> disX(min.x, max.x);
  static std::uniform_real_distribution<float> disY(min.y, max.y);
  while (m < n_) {
    Vec2<float> qRand{disX(e), disY(e)};
    Node qNear; // TODO: Find nearest vertex from vertices and also update costs if needed
    Vec2<float> qNew = (qRand - qNear.loc) * d_;
    nodes.push_back({qNew, &qNear, qNear.cost + d_});
    
    if ((qNew - goal_).length() < endThreshold_) {
      break;
    }
    m++;
  }
}

void Planner::mapCallback(scitos_common::LineArray msg) {
  std::vector<scitos_common::map::Line<float>> lines;
  lines.reserve(msg.lines.size());
  for (const auto l : msg.lines) {
      lines.push_back({{l.x1, l.y1}, {l.x2, l.y2}});
  }
  map_.loadFromLines(lines);
}
