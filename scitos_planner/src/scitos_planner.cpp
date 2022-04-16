
#include <ros/package.h>
#include <utility>
#include <vector>

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
  std::vector<Vec2<float>> vertices;
  while (m < n_) {
    Vec2<float> qRand; // TODO: Random withing map
    Vec2<float> qNear; // TODO: Find nearest vertex from vertices
    Vec2<float> qNew;  // TODO: Find point at most d_ away from qNear (pretty much clamp)
    // TODO: Add to graph
    
    // TODO: Check if close enough to goal (or maybe if straight line to goal can be used)
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
