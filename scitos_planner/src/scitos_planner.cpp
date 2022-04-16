
#include <ros/package.h>
#include <utility>
#include <vector>

#include "scitos_common/LineArray.h"
#include "scitos_common/map/line.hpp"
#include "scitos_planner/scitos_planner.hpp"

Planner::Planner(ros::NodeHandle nh) : nh_{nh} {
    mapSub_ =
      nh_.subscribe("/map", 1, &Planner::mapCallback, this);

}

void Planner::step(const ros::TimerEvent &event) {}

void Planner::mapCallback(scitos_common::LineArray msg) {
  std::vector<scitos_common::map::Line<float>> lines;
  lines.reserve(msg.lines.size());
  for (const auto l : msg.lines) {
      lines.push_back({{l.x1, l.y1}, {l.x2, l.y2}});
  }
  map_.loadFromLines(lines);
}
