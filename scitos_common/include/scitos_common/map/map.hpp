#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <tuple>
#include <vector>

#include "scitos_common/map/line.hpp"
#include "scitos_common/vec2.hpp"

namespace scitos_common::map {

template <typename T> class Map {
public:
  Map() = default;
  Map(float mergeThreshold) : mergeThreshold_{mergeThreshold} {}
  void accumulate(const std::vector<Line<T>> &newLines) {
    std::vector<bool> merged(newLines.size(), false);

    for (int i = 0; i < lines_.size(); i++) {
      auto line = lines_.at(i);
      auto lHough = line.toHoughSpace();
      T xMin = std::min(line.p1.x, line.p2.x);
      T xMax = std::max(line.p1.x, line.p2.x);
      T yMin = std::min(line.p1.y, line.p2.y);
      T yMax = std::max(line.p1.y, line.p2.y);
      float confidence =
          line.confidence; // TODO: Bayesian probability: p1 + p2 - p1 * p2
      auto updated = line.toHoughSpace(); // * confidence;
      for (size_t j = 0; j < newLines.size(); j++) {
        auto newLine = newLines.at(j);
        auto nHough = newLine.toHoughSpace();
        if (line.overlaps(newLine, padding_) &&
            std::atan(lHough.x - nHough.x) < mergeThreshold_) {
          updated = nHough; /*   * newLine.confidence; */
          confidence += newLine.confidence;
          xMin = std::min(xMin, std::min(newLine.p1.x, newLine.p2.x));
          xMax = std::max(xMax, std::max(newLine.p1.x, newLine.p2.x));
          yMin = std::min(yMin, std::min(newLine.p1.y, newLine.p2.y));
          yMax = std::max(yMax, std::max(newLine.p1.y, newLine.p2.y));
          merged[j] = true;
        }
      }
      // updated = updated / confidence;
      Line<T> updatedLine(
          {xMin, std::clamp(updated.y + xMin * updated.x, yMin, yMax)},
          {xMax, std::clamp(updated.y + xMax * updated.x, yMin, yMax)},
          confidence);
      lines_[i] = updatedLine;
    }

    for (size_t i = 0; i < newLines.size(); i++) {
      if (!merged.at(i)) {
        lines_.push_back(newLines.at(i));
      }
    }
    // TODO: Lines in self should be merged as well
  }

  void accumulate2(const std::vector<Line<T>> &newLines) {
    std::vector<Line<T>> merged = lines_;
    merged.reserve(merged.size() + newLines.size());
    merged.insert(merged.end(), newLines.begin(), newLines.end());
    std::vector<bool> toAdd(merged.size(), true);

    // Merge exsisting
    std::vector<Line<T>> tmpLines;
    tmpLines.reserve(merged.size());
    for (int i = 0; i < merged.size(); i++) {
      auto line = merged.at(i);
      // float confidence = line.confidence;
      Vec2<T> p1 = line.p1;
      Vec2<T> p2 = line.p2;
      Line<T> regLine = line;
      for (size_t j = i + 1; j < merged.size(); j++) {
        // if (i == j) continue;

        // TODO: Merge also lines from farther away that have low conf
        auto newLine = merged.at(j);
        if (line.perpendicularDistance(newLine.p1) < padding_ &&
            line.perpendicularDistance(newLine.p2) < padding_ &&
            line.overlaps(newLine, padding_)) {
          auto np1Closer = (newLine.p1 - regLine.p1).length() <
                           (newLine.p2 - regLine.p1).length();
          auto np1 = np1Closer ? newLine.p1 : newLine.p2;
          auto np2 = !np1Closer ? newLine.p1 : newLine.p2;
          regLine = {
              (regLine.p1 * regLine.confidence + np1 * newLine.confidence) /
                  (regLine.confidence + newLine.confidence),
              (regLine.p2 * regLine.confidence + np2 * newLine.confidence) /
                  (regLine.confidence + newLine.confidence),
              regLine.confidence + newLine.confidence -
                  regLine.confidence * newLine.confidence};

          auto [a, b] = maxDist({p1, p2, newLine.p1, newLine.p2});
          p1 = a;
          p2 = b;
          toAdd[j] = false;
          // confidence =
          //     confidence + newLine.confidence - confidence *
          //     newLine.confidence;
        }
      }
      if (regLine.confidence > 0.05 && toAdd.at(i)) {
        // ROS_INFO("conf: %f", regLine.confidence);
        // tmpLines.push_back({p1, p2, confidence * confFade});
        tmpLines.push_back({regLine.projectInf(p1), regLine.projectInf(p2),
                            regLine.confidence * confFade});
      }
    }
    lines_ = tmpLines;
  }

  std::vector<Line<T>> getLines() const { return lines_; }

private:
  float mergeThreshold_;
  float confFade = 0.998f;
  float padding_ = 0.16f; // TODO: Move to params and reasonable value
  std::vector<Line<T>> lines_;

  // Needs at least 2 points not to segfault
  std::tuple<Vec2<T>, Vec2<T>> maxDist(const std::vector<Vec2<T>> &points) {
    auto p1 = points.at(0);
    auto p2 = points.at(1);
    float dist = (p1 - p2).length();
    for (size_t i = 0; i < points.size(); i++) {
      auto a = points.at(i);
      for (size_t j = 0; j < points.size(); j++) {
        auto b = points.at(j);
        if ((a - b).length() > dist) {
          p1 = a;
          p2 = b;
          dist = (a - b).length();
        }
      }
    }
    return std::make_tuple(p1, p2);
  }
};
} // namespace scitos_common::map
