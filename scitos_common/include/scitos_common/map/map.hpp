#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <optional>
#include <tuple>
#include <vector>

#include "scitos_common/map/line.hpp"
#include "scitos_common/vec2.hpp"

namespace scitos_common::map {

template <typename T> class Map {
public:
  Map() = default;
  Map(float padding, float fadePower)
      : padding_{padding}, fadePower_{fadePower} {}

  // TODO: Decide if this should be removed in favour of accumulate2
  void accumulate(const std::vector<Line<T>> &newLines) {
    std::vector<bool> merged(newLines.size(), false);

    for (int i = 0; i < lines_.size(); i++) {
      auto line = lines_.at(i);
      auto lHough = line.toHoughSpace();
      T xMin = std::min(line.p1.x, line.p2.x);
      T xMax = std::max(line.p1.x, line.p2.x);
      T yMin = std::min(line.p1.y, line.p2.y);
      T yMax = std::max(line.p1.y, line.p2.y);
      float confidence = line.confidence;
      auto updated = line.toHoughSpace(); // * confidence;
      for (size_t j = 0; j < newLines.size(); j++) {
        auto newLine = newLines.at(j);
        auto nHough = newLine.toHoughSpace();
        if (line.overlaps(newLine, padding_) &&
            std::atan(lHough.x - nHough.x) < 0.3f) {
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
  }

  /*!
   * Accumulate new lines onto map
   *
   * @param newLines - new lines to add
   */
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
      Vec2<T> p1 = line.p1;
      Vec2<T> p2 = line.p2;
      Line<T> regLine = line;
      for (size_t j = i + 1; j < merged.size(); j++) {

        // TODO: Merge also lines from farther away that have low conf?
        auto newLine = merged.at(j);
        if (line.perpendicularDistance(newLine.p1) < padding_ &&
            line.perpendicularDistance(newLine.p2) < padding_ &&
            line.overlaps(newLine, padding_)) {
          regLine = regLine.merge(newLine);
          auto [a, b] = maxDist({p1, p2, newLine.p1, newLine.p2});
          p1 = a;
          p2 = b;
          toAdd[j] = false;
        }
      }
      if (regLine.confidence > 0.05 && toAdd.at(i) && regLine.length() > 0.02) {
        tmpLines.push_back({regLine.projectInf(p1), regLine.projectInf(p2),
                            std::pow(regLine.confidence, fadePower_)});
      }
    }
    lines_ = tmpLines;
  }

  /*!
   * Prune lines that do not exsist and line endings that extend too far
   *
   * @param lines - lines from latest measurement
   * @param loc - current location
   * @param l - left bound vector
   * @param r - right bound vector
   */
  void prune(const std::vector<Line<T>> &lines, const Vec2<T> &loc,
             const Vec2<T> &l, const Vec2<T> &r) {
    if (lines_.size() < 1) {
      return;
    }
    for (size_t i = 0; i < lines_.size(); i++) {
      auto line = lines_.at(i);
      bool p1InFov = l.cross(line.p1 - loc) > 0 && (line.p1 - loc).cross(r) > 0;
      bool p2InFov = l.cross(line.p2 - loc) > 0 && (line.p2 - loc).cross(r) > 0;

      bool p1Visible = true;
      bool p2Visible = true;

      // BUG: Visibility check seems to fail
      for (const auto &ln : lines) {
        // Raycast
        p1Visible &= !ln.intersect({loc, line.p1}).has_value();
        p2Visible &= !ln.intersect({loc, line.p2}).has_value();
      }

      if (p1InFov && p1Visible) {
        // Check if p1 should be moved torwards p2
        lines_[i].p1 = line.p1 + (line.p2 - line.p1).normalize() * 0.03f;
      }
      if (p2InFov && p2Visible) {
        // Check if p2 should me moved torwards p1
        lines_[i].p2 = line.p2 + (line.p1 - line.p2).normalize() * 0.03f;
      }
    }
  }

  std::vector<Line<T>> getLines() const { return lines_; }

private:
  float fadePower_;
  float padding_;
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
