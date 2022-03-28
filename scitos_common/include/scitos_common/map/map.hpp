#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <optional>
#include <tuple>
#include <vector>

#include "scitos_common/dbscan.hpp"
#include "scitos_common/map/line.hpp"
#include "scitos_common/polar2.hpp"
#include "scitos_common/vec2.hpp"

namespace scitos_common::map {

template <typename T> class Map {
public:
  Map() = default;
  Map(float padding, float fadePower)
      : padding_{padding}, fadePower_{fadePower} {}

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
    for (size_t i = 0; i < merged.size(); i++) {
      auto line = merged.at(i);
      Vec2<T> p1 = line.p1;
      Vec2<T> p2 = line.p2;
      Line<T> regLine = line;
      for (size_t j = i + 1; j < merged.size(); j++) {

        // TODO: Merge also lines from farther away that have low conf?
        auto newLine = merged.at(j);
        if (line.perpendicularDistance(newLine) < padding_ &&
            // line.perpendicularDistance(newLine.p2) < padding_ &&
            line.overlaps2(newLine, 0.f) &&
            line.v().angle_nodir(newLine.v()) < 0.6) {
          regLine = regLine.merge(newLine);
          auto [a, b] = maxDist({p1, p2, newLine.p1, newLine.p2});
          p1 = a;
          p2 = b;
          toAdd[j] = false;
        }
      }
      if (regLine.confidence > 0.05 && toAdd.at(i) && regLine.length() > 0.04) {
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
  void prune(std::vector<Line<T>> lines, const Vec2<T> &loc, const Vec2<T> &l,
             const Vec2<T> &r) {
    if (lines_.size() < 1) {
      return;
    }

    // Add padding to lines
    for (size_t i = 0; i < lines.size(); i++) {
      lines[i].p1 =
          lines[i].p1 + (lines[i].p1 - lines[i].p2).normalize() * 0.05f;
      lines[i].p2 =
          lines[i].p2 + (lines[i].p2 - lines[i].p1).normalize() * 0.05f;
    }

    for (size_t i = 0; i < lines_.size(); i++) {
      auto line = lines_.at(i);
      bool p1InFov = l.cross(line.p1 - loc) > 0 && (line.p1 - loc).cross(r) > 0;
      bool p2InFov = l.cross(line.p2 - loc) > 0 && (line.p2 - loc).cross(r) > 0;

      bool p1Visible = (line.p1 - loc).length() < pruneDist_;
      bool p2Visible = (line.p2 - loc).length() < pruneDist_;

      for (const auto &ln : lines) {
        // Raycast
        p1Visible &= !ln.intersect({loc, line.p1}).has_value();
        p2Visible &= !ln.intersect({loc, line.p2}).has_value();
      }

      // Treat too parallel as not visible
      p1Visible &= (line.p1 - loc).angle_nodir(line.v()) > 0.6f;
      p2Visible &= (line.p2 - loc).angle_nodir(line.v()) > 0.6f;

      if (p1InFov && p1Visible) {
        // Check if p1 should be moved torwards p2
        lines_[i].p1 = line.p1 + (line.p2 - line.p1).normalize() * 0.05f;
        // lines_[i].confidence *= 0.99f;
      }
      if (p2InFov && p2Visible) {
        // Check if p2 should me moved torwards p1
        lines_[i].p2 = line.p2 + (line.p1 - line.p2).normalize() * 0.05f;
        // lines_[i].confidence *= 0.99f;
      }
    }
  }

  void align(int k) {
    if (lines_.size() < k * 20)
      return;

    auto headingDistance = [](const float a, const float b) {
      return std::fmod(abs(a - b), M_PI_2);
    };

    std::vector<float> angles;
    if (k > 1) {
      for (int i = 0; i < k; i++) {
        angles.push_back((i * M_PI_2) / (k - 1));
      }
    } else {
      angles.push_back(0);
    }

    std::vector<uint> angleIndices(lines_.size(), 0);

    for (int n = 0; n < 50; n++) {
      std::vector<std::pair<float, float>> angleSums(angles.size(),
                                                     {0.0f, 0.0f});
      std::vector<int> angleCounts(angles.size(), 0);
      for (uint lineIndex = 0; lineIndex < lines_.size(); lineIndex++) {
        auto line = lines_[lineIndex];
        const float heading = line.heading();
        float bestDistance = std::numeric_limits<float>::infinity();
        for (uint angleIndex = 0; angleIndex < angles.size(); angleIndex++) {
          float distance = headingDistance(heading, angles[angleIndex]);
          if (distance < bestDistance) {
            angleIndices[lineIndex] = angleIndex;
            bestDistance = distance;
          }
        }
        angleSums[angleIndices[lineIndex]].first += sin(2 * heading);
        angleSums[angleIndices[lineIndex]].second += cos(2 * heading);
        ++angleCounts[angleIndices[lineIndex]];
      }
      float maxChange = 0.0f;
      for (uint angleIndex = 0; angleIndex < angles.size(); angleIndex++) {
        float newAngle =
            atan2(angleSums[angleIndex].first / angleCounts[angleIndex],
                  angleSums[angleIndex].second / angleCounts[angleIndex]) /
            2.0f;
        maxChange = std::max(maxChange, abs(angles[angleIndex] - newAngle));
        angles[angleIndex] = newAngle;
      }
      if (maxChange <
          1.0f / 180.0f * M_PI) // Stop if improvement is less than 2 degrees
      {
        break;
      }
    }

    for (uint lineIndex = 0; lineIndex < lines_.size(); lineIndex++) {
      Line<T> &line = lines_[lineIndex];
      const Vec2<T> center = (line.p1 + line.p2) / 2.0f;
      ::Polar2<float> pointHeading(line.length() / 2.0f,
                                   angles[angleIndices[lineIndex]]);
      const Vec2<T> newP1 = center + pointHeading;
      const Vec2<T> newP2 = center + pointHeading.opposite();
      line.p1 = line.p1 * 0.95f + newP1 * 0.05f;
      line.p2 = line.p2 * 0.95f + newP2 * 0.05f;
    }
  }

  void combineCorners(const int n, const float r,
                      std::vector<std::pair<Vec2<T>, std::set<Vec2<T> *>>>
                          &cornerVisualization) {
    std::map<Vec2<T> *, Line<T> *> pointLines = getPointLinePointers();
    std::map<Vec2<T> *, Line<T> *> tmp;
    std::vector<Vec2<T> *> points;
    for (auto const &pointLine : pointLines) {
      if (pointLine.second->confidence < 0.5f)
        continue;
      tmp.insert(pointLine);
      points.push_back(pointLine.first);
    }
    pointLines = tmp;

    std::vector<std::vector<Vec2<T> *>> clusters =
        scitos_common::dbscan2<Vec2<T> *>(
            points, [](auto a, auto b) { return (*a - *b).length(); }, n, r);
    for (auto cluster : clusters) {
      if (cluster.size() < 2)
        continue;
      auto [a, b] = maxDist(cluster);
      if ((*a - *b).length() > r * 2)
        continue;
      std::set<Line<T> *> encounteredLines;
      std::set<Vec2<T> *> pointsToMove;
      Vec2<T> clusterCenter;
      float confidenceSum = 0;
      for (auto point : cluster) {
        Line<T> *line = pointLines.at(point);
        if (encounteredLines.erase(line)) {
          pointsToMove.erase(&line->p1);
          pointsToMove.erase(&line->p2);
          continue;
        }
        clusterCenter += (*point) * line->confidence;
        confidenceSum += line->confidence;
        if (line->length() > r) {
          encounteredLines.insert(line);
          pointsToMove.insert(point);
        }
      }
      if (pointsToMove.size() < 2)
        continue;

      clusterCenter = clusterCenter / confidenceSum;
      cornerVisualization.push_back(
          std::make_pair(clusterCenter, pointsToMove));
      for (auto pointToMove : pointsToMove) {
        if ((clusterCenter - *pointToMove).length() <= 2 * r) {
          *pointToMove = clusterCenter;
        }
      }
    }
  }

  /*!
   * Get visible lines
   *
   * @param loc - current location
   * @param l - left bound vector
   * @param r - right bound vector
   */
  std::vector<Line<T>> getVisibleLines(const Vec2<T> &loc) const {
    if (lines_.size() < 1) {
      return {};
    }

    std::vector<Line<T>> filtered;

    for (size_t i = 0; i < lines_.size(); i++) {
      auto line = lines_.at(i);
      bool p1Visible = true;
      bool p2Visible = true;

      for (const auto &ln : lines_) {
        // Raycast
        p1Visible &= !ln.padded(0.05f).intersect({loc, line.p1}).has_value();
        p2Visible &= !ln.padded(0.05f).intersect({loc, line.p2}).has_value();
      }

      if (p1Visible || p2Visible) {
        filtered.push_back(line);
      }
    }
    return filtered;
  }

  bool isVisible(const Vec2<T> &p, const Vec2<T> &loc) const {
    for (const auto &ln : lines_) {
      if (ln.padded(0.05f).intersect({loc, p - (p - loc).normalize() * 0.05f}).has_value()) {
        return false;
      }
    }
    return true;
  }

  std::vector<Line<T>> getLines() const { return lines_; }
  void loadFromLines(std::vector<Line<T>> lines) { lines_ = lines; }

private:
  float padding_;
  float pruneDist_ = 5.f;
  float fadePower_;
  std::vector<Line<T>> lines_;

  std::map<Vec2<T> *, Line<T> *> getPointLinePointers() {
    std::map<Vec2<T> *, Line<T> *> output;
    for (size_t i = 0; i < lines_.size(); i++) {
      output.insert(std::pair<Vec2<T> *, Line<T> *>(&lines_[i].p1, &lines_[i]));
      output.insert(std::pair<Vec2<T> *, Line<T> *>(&lines_[i].p2, &lines_[i]));
    }
    return output;
  }
  // Needs at least 2 points not to segfault
  std::tuple<Vec2<T>, Vec2<T>> maxDist(const std::vector<Vec2<T>> &points) {
    auto p1 = points.at(0);
    auto p2 = points.at(1);
    float dist = (p1 - p2).length();
    for (size_t i = 0; i < points.size(); i++) {
      auto a = points.at(i);
      for (size_t j = i + 1; j < points.size(); j++) {
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
  // Needs at least 2 points not to segfault
  std::tuple<Vec2<T> *, Vec2<T> *>
  maxDist(const std::vector<Vec2<T> *> &points) {
    auto p1 = points.at(0);
    auto p2 = points.at(1);
    float dist = (*p1 - *p2).length();
    for (size_t i = 0; i < points.size(); i++) {
      auto a = points.at(i);
      for (size_t j = i + 1; j < points.size(); j++) {
        auto b = points.at(j);
        if ((*a - *b).length() > dist) {
          p1 = a;
          p2 = b;
          dist = (*a - *b).length();
        }
      }
    }
    return std::make_tuple(p1, p2);
  }
};
} // namespace scitos_common::map
