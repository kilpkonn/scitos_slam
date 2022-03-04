#pragma once

#include <algorithm>
#include <cstddef>
#include <vector>

#include "scitos_common/map/line.hpp"

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
      float confidence =
          line.confidence; // TODO: Bayesian probability: p1 + p2 - p1 * p2
      auto updated = line.toHoughSpace() * confidence;
      for (size_t i = 0; i < newLines.size(); i++) {
        auto newLine = newLines.at(i);
        auto nHough = newLine.toHoughSpace();
        if (line.overlaps(newLine, padding_) &&
            std::abs(lHough.x - nHough.x) < mergeThreshold_) {
          updated += nHough * newLine.confidence;
          confidence += newLine.confidence;
          xMin = std::min(xMin, std::min(newLine.p1.x, newLine.p2.x));
          xMax = std::max(xMax, std::max(newLine.p1.x, newLine.p2.x));
          merged[i] = merged[i] || true;
        }
      }
      updated = updated / confidence;
      Line<T> updatedLine({xMin, updated.y + xMin * updated.x},
                          {xMax, updated.y + xMax * updated.x}, confidence);
      lines_[i] = updatedLine;
    }
    for (size_t i = 0; i < newLines.size(); i++) {
      if (!merged.at(i)) {
        lines_.push_back(newLines.at(i));
      }
    }
    // TODO: Lines in self should be merged as well
  }

  std::vector<Line<T>> getLines() const { return lines_; }

private:
  float mergeThreshold_;
  float padding_ = 0.f; // TODO: Move to params and reasonable value
  std::vector<Line<T>> lines_;
};
} // namespace scitos_common::map
