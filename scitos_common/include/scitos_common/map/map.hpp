#pragma once

#include <vector>

#include "scitos_common/map/line.hpp"

namespace scitos_common::map {

template <typename T> class Map {
public:
  void accumulate(const std::vector<Line<T>> &newLines) {
    // TODO: Figure out how to add lines that are not merged
    for (int i = 0; i < lines_.size(); i++) {
      auto line = lines_.at(i);
      auto updatedLine = line;
      auto lHough = line.toHoughSpace();
      for (auto newLine : newLines) {
        auto nHough = newLine.toHoughSpace();
        if (line.overlaps(newLine) &&
            (lHough - nHough).length() < mergeThreshold_) {
          // TODO: Merge lines in hough space (note that we should accumulate them befor updating lines_ as maybe 3 lines should be merged)
          //       Accumulate to updatedLine;
        }
      }
      lines_[i] = updatedLine;
    }
  }

private:
  float mergeThreshold_;
  std::vector<Line<T>> lines_;
};
} // namespace scitos_common::map
