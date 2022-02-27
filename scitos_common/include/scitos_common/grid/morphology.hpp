#pragma once

#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/hal/interface.h>

#include "scitos_common/vec2.hpp"
#include "scitos_common/grid/transformations.hpp"


namespace grid {
  std::vector<Vec2<float>> open(std::vector<Vec2<float>> points, float resolution)
  {
    if(points.size() == 0)
      return {};
    
    Vec2<float> point_min = points[0];
    Vec2<float> point_max = points[0];
    for (Vec2<float> point: points)
    {
      point_min.x = std::min(point_min.x, point.x);
      point_min.y = std::min(point_min.y, point.y);
      point_max.x = std::max(point_max.x, point.x);
      point_max.y = std::max(point_max.y, point.y);
    }

    Vec2<float> grid_size = point_max - point_min;
    Vec2<int> grid_squares = (grid_size / resolution).ceil();
    cv::Mat point_grid(static_cast<cv::Size>(grid_squares), CV_8U);

    std::vector<std::pair<Vec2<float>, Vec2<int>>> point_locs;
    point_locs.reserve(points.size());
    for (Vec2<float> point: points)
    {
      std::optional<Vec2<int>> point_loc = grid::world_to_grid(point, point_min, grid_size, resolution);
      if (!point_loc)
        continue;
      point_locs.push_back(std::make_pair(point, point_loc.value()));
      point_grid.at<bool>(static_cast<cv::Point>(point_loc.value())) = true;
    }

    cv::morphologyEx(point_grid, point_grid, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3)));
    
    std::vector<Vec2<float>> output;
    output.reserve(points.size());
    for(auto p: point_locs)
    {
      if (point_grid.at<bool>(static_cast<cv::Point>(p.second)))
        output.push_back(p.first);
    }
    return output;
  }
}
