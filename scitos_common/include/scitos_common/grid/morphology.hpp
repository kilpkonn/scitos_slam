#pragma once

#include <vector>

//#define OPENCV_TRAITS_ENABLE_DEPRECATED
#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/hal/interface.h>

#include <nav_msgs/OccupancyGrid.h>

#include "scitos_common/vec2.hpp"
#include "scitos_common/grid/transformations.hpp"


namespace grid {
  std::vector<Vec2<float>> open(std::vector<Vec2<float>> points, float resolution
                                , nav_msgs::OccupancyGrid& erodedGrid)
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
    Vec2<int> grid_squares = (grid_size / resolution).ceil() + Vec2<int>(1, 1);
    if (grid_squares.x <= 0 || grid_squares.y <= 0)
      return {};
    cv::Mat point_grid(static_cast<cv::Size>(grid_squares), CV_8U, cv::Scalar(0));

    std::vector<std::pair<Vec2<float>, Vec2<int>>> point_locs;
    point_locs.reserve(points.size());
    for (Vec2<float> point: points)
    {
      std::optional<Vec2<int>> point_loc = grid::world_to_grid(point, point_min, grid_size, resolution);
      if (!point_loc)
        continue;
      point_locs.push_back(std::make_pair(point, point_loc.value()));
      point_grid.at<unsigned char>(static_cast<cv::Point>(point_loc.value())) = 255;
    }

    //cv::Mat debugImg(static_cast<cv::Size>((grid_squares * 4.0f)), CV_8U);
    //cv::resize(point_grid, debugImg, cv::Size(), 4, 4);
    //cv::imshow("original", debugImg);

    cv::morphologyEx(point_grid, point_grid, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3)));
    cv::Mat donutKernel = (cv::Mat_<unsigned char>(3, 3) << 1, 1, 1,
                                                            1, 0, 1,
                                                            1, 1, 1);
    cv::Mat hasNeighbor(static_cast<cv::Size>(grid_squares), CV_8U, cv::Scalar(0));
    cv::morphologyEx(point_grid, hasNeighbor, cv::MORPH_DILATE, donutKernel);
    cv::bitwise_and(point_grid, hasNeighbor, point_grid);
    //cv::morphologyEx(point_grid, point_grid, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3)));
    
    std::vector<Vec2<float>> output;
    output.reserve(points.size());
    for(auto p: point_locs)
    {
      if (point_grid.at<unsigned char>(static_cast<cv::Point>(p.second)) == 255)
        output.push_back(p.first);
    }

    //cv::resize(point_grid, debugImg, cv::Size(), 4, 4);
    //cv::imshow("grid", debugImg);
    //cv::waitKey(1);

    /**Create debug map**/
    erodedGrid.data.reserve(point_grid.rows * point_grid.cols);
    for (int y = 0; y < point_grid.rows; y++) {
      for (int x = 0; x < point_grid.cols; x++) {
        const unsigned char intensity = point_grid.at<unsigned char>(y, x);
        erodedGrid.data.push_back(intensity);
      }
    }
    erodedGrid.info.resolution = resolution;
    erodedGrid.info.width = point_grid.cols;
    erodedGrid.info.height = point_grid.rows;
    erodedGrid.info.origin.position.x = point_min.x;
    erodedGrid.info.origin.position.y = point_min.y;

    return output;
  }
}
