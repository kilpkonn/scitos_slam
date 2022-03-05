#include <cstddef>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/console.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <visualization_msgs/MarkerArray.h>

#include "scitos_common/dbscan.hpp"
#include "scitos_common/grid/morphology.hpp"
#include "scitos_common/iepf.hpp"
#include "scitos_common/kmeans.hpp"
#include "scitos_common/map/line.hpp"
#include "scitos_common/polar2.hpp"

#include "scitos_annus_loomets_kitsing/mapper.hpp"
#include "scitos_common/vec2.hpp"

Mapper::Mapper(ros::NodeHandle nh) : nh_{nh} {
  odometrySub_ =
      nh_.subscribe("/ground_truth", 1, &Mapper::odometryCallback, this);
  laserScanSub_ =
      nh_.subscribe("/laser_scan", 1, &Mapper::laserScanCallback, this);
  erosionPub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/debug/erosion", 3);
  erosionGridPub_ =
      nh_.advertise<nav_msgs::OccupancyGrid>("/debug/erosion_grid", 3);
  dbscanPub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/debug/dbscan", 3);
  kmeansPub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/debug/kmeans", 3);
  linesPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/debug/lines", 3);
  mapPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/debug/map", 3);

  dbscan_.r = nh_.param("/dbscan/r", 0.1f);
  dbscan_.n = nh_.param("/dbscan/n", 10);
  kmeans_.k = nh_.param("/kmeans/k", 10);
  kmeans_.iterations = nh_.param("/kmeans/iterations", 5);
  iepf_.epsilon = nh_.param("/iepf/epsilon", 0.5f);
  float mergeLinesThreshold = nh_.param("/map/merge_lines_threshold", 0.5f);

  map_ = scitos_common::map::Map<float>(mergeLinesThreshold);
}

void Mapper::step(const ros::TimerEvent &event) {
  if (odometry_ == nullptr) {
    return;
  }
  if (!laserScan_.header.stamp.isValid()) {
    return;
  }

  // ROS_INFO("start");
  nav_msgs::OccupancyGrid erodedGrid;
  std::vector<Vec2<float>> scanPoints = getLaserScanPoints();
  std::vector<Vec2<float>> erodedPoints =
      grid::open(scanPoints, 0.1f, erodedGrid);
  erodedGrid.info.origin.orientation = odometry_->pose.pose.orientation;
  erodedGrid.header = laserScan_.header;

  auto labels = scitos_common::dbscan<Vec2<float>>(
      erodedPoints, [](auto a, auto b) { return (a - b).length(); }, dbscan_.n,
      dbscan_.r);

  std::vector<Vec2<float>> filteredPoints;
  filteredPoints.reserve(filteredPoints.size() / 2); // Estimate 50% dropout
  for (size_t i = 0; i < labels.size(); i++) {
    if (labels[i] != 0)
      filteredPoints.push_back(scanPoints.at(i));
  }

  // NOTE: Kmeans can likely be removed
  auto centroids = scitos_common::kmeans<Vec2<float>>(
      filteredPoints, kmeans_.k,
      [](auto a, auto b) { return (a - b).length(); }, kmeans_.iterations);
  auto clusters = scitos_common::dbscan2<Vec2<float>>(
      erodedPoints, [](auto a, auto b) { return (a - b).length(); }, dbscan_.n,
      dbscan_.r);

  iepf_.lines.clear();
  for (const auto &cluster : clusters) {
    iepf_.lines.push_back(
        scitos_common::douglas_peuker::simplify(cluster, iepf_.epsilon));
  }

  // Split to segments
  std::vector<scitos_common::map::Line<float>> mapLines;
  mapLines.reserve(iepf_.lines.size());
  for (const auto &line : iepf_.lines) {
    if (line.size() >= 2) {
      for (size_t i = 0; i < line.size() - 1; i++) {
        mapLines.push_back({line.at(i), line.at(i + 1), 0.1f});  // TODO: Some estimate for confidence
      }
    }
  }

  map_.accumulate2(mapLines);

  // ROS_INFO("done");

  erosionGridPub_.publish(erodedGrid);
  publishErosion(erodedPoints);
  publishDbscan(erodedPoints, labels);
  publishKMeans(centroids);
  publishLines();
  publishMap();
}

void Mapper::odometryCallback(nav_msgs::OdometryPtr msg) { odometry_ = msg; }

void Mapper::laserScanCallback(sensor_msgs::LaserScan msg) { laserScan_ = msg; }

std::vector<Vec2<float>> Mapper::getLaserScanPoints() {
  if (!laserScan_.header.stamp.isValid() || odometry_ == nullptr) {
    return {};
  }

  const Vec2<float> loc(odometry_->pose.pose.position.x,
                        odometry_->pose.pose.position.y);
  double roll, pitch, yaw;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(odometry_->pose.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  std::vector<Vec2<float>> output;
  output.reserve(laserScan_.ranges.size());
  for (uint i = 0; i < laserScan_.ranges.size(); i++) {
    float angle = yaw + laserScan_.angle_min + laserScan_.angle_increment * i;
    output.push_back(Polar2(laserScan_.ranges[i], angle) + loc);
  }
  return output;
}

void Mapper::publishDbscan(const std::vector<Vec2<float>> &points,
                           const std::vector<int> &labels) const {
  visualization_msgs::MarkerArray markers;
  for (size_t i = 0; i < points.size(); i++) {
    auto p = points.at(i);
    auto lbl = labels.at(i);
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = std::clamp(lbl * 0.2, 0.0, 1.0);
    marker.color.g = std::clamp(-1.0 + lbl * 0.2, 0.0, 1.0);
    marker.color.b = std::clamp(-2.0 + lbl * 0.2, 0.0, 1.0);
    marker.pose.orientation.w = 1.0;
    marker.pose.position.x = p.x;
    marker.pose.position.y = p.y;
    marker.pose.position.z = 0.05;
    marker.id = i++;
    markers.markers.push_back(marker);
  }

  dbscanPub_.publish(markers);
}

void Mapper::publishKMeans(const std::vector<Vec2<float>> &centroids) const {
  visualization_msgs::MarkerArray markers;
  for (size_t i = 0; i < centroids.size(); i++) {
    auto p = centroids.at(i);
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = std::clamp(i * 0.1, 0.0, 1.0);
    marker.color.g = std::clamp(-1.0 + i * 0.1, 0.0, 1.0);
    marker.color.b = std::clamp(-2.0 + i * 0.1, 0.0, 1.0);
    marker.pose.orientation.w = 1.0;
    marker.pose.position.x = p.x;
    marker.pose.position.y = p.y;
    marker.pose.position.z = 0.05;
    marker.id = i++;
    markers.markers.push_back(marker);
  }

  kmeansPub_.publish(markers);
}

void Mapper::publishErosion(const std::vector<Vec2<float>> &points) const {
  visualization_msgs::MarkerArray markers;
  for (size_t i = 0; i < points.size(); i++) {
    auto p = points.at(i);
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.pose.orientation.w = 1.0;
    marker.pose.position.x = p.x;
    marker.pose.position.y = p.y;
    marker.pose.position.z = 0.05;
    marker.id = i++;
    markers.markers.push_back(marker);
  }

  erosionPub_.publish(markers);
}

void Mapper::publishLines() const {
  visualization_msgs::MarkerArray markers;
  for (size_t i = 0; i < iepf_.lines.size(); i++) {
    auto line = iepf_.lines.at(i);
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = std::clamp(i * 0.2, 0.0, 1.0);
    marker.color.g = std::clamp(-1.0 + i * 0.2, 0.0, 1.0);
    marker.color.b = 1.0;
    marker.pose.orientation.w = 1.0;

    for (auto point : line) {
      geometry_msgs::Point p;
      p.x = point.x;
      p.y = point.y;
      p.z = 0.05;
      marker.points.push_back(p);
    }
    marker.id = i;
    markers.markers.push_back(marker);
  }

  linesPub_.publish(markers);
}

void Mapper::publishMap() const {
  visualization_msgs::MarkerArray markers;
  int i = 0;
  for (auto line : map_.getLines()) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = line.confidence;
    marker.color.r = std::clamp(i * 0.1, 0.0, 1.0);
    marker.color.g = std::clamp(-1.0 + i * 0.1, 0.0, 1.0);
    marker.color.b = 1.0;
    marker.pose.orientation.w = 1.0;

    geometry_msgs::Point p;
    p.z = 0.05;
    p.x = line.p1.x;
    p.y = line.p1.y;
    marker.points.push_back(p);
    p.x = line.p2.x;
    p.y = line.p2.y;
    marker.points.push_back(p);

    marker.id = i++;
    markers.markers.push_back(marker);
  }

  mapPub_.publish(markers);
}
