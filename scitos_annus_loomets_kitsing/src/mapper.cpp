#include <cstddef>
#include <ros/console.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <visualization_msgs/MarkerArray.h>

#include "scitos_common/dbscan.hpp"
#include "scitos_common/grid/morphology.hpp"
#include "scitos_common/iepf.hpp"
#include "scitos_common/kmeans.hpp"
#include "scitos_common/polar2.hpp"

#include "scitos_annus_loomets_kitsing/mapper.hpp"
#include "scitos_common/vec2.hpp"

Mapper::Mapper(ros::NodeHandle nh) : nh_{nh} {
  odometrySub_ = nh_.subscribe("/controller_diffdrive/odom", 1,
                               &Mapper::odometryCallback, this);
  laserScanSub_ =
      nh_.subscribe("/laser_scan", 1, &Mapper::laserScanCallback, this);
  filteredLaserScanPub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "/debug/filtered_laser_scan", 3);
  dbscanPub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/debug/dbscan", 3);
  kmeansPub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/debug/kmeans", 3);
  linesPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/debug/lines", 3);

  dbscan_.r = nh_.param("/dbscan/r", 0.1f);
  dbscan_.n = nh_.param("/dbscan/n", 10);
  kmeans_.k = nh_.param("/kmeans/k", 10);
  kmeans_.iterations = nh_.param("/kmeans/iterations", 5);
  iepf_.epsilon = nh_.param("/iepf/epsilon", 0.5f);
}

void Mapper::step(const ros::TimerEvent &event) {
  if (odometry_ == nullptr) {
    return;
  }
  if (!laserScan_.header.stamp.isValid()) {
    return;
  }

  ROS_INFO("start");
  std::vector<Vec2<float>> scanPoints = getLaserScanPoints();
  std::vector<Vec2<float>> erodedPoints = grid::open(scanPoints, 0.1f);

  auto labels = scitos_common::dbscan<Vec2<float>>(
      erodedPoints, [](auto a, auto b) { return (a - b).length(); }, dbscan_.n,
      dbscan_.r);

  std::vector<Vec2<float>> filteredPoints;
  filteredPoints.reserve(filteredPoints.size() / 2); // Estimate 50% dropout
  for (size_t i = 0; i < labels.size(); i++) {
    if (labels[i] != 0)
      filteredPoints.push_back(scanPoints.at(i));
  }

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
  ROS_INFO("done");

  publishDbscan(erodedPoints, labels);
  publishKMeans(centroids);
  publishLines();
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

sensor_msgs::LaserScan Mapper::getLaserScan(std::vector<Vec2<float>> points) {
  sensor_msgs::LaserScan scan;
  if (laserScan_.header.stamp.isValid()) {
    scan.header = laserScan_.header;
  }

  // TODO: Finish this.
  return scan;
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
    marker.id = i++;
    markers.markers.push_back(marker);
  }

  linesPub_.publish(markers);
}
