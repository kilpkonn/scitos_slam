#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include <ros/package.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/console.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>

#include <yaml-cpp/yaml.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "scitos_common/dbscan.hpp"
#include "scitos_common/ekf.hpp"
#include "scitos_common/grid/morphology.hpp"
#include "scitos_common/iepf.hpp"
#include "scitos_common/kmeans.hpp"
#include "scitos_common/map/line.hpp"
#include "scitos_common/odometry_subscriber.hpp"
#include "scitos_common/polar2.hpp"
#include "scitos_common/vec2.hpp"

#include "scitos_common/Vec2Array.h"

#include "scitos_mapper/scitos_mapper.hpp"

Mapper::Mapper(ros::NodeHandle nh) : nh_{nh} {
  ekfOdometrySub_ = std::make_unique<scitos_common::OdometrySubscriber>(
      &nh_, "/ekf_odom", 50);
  odometrySub_ = nh_.subscribe("/odom", 1, &Mapper::odometryCallback, this);
  laserScanSub_ =
      nh_.subscribe("/laser_scan", 1, &Mapper::laserScanCallback, this);
  saveMapSub_ = nh_.subscribe("/save_map", 1, &Mapper::saveMapCallback, this);
  // cmdVelSub_ = nh_.subscribe("/cmd_vel", 1, &Mapper::cmdVelCallback, this);

  mapLineHoughPub_ =
      nh_.advertise<scitos_common::Vec2Array>("/debug/map_line_hough", 3);
  erosionPub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/debug/erosion", 3);
  erosionGridPub_ =
      nh_.advertise<nav_msgs::OccupancyGrid>("/debug/erosion_grid", 3);
  dbscanPub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/debug/dbscan", 3);
  cornerCombinationPub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "/debug/corner_combination", 3);
  linesPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/debug/lines", 3);
  matchedLinesPub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/debug/matched_lines", 3);
  mapPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/debug/map", 3);
  ekfPub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      "/debug/ekf_estimate", 3);
  odomPub_ = nh_.advertise<nav_msgs::Odometry>("/ekf_odom", 3);

  dbscan_.r = nh_.param("/dbscan/r", 0.1f);
  dbscan_.n = nh_.param("/dbscan/n", 10);
  cornerCombinationDBScan.n = nh_.param("/corner_combination/n", 2);
  cornerCombinationDBScan.r = nh_.param("/corner_combination/r", 0.2f);
  kmeans_.k = nh_.param("/kmeans/k", 10);
  kmeans_.iterations = nh_.param("/kmeans/iterations", 5);
  iepf_.epsilon = nh_.param("/iepf/epsilon", 0.5f);
  float padding = nh_.param("/map/padding", 0.05f);
  float fadePower = nh_.param("/map/fade_power", 0.1f);

  float a1 = nh_.param("/map/ekf/a1", 0.1f);
  float a2 = nh_.param("/map/ekf/a2", 0.1f);
  float a3 = nh_.param("/map/ekf/a3", 0.1f);
  float a4 = nh_.param("/map/ekf/a4", 0.1f);
  float s1 = nh_.param("/map/ekf/s1", 0.1f);
  float s2 = nh_.param("/map/ekf/s2", 0.1f);
  float s3 = nh_.param("/map/ekf/s3", 0.1f);
  ekf_ = scitos_common::EKF(a1, a2, a3, a4);
  ekf_.setState({0.f, 0.f, 0.f});
  Eigen::Matrix3f sigma{{0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}};
  ekf_.setVariances(sigma);
  Eigen::Matrix3f covs{{s1, 0.f, 0.f}, {0.f, s2, 0.f}, {0.f, 0.f, s3}};
  ekf_.setSensorVariances(covs);

  map_ = scitos_common::map::Map<float>(padding, fadePower);
  if (nh_.hasParam("/map/load")) {
    std::string path = nh_.param<std::string>("/map/load", "map.yaml");
    loadMap(path);
  }
}

void Mapper::step(const ros::TimerEvent &event) {
  if (!laserScan_.header.stamp.isValid()) {
    return;
  }

  std::chrono::nanoseconds dt(event.profile.last_duration.toNSec());
  // ROS_INFO("Time diff: %f", (odometry_->header.stamp -
  // laserScan_.header.stamp).toSec());

  // ROS_INFO("start");
  nav_msgs::OccupancyGrid erodedGrid;
  std::vector<Vec2<float>> scanPoints = getLaserScanPoints();
  std::vector<Vec2<float>> erodedPoints =
      grid::open(scanPoints, 0.1f, erodedGrid);
  erodedGrid.header = laserScan_.header;
  erodedGrid.header.frame_id = "odom";

  auto labels = scitos_common::dbscan<Vec2<float>>(
      erodedPoints, [](auto a, auto b) { return (a - b).length(); }, dbscan_.n,
      dbscan_.r);

  std::vector<Vec2<float>> filteredPoints;
  filteredPoints.reserve(filteredPoints.size() / 2); // Estimate 50% dropout
  for (size_t i = 0; i < labels.size(); i++) {
    if (labels[i] != 0)
      filteredPoints.push_back(scanPoints.at(i));
  }

  auto clusters = scitos_common::dbscan2<Vec2<float>>(
      erodedPoints, [](auto a, auto b) { return (a - b).length(); }, dbscan_.n,
      dbscan_.r);

  iepf_.lines.clear();
  for (const auto &cluster : clusters) {
    iepf_.lines.push_back(scitos_common::douglas_peuker::simplify2<float>(
        cluster, iepf_.epsilon, [&](const std::vector<Vec2<float>> &v) {
          float len = v.size() > 0 ? (v[0] - v[v.size() - 1]).length() : 1.f;
          return std::clamp(static_cast<float>(v.size()) / len, 0.f, 0.5f);
        }));
  }

  // Flatten lines
  std::vector<scitos_common::map::Line<float>> mapLines;
  mapLines.reserve(iepf_.lines.size());
  for (auto line : iepf_.lines) {
    mapLines.insert(mapLines.end(), line.begin(), line.end());
  }

  std::vector<std::pair<Vec2<float>, Vec2<float>>> matchedLines;
  ekf_.correct(map_, mapLines, &matchedLines);

  // NOTE:mapLines should be updated to new pos and rot before accumulate

  // if (erodedPoints.size() > 1) {
  //   const Vec2<float> loc(odometry_->pose.pose.position.x,
  //                         odometry_->pose.pose.position.y);
  //   size_t n = erodedPoints.size() / 4; // Skip 25% on both sides
  //   map_.prune(mapLines, loc, (erodedPoints[n] - loc).normalize(),
  //              (erodedPoints[erodedPoints.size() - n] - loc).normalize());
  // }

  // map_.accumulate2(mapLines);

  // std::vector<std::pair<Vec2<float>, std::set<Vec2<float> *>>>
  //     cornerVisualization;
  // map_.combineCorners(cornerCombinationDBScan.n, cornerCombinationDBScan.r,
  //                     cornerVisualization);
  // BUG: align is broken
  // map_.align(4);
  //  ROS_INFO("map size: %zu", map_.getLines().size());

  // ROS_INFO("done");

  scitos_common::Vec2Array mapLineHoughMsg;
  for (auto line : map_.getLines()) {
    auto point = line.toHoughSpace();
    mapLineHoughMsg.x.push_back(point.x);
    mapLineHoughMsg.y.push_back(point.y);
  }
  mapLineHoughPub_.publish(mapLineHoughMsg);
  erosionGridPub_.publish(erodedGrid);
  publishErosion(erodedPoints);
  publishDbscan(erodedPoints, labels);
  // publishCornerCombining(cornerVisualization);
  publishMatchedLines(matchedLines);
  publishLines();
  publishMap();
}

void Mapper::odometryCallback(nav_msgs::Odometry msg) {
  double roll, pitch, yaw;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  auto [m, sigma] =
      ekf_.predictOdom(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw);
  worldToRobot_.child_frame_id_ = "base_footprint";
  worldToRobot_.frame_id_ = "map";
  worldToRobot_.stamp_ = odomStamp_;
  worldToRobot_.setOrigin({m(0), m(1), 0.05f});
  worldToRobot_.setRotation(tf::createQuaternionFromRPY(0.f, 0.f, m(2)));

  publishEkf(m, sigma);
  publishOdom();
}

void Mapper::laserScanCallback(sensor_msgs::LaserScan msg) { laserScan_ = msg; }

void Mapper::cmdVelCallback(geometry_msgs::Twist msg) {
  auto dt =
      std::chrono::nanoseconds((ros::Time::now() - cmdVelStamp_).toNSec());
  cmdVelStamp_ = ros::Time::now();
  auto [m, sigma] =
      ekf_.predictVelCmd(msg.linear.x, msg.angular.z,
                   std::chrono::duration_cast<std::chrono::milliseconds>(dt));
  worldToRobot_.child_frame_id_ = "base_footprint";
  worldToRobot_.frame_id_ = "map";
  worldToRobot_.stamp_ = cmdVelStamp_;
  worldToRobot_.setOrigin({m(0), m(1), 0.05f});
  // ROS_INFO("(%f, %f, %f)", m(0), m(1), m(2));
  worldToRobot_.setRotation(tf::createQuaternionFromRPY(0.f, 0.f, m(2)));

  publishEkf(m, sigma);
  publishOdom();
}

void Mapper::saveMapCallback(std_msgs::String msg) const {
  std::string map_file = msg.data;
  ROS_INFO("SAVING MAP TO: %s", map_file.c_str());
  YAML::Node map = YAML::Load("[]");

  for (const auto &line : map_.getLines()) {
    if (line.confidence > 0.75) {
      YAML::Node lineNode;
      lineNode["confidence"] = 1.0; // line.confidence;
      lineNode["p1"]["x"] = line.p1.x;
      lineNode["p1"]["y"] = line.p1.y;
      lineNode["p2"]["x"] = line.p2.x;
      lineNode["p2"]["y"] = line.p2.y;
      map.push_back(lineNode);
    }
  }
  ROS_INFO("OPENING SAVE FILE");
  std::ofstream fout(map_file);
  fout << map;
  fout.close();
  ROS_INFO("FINISHED MAP SAVING!");
}

void Mapper::loadMap(std::string path) {
  std::string mapperPackagePath = ros::package::getPath("scitos_mapper");
  std::string fullPath = mapperPackagePath + "/" + path;
  ROS_INFO("LOADING MAP FROM: %s", fullPath.c_str());
  YAML::Node map = YAML::LoadFile(fullPath);
  std::vector<scitos_common::map::Line<float>> lines;
  lines.reserve(map.size());

  for (size_t i = 0; i < map.size(); i++) {
    // Not sure why emblace_back doesn't work
    scitos_common::map::Line<float> line = {
        {map[i]["p1"]["x"].as<float>(), map[i]["p1"]["y"].as<float>()},
        {map[i]["p2"]["x"].as<float>(), map[i]["p2"]["y"].as<float>()},
        map[i]["confidence"].as<float>()};
    lines.push_back(line);
  }
  map_.loadFromLines(lines);
  ROS_INFO("MAP LOADED!");
}

std::vector<Vec2<float>> Mapper::getLaserScanPoints() {
  if (!laserScan_.header.stamp.isValid() ||
      !ekfOdometrySub_->hasReceivedFirst) {
    return {};
  }

  tf::StampedTransform lidarToRobot;
  try {
    tfListener_.lookupTransform("/base_footprint", "/hokuyo_link",
                                laserScan_.header.stamp, lidarToRobot);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  nav_msgs::Odometry odometry = ekfOdometrySub_->getNearestOrThrow(
      std::chrono::nanoseconds(laserScan_.header.stamp.toNSec()),
      "MAPPER: No odometry received");
  const Vec2<float> loc(odometry.pose.pose.position.x,
                        odometry.pose.pose.position.y);
  double roll, pitch, yaw;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(odometry.pose.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  std::vector<Vec2<float>> output;
  output.reserve(laserScan_.ranges.size());
  for (uint i = 0; i < laserScan_.ranges.size(); i++) {
    output.push_back(
        worldToRobot_ * lidarToRobot *
        static_cast<tf::Vector3>(
            Polar2(laserScan_.ranges[i],
                   laserScan_.angle_min + laserScan_.angle_increment * i)));
  }
  return output;
}

void Mapper::publishCornerCombining(
    std::vector<std::pair<Vec2<float>, std::set<Vec2<float> *>>>
        &cornerVisualization) const {
  visualization_msgs::MarkerArray markers;
  for (size_t i = 0; i < cornerVisualization.size(); i++) {
    std::pair<Vec2<float>, std::set<Vec2<float> *>> corner =
        cornerVisualization.at(i);
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.pose.orientation.w = 1.0;
    marker.pose.position.x = corner.first.x;
    marker.pose.position.y = corner.first.y;
    marker.pose.position.z = 0.05;
    marker.id = i * 1000;
    markers.markers.push_back(marker);

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 0;
    marker.color.g = std::clamp(i * 0.1, 0.0, 1.0);
    marker.color.b = std::clamp(1.0 - i * 0.1, 0.0, 1.0);
    int j = 0;
    for (Vec2<float> *point : corner.second) {
      marker.pose.position.x = point->x;
      marker.pose.position.y = point->y;
      marker.id = i * 1000 + (++j);
      markers.markers.push_back(marker);
    }
  }

  cornerCombinationPub_.publish(markers);
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
    marker.color.r = 0.0; // std::clamp(i * 0.2, 0.0, 1.0);
    marker.color.g = 1.0; // std::clamp(-1.0 + i * 0.2, 0.0, 1.0);
    marker.color.b = 1.0;
    marker.pose.orientation.w = 1.0;

    for (auto segment : line) {
      geometry_msgs::Point p;
      p.z = 0.05;
      p.x = segment.p1.x;
      p.y = segment.p1.y;
      marker.points.push_back(p);
      p.x = segment.p2.x;
      p.y = segment.p2.y;
      marker.points.push_back(p);
    }
    marker.id = i;
    markers.markers.push_back(marker);
  }

  linesPub_.publish(markers);
}

void Mapper::publishMatchedLines(
    std::vector<std::pair<Vec2<float>, Vec2<float>>> matchedLines) const {
  visualization_msgs::MarkerArray markers;
  int i = 0;
  for (auto match : matchedLines) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 0.5;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.0;
    marker.pose.orientation.w = 1.0;

    geometry_msgs::Point p;
    p.z = 0.05;
    p.x = match.first.x;
    p.y = match.first.y;
    marker.points.push_back(p);
    p.x = match.second.x;
    p.y = match.second.y;
    marker.points.push_back(p);

    marker.id = i++;
    markers.markers.push_back(marker);
  }

  matchedLinesPub_.publish(markers);
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
    marker.color.a = 0.5 * line.confidence;
    marker.color.r = std::clamp(i * 0.05, 0.0, 1.0);
    marker.color.g = std::clamp(-1.0 + i * 0.05, 0.0, 1.0);
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

// Remove as publish Odom has all the data
void Mapper::publishEkf(const Eigen::Vector3f &m,
                        const Eigen::Matrix3f &sigma) const {
  // geometry_msgs::PoseStamped msg;
  geometry_msgs::PoseWithCovarianceStamped msg;
  msg.header.frame_id = "odom";
  msg.header.stamp = ros::Time::now();

  msg.pose.pose.position.x = m(0);
  msg.pose.pose.position.y = m(1);
  msg.pose.pose.position.z = 0.05;

  msg.pose.covariance[0] = sigma(0, 0);
  msg.pose.covariance[1] = sigma(0, 1);
  msg.pose.covariance[5] = sigma(0, 2);
  msg.pose.covariance[6] = sigma(1, 0);
  msg.pose.covariance[7] = sigma(1, 1);
  msg.pose.covariance[11] = sigma(1, 2);
  msg.pose.covariance[31] = sigma(2, 0);
  msg.pose.covariance[32] = sigma(2, 1);
  msg.pose.covariance[35] = sigma(2, 2);

  auto quat = tf::createQuaternionFromRPY(0.f, 0.f, m(2));

  msg.pose.pose.orientation.w = quat.getW();
  msg.pose.pose.orientation.x = quat.getX();
  msg.pose.pose.orientation.y = quat.getY();
  msg.pose.pose.orientation.z = quat.getZ();

  ekfPub_.publish(msg);
}

void Mapper::publishOdom() const {
  static uint64_t odomFrameId = 0;
  nav_msgs::Odometry msg;
  msg.header.frame_id = "map";
  msg.child_frame_id = "base_footprint";
  msg.header.seq = odomFrameId++;
  msg.header.stamp = cmdVelStamp_;

  auto pos = ekf_.getPos();
  auto rot = ekf_.getRotation();
  auto quat = tf::createQuaternionFromRPY(0.f, 0.f, rot);

  msg.pose.pose.position.x = pos.x;
  msg.pose.pose.position.y = pos.y;
  msg.pose.pose.position.z = 0.05;

  msg.pose.pose.orientation.w = quat.getW();
  msg.pose.pose.orientation.x = quat.getX();
  msg.pose.pose.orientation.y = quat.getY();
  msg.pose.pose.orientation.z = quat.getZ();

  auto sigma = ekf_.getCovarince();

  msg.pose.covariance[0] = sigma(0, 0);
  msg.pose.covariance[1] = sigma(0, 1);
  msg.pose.covariance[5] = sigma(0, 2);
  msg.pose.covariance[6] = sigma(1, 0);
  msg.pose.covariance[7] = sigma(1, 1);
  msg.pose.covariance[11] = sigma(1, 2);
  msg.pose.covariance[31] = sigma(2, 0);
  msg.pose.covariance[32] = sigma(2, 1);
  msg.pose.covariance[35] = sigma(2, 2);

  // TODO: Velocity

  odomPub_.publish(msg);
}
