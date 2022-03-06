#pragma once

#include <cstdint>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "scitos_common/growing_pc.hpp"
#include "scitos_common/map/line.hpp"
#include "scitos_common/map/map.hpp"
#include "scitos_common/vec2.hpp"

class Mapper {
public:
  explicit Mapper(ros::NodeHandle nh);
  void step(const ros::TimerEvent &event);

private:
  ros::NodeHandle nh_;

  nav_msgs::OdometryPtr odometry_;
  tf::StampedTransform worldToRobot_;
  sensor_msgs::LaserScan laserScan_;

  ros::Subscriber odometrySub_;
  ros::Subscriber laserScanSub_;

  ros::Publisher erosionGridPub_;
  ros::Publisher erosionPub_;
  ros::Publisher dbscanPub_;
  ros::Publisher kmeansPub_;
  ros::Publisher linesPub_;
  ros::Publisher mapPub_;

  tf::TransformListener tfListener_;

  struct {
    float r;
    int n;
  } dbscan_;

  struct {
    uint32_t k;
    uint32_t iterations;
  } kmeans_;

  struct {
    float epsilon;
    std::vector<std::vector<scitos_common::map::Line<float>>> lines;
  } iepf_;

  scitos_common::map::Map<float> map_;

  void odometryCallback(nav_msgs::OdometryPtr msg);
  void laserScanCallback(sensor_msgs::LaserScan msg);
  std::vector<Vec2<float>> getLaserScanPoints(const ros::Time& currentTime);
  void publishDbscan(const std::vector<Vec2<float>> &points,
                     const std::vector<int> &labels) const;
  void publishKMeans(const std::vector<Vec2<float>> &centroids) const;
  void publishErosion(const std::vector<Vec2<float>> &centroids) const;
  void publishLines() const;
  void publishMap() const;
};
