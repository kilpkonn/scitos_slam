#pragma once

#include <cstdint>

#include <eigen3/Eigen/Eigen>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "geometry_msgs/Twist.h"
#include "scitos_common/ekf.hpp"
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
  geometry_msgs::Twist cmdVel_;

  ros::Subscriber odometrySub_;
  ros::Subscriber laserScanSub_;
  ros::Subscriber cmdVelSub_;
  ros::Subscriber saveMapSub_;

  ros::Publisher mapLineHoughPub_;
  ros::Publisher cornerCombinationPub_;
  ros::Publisher erosionGridPub_;
  ros::Publisher erosionPub_;
  ros::Publisher dbscanPub_;
  ros::Publisher linesPub_;
  ros::Publisher mapPub_;
  ros::Publisher ekfPub_;

  tf::TransformListener tfListener_;

  struct {
    float r;
    int n;
  } dbscan_, cornerCombinationDBScan;

  struct {
    uint32_t k;
    uint32_t iterations;
  } kmeans_;

  struct {
    float epsilon;
    std::vector<std::vector<scitos_common::map::Line<float>>> lines;
  } iepf_;

  scitos_common::map::Map<float> map_;
  scitos_common::EKF ekf_;

  void odometryCallback(nav_msgs::OdometryPtr msg);
  void laserScanCallback(sensor_msgs::LaserScan msg);
  void saveMapCallback(std_msgs::String msg) const;
  void cmdVelCallback(geometry_msgs::Twist msg);
  void loadMap(std::string msg);
  std::vector<Vec2<float>> getLaserScanPoints(const ros::Time &currentTime);
  void publishDbscan(const std::vector<Vec2<float>> &points,
                     const std::vector<int> &labels) const;
  void publishCornerCombining(
      std::vector<std::pair<Vec2<float>, std::set<Vec2<float> *>>>
          &cornerVisualization) const;
  void publishErosion(const std::vector<Vec2<float>> &centroids) const;
  void publishEkf(const Eigen::Vector3f &m) const;
  void publishLines() const;
  void publishMap() const;
};
