#pragma once

#include <cstdint>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>

#include "scitos_common/vec2.hpp"
#include "scitos_common/growing_pc.hpp"

class Mapper {
public:
  explicit Mapper(ros::NodeHandle nh);
  void step(const ros::TimerEvent &event);

private:
  ros::NodeHandle nh_;

  nav_msgs::OdometryPtr odometry_;
  sensor_msgs::LaserScan laserScan_;

  ros::Subscriber odometrySub_;
  ros::Subscriber laserScanSub_;

  ros::Publisher filteredLaserScanPub_;
  ros::Publisher dbscanPub_;
  ros::Publisher kmeansPub_;
  ros::Publisher pcPub_;

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
    std::vector<Vec2<float>> data;
  } pc_;


  void odometryCallback(nav_msgs::OdometryPtr msg);
  void laserScanCallback(sensor_msgs::LaserScan msg);
  std::vector<Vec2<float>> getLaserScanPoints();
  sensor_msgs::LaserScan getLaserScan(std::vector<Vec2<float>> points);
  void publishDbscan(const std::vector<Vec2<float>> &points,
                     const std::vector<int> &labels) const;
  void publishKMeans(const std::vector<Vec2<float>> &centroids) const;
  void publishPC() const;
};
