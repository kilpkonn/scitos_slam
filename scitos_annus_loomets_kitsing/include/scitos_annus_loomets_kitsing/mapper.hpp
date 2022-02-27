#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <scitos_common/vec2.hpp>

class Mapper
{
public:
  explicit Mapper(ros::NodeHandle nh);
  void step(const ros::TimerEvent& event);
private:
  ros::NodeHandle nh_;

  nav_msgs::OdometryPtr odometry_;
  sensor_msgs::LaserScan laserScan_;

  ros::Subscriber odometrySub_;
  ros::Subscriber laserScanSub_;

  ros::Publisher filteredLaserScanPub_;

  void odometryCallback(nav_msgs::OdometryPtr msg);
  void laserScanCallback(sensor_msgs::LaserScan msg);
  std::vector<Vec2<float>> getLaserScanPoints();
  sensor_msgs::LaserScan getLaserScan(std::vector<Vec2<float>> points);
};
