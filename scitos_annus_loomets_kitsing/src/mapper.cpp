#include <scitos_common/grid/morphology.hpp>

#include "scitos_common/polar2.hpp"

#include "scitos_annus_loomets_kitsing/mapper.hpp"


Mapper::Mapper(ros::NodeHandle nh) : nh_{nh}
{
  odometrySub_ = nh_.subscribe("/controller_diffdrive/odom", 1,
                               &Mapper::odometryCallback, this);
  laserScanSub_ = nh_.subscribe("/laser_scan", 1,
                               &Mapper::laserScanCallback, this);
  filteredLaserScanPub_ =
      nh_.advertise<sensor_msgs::LaserScan>("/debug/filtered_laser_scan", 3);
}

void Mapper::step(const ros::TimerEvent &event)
{
  if (odometry_ == nullptr)
  {
    return;
  }
  if (!laserScan_.header.stamp.isValid())
  {
    return;
  }

  std::vector<Vec2<float>> scanPoints = getLaserScanPoints();
  std::vector<Vec2<float>> filteredPoints = grid::open(scanPoints, 0.05f);

  //filteredLaserScanPub_
}

void Mapper::odometryCallback(nav_msgs::OdometryPtr msg) {
  odometry_ = msg;
}

void Mapper::laserScanCallback(sensor_msgs::LaserScan msg) {
  laserScan_ = msg;
}

std::vector<Vec2<float>> Mapper::getLaserScanPoints()
{
  if (!laserScan_.header.stamp.isValid())
  {
    return {};
  }

  std::vector<Vec2<float>> output;
  output.reserve(laserScan_.ranges.size());
  for(uint i=0; i<laserScan_.ranges.size(); i++)
  {
    float angle = laserScan_.angle_min + laserScan_.angle_increment * i;
    output.push_back(Polar2(laserScan_.ranges[i], angle));
  }
  return output;
}

sensor_msgs::LaserScan Mapper::getLaserScan(std::vector<Vec2<float>> points)
{
  sensor_msgs::LaserScan scan;
  if (laserScan_.header.stamp.isValid())
  {
    scan.header = laserScan_.header;
  }

  //TODO: Finish this.
  return scan;
}