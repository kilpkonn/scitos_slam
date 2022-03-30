#pragma once
//#include <stdlib.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>

#include "scitos_common/queue_subscriber.hpp"

namespace scitos_common
{
  class OdometrySubscriber : public QueueSubscriber<nav_msgs::Odometry>
  {
    public:
    OdometrySubscriber(ros::NodeHandle * const nh
                        , const char * const topic
                        , unsigned int queueSize
                        , std::function<void (nav_msgs::Odometry)> cb)
                    : QueueSubscriber<nav_msgs::Odometry>(nh, topic, queueSize, cb)
    {
    }

    std::optional<nav_msgs::Odometry> getNearest(std::chrono::nanoseconds timestamp) override
    {
      auto lower = std::lower_bound(messages.begin(), messages.end(), std::make_pair(timestamp, nav_msgs::Odometry()), lessThanKey());
      static double extrapolationCount = 0; 
      static double interpolationCount = 0; 
      if (messages.empty())
        return std::nullopt;
      nav_msgs::Odometry toReturn;
      // ROS_INFO("Time: %zu ? %zu .. %zu", timestamp.count(), messages[0].first.count(), messages[messages.size()-1].first.count());
      if (lower == messages.end()){
        std::vector<nav_msgs::Odometry> recentOdometry;
        std::transform(messages.end() - std::min(3, static_cast<int>(messages.size())), messages.end(), std::back_inserter(recentOdometry),
                       [](std::pair<std::chrono::nanoseconds, nav_msgs::Odometry> e) -> nav_msgs::Odometry { return e.second; });
        toReturn = extrapolate(recentOdometry, timestamp);//(*(--lower)).second;
        ++extrapolationCount;
      }
      else{
        toReturn = interpolate((*(std::prev(lower))).second, (*(lower)).second, timestamp);//abs((*(std::prev(lower))).first - timestamp) < abs((*(lower)).first - timestamp)? (*(--lower)).second : (*(lower)).second;
        ++interpolationCount;
      }

      std::cout << "extrapolation " << extrapolationCount / (extrapolationCount + interpolationCount) * 100 
                << "% interpolation " << interpolationCount / (extrapolationCount + interpolationCount) * 100 << "%\n";

      if (cb)
        cb(toReturn);

      return toReturn;
    }

    private:
    nav_msgs::Odometry extrapolate(const std::vector<nav_msgs::Odometry>& past_odometry, const std::chrono::nanoseconds timestamp)
    {
      nav_msgs::Odometry o = past_odometry[past_odometry.size()-1];
      double timeDiff = timestamp.count() - o.header.stamp.toNSec();
      nav_msgs::Odometry v;
      nav_msgs::Odometry a;

      if(past_odometry.size() > 1)
      {
        v = odometryDiff(past_odometry[past_odometry.size()-1], past_odometry[past_odometry.size()-2]);
        v = odometryMultiply(v, 1.0 / static_cast<double>(v.header.stamp.toNSec()));
        if(past_odometry.size() > 2)
        {
          nav_msgs::Odometry v1 = odometryDiff(past_odometry[past_odometry.size()-2], past_odometry[past_odometry.size()-3]);
          if (v1.header.stamp.toNSec() != 0) {
            v1 = odometryMultiply(v1, 1.0 / static_cast<double>(v1.header.stamp.toNSec()));
            a = odometryDiff(v1, v);
          }
        }
      }

      nav_msgs::Odometry toReturn = odometryAdd(o, 
                                                odometryAdd(odometryMultiply(v, timeDiff),
                                                            odometryMultiply(a, timeDiff * timeDiff / 2.0)));
      
      return toReturn;
    }

    nav_msgs::Odometry odometryDiff(const nav_msgs::Odometry& o0, const nav_msgs::Odometry& o1) const
    {
      nav_msgs::Odometry toReturn;

      //std::cout << "time diff: " << (o1.header.stamp - o0.header.stamp).toSec() << "\n";
      toReturn.header.stamp = ros::Time((o0.header.stamp - o1.header.stamp).toSec());
      toReturn.header.frame_id = o0.header.frame_id;
      toReturn.child_frame_id = o0.child_frame_id;
      toReturn.pose.pose.position.x = o0.pose.pose.position.x - o1.pose.pose.position.x;
      toReturn.pose.pose.position.y = o0.pose.pose.position.y - o1.pose.pose.position.y;
      toReturn.pose.pose.position.z = o0.pose.pose.position.z - o1.pose.pose.position.z;
      tf2::Quaternion q0;
      tf2::Quaternion q1;
      tf2::fromMsg(o0.pose.pose.orientation, q0);
      tf2::fromMsg(o1.pose.pose.orientation, q1);
      toReturn.pose.pose.orientation = tf2::toMsg(q0 - q1);
      toReturn.twist.twist.linear.x = o0.twist.twist.linear.x - o1.twist.twist.linear.x;
      toReturn.twist.twist.linear.y = o0.twist.twist.linear.y - o1.twist.twist.linear.y;
      toReturn.twist.twist.linear.z = o0.twist.twist.linear.z - o1.twist.twist.linear.z;
      toReturn.twist.twist.angular.x = o0.twist.twist.angular.x - o1.twist.twist.angular.x;
      toReturn.twist.twist.angular.y = o0.twist.twist.angular.y - o1.twist.twist.angular.y;
      toReturn.twist.twist.angular.z = o0.twist.twist.angular.z - o1.twist.twist.angular.z;

      return toReturn;
    }

    nav_msgs::Odometry odometryAdd(const nav_msgs::Odometry& o0, const nav_msgs::Odometry& o1) const
    {
      nav_msgs::Odometry toReturn;

      toReturn.header.stamp = ros::Time((ros::Duration(o0.header.stamp.toSec()) + ros::Duration(o1.header.stamp.toSec())).toSec());
      toReturn.header.frame_id = o0.header.frame_id;
      toReturn.child_frame_id = o0.child_frame_id;
      toReturn.pose.pose.position.x = o0.pose.pose.position.x + o1.pose.pose.position.x;
      toReturn.pose.pose.position.y = o0.pose.pose.position.y + o1.pose.pose.position.y;
      toReturn.pose.pose.position.z = o0.pose.pose.position.z + o1.pose.pose.position.z;
      tf2::Quaternion q0;
      tf2::Quaternion q1;
      tf2::fromMsg(o0.pose.pose.orientation, q0);
      tf2::fromMsg(o1.pose.pose.orientation, q1);
      toReturn.pose.pose.orientation = tf2::toMsg(q0 + q1);
      toReturn.twist.twist.linear.x = o0.twist.twist.linear.x + o1.twist.twist.linear.x;
      toReturn.twist.twist.linear.y = o0.twist.twist.linear.y + o1.twist.twist.linear.y;
      toReturn.twist.twist.linear.z = o0.twist.twist.linear.z + o1.twist.twist.linear.z;
      toReturn.twist.twist.angular.x = o0.twist.twist.angular.x + o1.twist.twist.angular.x;
      toReturn.twist.twist.angular.y = o0.twist.twist.angular.y + o1.twist.twist.angular.y;
      toReturn.twist.twist.angular.z = o0.twist.twist.angular.z + o1.twist.twist.angular.z;

      return toReturn;
    }

    nav_msgs::Odometry odometryMultiply(const nav_msgs::Odometry& o, const double m) const
    {
      nav_msgs::Odometry toReturn;

      toReturn.header.stamp = ros::Time((ros::Duration(o.header.stamp.toSec()) * m).toSec());
      toReturn.header.frame_id = o.header.frame_id;
      toReturn.child_frame_id = o.child_frame_id;
      toReturn.pose.pose.position.x = o.pose.pose.position.x * m;
      toReturn.pose.pose.position.y = o.pose.pose.position.y * m;
      toReturn.pose.pose.position.z = o.pose.pose.position.z * m;
      tf2::Quaternion q;
      tf2::fromMsg(o.pose.pose.orientation, q);
      toReturn.pose.pose.orientation = tf2::toMsg(q * m);
      toReturn.twist.twist.linear.x = o.twist.twist.linear.x * m;
      toReturn.twist.twist.linear.y = o.twist.twist.linear.y * m;
      toReturn.twist.twist.linear.z = o.twist.twist.linear.z * m;
      toReturn.twist.twist.angular.x = o.twist.twist.angular.x * m;
      toReturn.twist.twist.angular.y = o.twist.twist.angular.y * m;
      toReturn.twist.twist.angular.z = o.twist.twist.angular.z * m;

      return toReturn;
    }

    nav_msgs::Odometry interpolate(const nav_msgs::Odometry& o0, const nav_msgs::Odometry& o1, const std::chrono::nanoseconds timestamp)
    {
      if(o0.header.stamp==o1.header.stamp)
        throw "OdometrySubscriber: Interpolating between same timestamp points and request";
      double weight0 = abs(static_cast<double>(o0.header.stamp.toNSec() - timestamp.count()));
      weight0 /= weight0 + abs(static_cast<double>(o1.header.stamp.toNSec() - timestamp.count()));

      nav_msgs::Odometry toReturn;
      toReturn.header.stamp = ros::Time(0, timestamp.count());
      toReturn.header.frame_id = o0.header.frame_id;
      toReturn.child_frame_id = o0.child_frame_id;
      toReturn.pose.pose.position.x = interpolateLinear(o0.pose.pose.position.x, o1.pose.pose.position.x, weight0);
      toReturn.pose.pose.position.y = interpolateLinear(o0.pose.pose.position.y, o1.pose.pose.position.y, weight0);
      toReturn.pose.pose.position.z = interpolateLinear(o0.pose.pose.position.z, o1.pose.pose.position.z, weight0);
      tf2::Quaternion q0;
      tf2::Quaternion q1;
      tf2::fromMsg(o0.pose.pose.orientation, q0);
      tf2::fromMsg(o1.pose.pose.orientation, q1);
      toReturn.pose.pose.orientation = tf2::toMsg(q0.slerp(q1, 1.0 - weight0));
      toReturn.twist.twist.linear.x = interpolateLinear(o0.twist.twist.linear.x, o1.twist.twist.linear.x, weight0);
      toReturn.twist.twist.linear.y = interpolateLinear(o0.twist.twist.linear.y, o1.twist.twist.linear.y, weight0);
      toReturn.twist.twist.linear.z = interpolateLinear(o0.twist.twist.linear.z, o1.twist.twist.linear.z, weight0);
      toReturn.twist.twist.angular.x = interpolateLinear(o0.twist.twist.angular.x, o1.twist.twist.angular.x, weight0);
      toReturn.twist.twist.angular.y = interpolateLinear(o0.twist.twist.angular.y, o1.twist.twist.angular.y, weight0);
      toReturn.twist.twist.angular.z = interpolateLinear(o0.twist.twist.angular.z, o1.twist.twist.angular.z, weight0);

      return toReturn;
    }

    inline double interpolateLinear(const double v0, const double v1, const double w0) const
    {
      return v0 * w0 + v1 * (1.0 - w0);
    }

    /*geometry_msgs::Quaternion slerp(const geometry_msgs::Quaternion& q0, const geometry_msgs::Quaternion& q1, double w0)
    {
      //https://www.lix.polytechnique.fr/~nielsen/WEBvisualcomputing/programs/slerp.cpp
      float dotproduct = q0.x * q1.x + q0.y * q1.y + q0.z * q1.z + q0.w * q1.w;

      // algorithm adapted from Shoemake's paper
      w0 = w0 / 2.0;

      float theta = (float) acos(dotproduct);
      if (theta < 0.0)
        theta=-theta;
      
      float st = sin(theta);
      float sut = sin(w0 * theta);
      float sout = sin((1-w0) * theta);
      float coeff0 = sout/st;
      float coeff1 = sut/st;

      tf2::Quaternion toReturn;
      toReturn.x = coeff0 * q0.x + coeff1 * q1.x;
      toReturn.y = coeff0 * q0.y + coeff1 * q1.y;
      toReturn.z = coeff0 * q0.z + coeff1 * q1.z;
      toReturn.w = coeff0 * q0.w + coeff1 * q1.w;

      toReturn.normalize();

      return tf2::toMsg(toReturn);
    }*/
  };
} // namespace scitos_common

