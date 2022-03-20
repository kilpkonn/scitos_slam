#pragma once
#include <chrono>
#include <deque>
#include <algorithm>
#include <functional>

#include <ros/subscriber.h>

namespace scitos_common
{
  template <class T> class QueueSubscriber: ros::Subscriber
  {
    public:
    bool hasReceivedFirst = false;
    std::function<void (T)> cb;
    std::deque<std::pair<std::chrono::nanoseconds, T>> messages;
    uint queueSize = 1;

    QueueSubscriber(){}

    QueueSubscriber(ros::NodeHandle * const nh
                    , const char * const topic
                    , unsigned int queueSize) : queueSize(queueSize)
    {
      *static_cast<ros::Subscriber *>(this) = nh->subscribe(topic, 1, &QueueSubscriber<T>::callback, this);
      messages = std::deque<std::pair<std::chrono::nanoseconds, T>>(queueSize);
    }

    QueueSubscriber(ros::NodeHandle * const nh
                    , const char * const topic
                    , unsigned int queueSize
                    , std::function<void (T)> cb)
                     : cb(cb), queueSize(queueSize)
    {
      *static_cast<ros::Subscriber *>(this) = nh->subscribe<nav_msgs::Odometry>(topic, 1, &QueueSubscriber<T>::callback, this);
      messages = std::deque<std::pair<std::chrono::nanoseconds, T>>(queueSize);
    }

    void callback(const boost::shared_ptr<T const> & msg){
      if (messages.size() > queueSize)
      {
        return;
      }
      hasReceivedFirst = true;
      ros::Time stamp;
      auto stampPointer = ros::message_traits::timeStamp(*msg);
      if (stampPointer)
      {
        stamp = *stampPointer;
      }
      else
        stamp = ros::Time::now();
      auto p = std::make_pair(std::chrono::nanoseconds(stamp.toNSec()), *msg);

      messages.push_back(std::move(p));
      while(messages.size() > queueSize)
        messages.pop_front();
      std::sort(messages.begin(), messages.end(), lessThanKey());
    }

    T getNearestOrThrow(std::chrono::nanoseconds timestamp, const char * errorMsg){
      auto value = getNearest(timestamp);
      if(!value.has_value())
        throw errorMsg;
      return value.value();
    }

    std::optional<T> getNearest(std::chrono::nanoseconds timestamp){
      auto lower = std::lower_bound(messages.begin(), messages.end(), std::make_pair(timestamp, T()), lessThanKey());
      if (messages.empty())
        return std::nullopt;
      T toReturn;
      if (lower == messages.end()){
        toReturn = (*(--lower)).second;
      }
      else{
        toReturn = abs((*(std::prev(lower))).first - timestamp) < abs((*(lower)).first - timestamp)? (*(--lower)).second : (*(lower)).second;
      }

      if (cb)
        cb(toReturn);

      return toReturn;
    }

    private:
    struct lessThanKey{
        inline bool operator() (const std::pair<std::chrono::nanoseconds, T>& a, const std::pair<std::chrono::nanoseconds, T>& b){
            return (a.first < b.first);
        }
    };

  };
} // namespace scitos_common
