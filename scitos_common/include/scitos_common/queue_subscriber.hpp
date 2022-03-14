#pragma once
#include <chrono>
#include <deque>
#include <algorithm>
#include <functional>

#include <ros/subscriber.h>

namespace scitos_common
{
  template <class T> class QueueSubscriber : public ros::Subscriber
  {
    public:
    bool hasReceivedFirst = false;

    QueueSubscriber(){}

    QueueSubscriber(ros::NodeHandle * const nh
                    , const char * const topic
                    , unsigned int queueSize)
    {
      messages = std::deque<std::pair<std::chrono::nanoseconds, T>>(queueSize);
      *static_cast<ros::Subscriber *>(this) = nh->subscribe(topic, 1, &QueueSubscriber<T>::callback, this);
    }

    QueueSubscriber(ros::NodeHandle * const nh
                    , const char * const topic
                    , unsigned int queueSize
                    , std::function<void (boost::shared_ptr<T>)> cb)
                     : cb(cb)
    {
      messages = std::deque<std::pair<std::chrono::nanoseconds, T>>(queueSize);
      *static_cast<ros::Subscriber *>(this) = nh->subscribe(topic, 1, &QueueSubscriber<T>::callback, this);
    }

    void callback(const boost::shared_ptr<T const> & msg){
      hasReceivedFirst = true;
      ros::Time stamp;
      auto stampPointer = ros::message_traits::timeStamp(*msg);
      if (stampPointer)
        stamp = *stampPointer;
      else
        stamp = ros::Time::now();
      //ros::Time stamp = ros::message_traits::HasHeader<T>()? ros::message_traits::timeStamp(*msg) : ros::Time::now();
      messages.push_back(std::make_pair(std::chrono::nanoseconds(stamp.toNSec()), *msg));
      std::sort(messages.begin(), messages.end(), lessThanKey());
    }

    T getNearestOrThrow(std::chrono::nanoseconds timestamp, const char * errorMsg){
      auto value = getNearest(timestamp);
      if(!value.has_value())
        throw errorMsg;
      return value.value();
    }

    std::optional<T> getNearest(std::chrono::nanoseconds timestamp){
      auto lower = std::lower_bound(messages.begin(), messages.end(), timestamp, lessThanKey());
      if (messages.empty())
        return std::nullopt;
      T toReturn;
      if (lower == messages.end())
        toReturn = (*(--lower)).second;
      else if (lower == --messages.end())
        toReturn = (*lower).second;
      else
        toReturn = abs((*(--lower)).first - timestamp) < abs((*(lower)).first - timestamp)? (*(--lower)).second : (*(lower)).second;
      
      //if (cb)
      //  cb(toReturn);

      return toReturn;
    }

    private:
    std::deque<std::pair<std::chrono::nanoseconds, T>> messages;
    std::function<void (boost::shared_ptr<T>)> cb;

    struct lessThanKey{
        inline bool operator() (const std::pair<std::chrono::nanoseconds, T>& a, const std::pair<std::chrono::nanoseconds, T>& b){
            return (a.first < b.first);
        }
    };

  };
} // namespace scitos_common
