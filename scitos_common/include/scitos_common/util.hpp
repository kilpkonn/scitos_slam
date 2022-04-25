#pragma once

#include <cmath>

namespace util
{
  template<typename T>
  inline T normalize_angle(T angle)
  {
    while(angle > M_PI){
        angle -= M_PI;
    }
    while(angle < -M_PI){
        angle += M_PI;
    }
    return angle;
  }
}
