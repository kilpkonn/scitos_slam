#pragma once

#include <cmath>

namespace Util
{
  template<typename T>
  inline T normalize_angle(T angle)
  {
    while(angle > 2 * M_PI){
        angle -= M_PI;
    }
    while(angle < -2 * M_PI){
        angle += M_PI;
    }
    return angle;
  }
}
