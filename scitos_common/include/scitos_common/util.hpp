#pragma once

#include <cmath>

namespace Util
{
  template<typename T>
  inline T normalize_angle(T angle)
  {
    while(angle > M_PI){
        angle -= 2 * M_PI;
    }
    while(angle < -M_PI){
        angle += 2 * M_PI;
    }
    return angle;
  }
}
