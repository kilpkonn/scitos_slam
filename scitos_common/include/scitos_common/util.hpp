#pragma once

#include <cmath>

namespace Util
{
  inline double normalize_angle(double angle)
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