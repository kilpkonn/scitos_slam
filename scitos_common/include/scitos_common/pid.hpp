#pragma once
#include <algorithm>
#include <chrono>

template <typename T> class PID {
public:
  PID()
      : kp_{1.f}, ki_{0.f}, kd_{0.f}, diffErrAlpha_{0.f}, minVal_{T()},
        maxVal_{T()}, maxErr_{T()}, last_{T()}, integratedError_{T()} {}

  PID(float kp, float ki, float kd, T maxErr, T minVal, T maxVal,
      float diffErrAlpha)
      : kp_{kp}, ki_{ki}, kd_{kd}, diffErrAlpha_{diffErrAlpha}, minVal_{minVal},
        maxVal_{maxVal}, maxErr_{maxErr}, last_{T()}, integratedError_{T()} {}

  /*!
   * Accumulate pid
   *
   * This function should be called once every `tick`.
   *
   * Arguments
   * @error - The difference between target and current value
   * @dt - Time since last update
   *
   * @return PID output;
   */
  T accumulate(T error, std::chrono::milliseconds dt) {
    float time = static_cast<float>(dt.count()) / 1000.f;
    integratedError_ += std::clamp(error * time, -maxErr_, maxErr_);
    T diffErr = time > 0.f ? (error - last_) / time : T();
    T output = kp_ * error + ki_ * integratedError_ + kd_ * diffErr;
    last_ =
        diffErrAlpha_ * last_ + (1 - diffErrAlpha_) * error; // Exp moving avg
    return std::clamp(output, minVal_, maxVal_);
  }

private:
  float kp_, ki_, kd_;
  float diffErrAlpha_;
  T minVal_;
  T maxVal_;
  T maxErr_;
  T last_;
  T integratedError_;
};
