#pragma once
#include <chrono>
#include <cmath>

template <typename T> class PID {
public:
  PID()
      : kp_{1.f}, ki_{0.f}, kd_{0.f},
        diffErrAlpha_{0.f}, maxErr_{0.f}, last_{T()}, integratedError_{T()} {}

  PID(float kp, float ki, float kd, float maxErr, float diffErrAlpha)
      : kp_{kp}, ki_{ki}, kd_{kd}, diffErrAlpha_{diffErrAlpha}, maxErr_{maxErr},
        last_{T()}, integratedError_{T()} {}

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
    integratedError_ += error * time;
    if (std::abs(integratedError_) > maxErr_) {
      // Clamp does not work wll with vectors
      integratedError_ = integratedError_ * maxErr_ / static_cast<float>(integratedError_);
    }
    T diffErr = time > 0.f ? (error - last_) / time : T();
    T output = kp_ * error + ki_ * integratedError_ + kd_ * diffErr;
    last_ =
        diffErrAlpha_ * last_ + (1 - diffErrAlpha_) * error; // Exp moving avg
    return output;
  }

private:
  float kp_, ki_, kd_;
  float diffErrAlpha_;
  float maxErr_;
  T last_;
  T integratedError_;
};
