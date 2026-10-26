/**
 ******************************************************************************
 * @file    pid.cpp/h
 * @brief   PID algorithm. PID算法实现
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "pid.h"

#include <cmath>
#include "math.h"

PID::PID(float kp, float ki, float kd, float i_max, float out_max,
         float d_filter_k)
    : kp_(kp),
      ki_(ki),
      kd_(kd),
      i_max_(i_max),
      out_max_(out_max),
      d_filter_(d_filter_k) {
  err_ = 0;
  err_sum_ = 0;
  last_err_ = 0;
}

void PID::reset(void) {
  err_ = 0;
  err_sum_ = 0;
  last_err_ = 0;
  pout_ = 0;
  iout_ = 0;
  dout_ = 0;
  d_filter_.reset();
}

float PID::calc(float ref, float fdb) {
  ref_ = ref;
  fdb_ = fdb;

  last_err_ = err_;
  err_ = ref_ - fdb_;
  err_sum_ = math::limit(err_sum_ + err_, -i_max_, i_max_);

  pout_ = kp_ * err_;
  iout_ = ki_ * err_sum_;
  dout_ = d_filter_.update(kd_ * (err_ - last_err_));
  output_ = math::limit(pout_ + iout_ + dout_, -out_max_, out_max_);

  return output_;
}

PIDEx::PIDEx(float kp, float ki, float kd, float i_max, float out_max,
             float d_filter_k)
    : PID(kp, ki, kd, i_max, out_max, d_filter_k) {
  err_ = 0;
  err_sum_ = 0;
  last_err_ = 0;
  if_trapezoidal_integrator_ = false;
  if_derivative_on_feedback_ = false;
  if_different_integration_rate_ = false;
}

PIDEx::PIDEx(float kp, float ki, float kd, float i_max, float out_max,
             bool if_trapezoidal_integrator, bool if_derivative_on_feedback,
             bool if_different_integration_rate,
             float integrator_lower_threshold, float integrator_upper_threshold,
             float d_filter_k)
    : PID(kp, ki, kd, i_max, out_max, d_filter_k),
      if_trapezoidal_integrator_(if_trapezoidal_integrator),
      if_derivative_on_feedback_(if_derivative_on_feedback),
      if_different_integration_rate_(if_different_integration_rate),
      i_lower_thres_(integrator_lower_threshold),
      i_upper_thres_(integrator_upper_threshold) {
  err_ = 0;
  err_sum_ = 0;
  last_err_ = 0;
}

float PIDEx::calc(float ref, float fdb) {
  last_fdb_ = fdb_;

  ref_ = ref;
  fdb_ = fdb;

  last_err_ = err_;
  err_ = ref_ - fdb_;

  float i_input;
  // 梯形积分，优化低频率情况下的 PID
  if (if_trapezoidal_integrator_) {
    i_input = (last_err_ + err_) / 2.0f;
  } else {
    i_input = err_;
  }

  // 变速积分，优化目标大幅度变化时积分项大幅度上升的情况
  if (if_different_integration_rate_) {
    if (fabs(i_input) < i_lower_thres_) {
      err_sum_ = math::limit(err_sum_ + i_input, -i_max_, i_max_);
    } else if (fabs(i_input) < i_upper_thres_) {
      // 超过 integrator_upper_threshold_ 的项不积分
      err_sum_ =
          math::limit(err_sum_ + i_input * (fabs(i_input) - i_lower_thres_) /
                                     (i_upper_thres_ - i_lower_thres_),
                      -i_max_, i_max_);
    }
  }

  pout_ = kp_ * err_;
  iout_ = ki_ * err_sum_;
  if (if_derivative_on_feedback_) {
    // 微分先行，优化目标大幅度变化时微分项可能导致尖峰的情况
    dout_ = d_filter_.update(kd_ * (fdb_ - last_fdb_));
  } else {
    dout_ = d_filter_.update(kd_ * (err_ - last_err_));
  }
  output_ = math::limit(pout_ + iout_ + dout_, -out_max_, out_max_);

  return output_;
}