/**
 ******************************************************************************
 * @file    pid.cpp/h
 * @brief   PID algorithm. PID算法实现
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef PID_H
#define PID_H

#include "base/common/filter.h"

// PID类
class PID {
 public:
  PID(void) : PID(0, 0, 0, 0, 0) {}
  PID(float kp, float ki, float kd, float i_max, float out_max,
      float d_filter_k = 1);

  void reset(void);
  float calc(float ref, float fdb);

 public:
  float kp_, ki_, kd_;
  float i_max_, out_max_;
  float output_;

  // 微分项低通滤波器
  LowPassFilter d_filter_;

 protected:
  float ref_, fdb_;
  float err_, err_sum_, last_err_;
  float pout_, iout_, dout_;
};

// 改进PID类
class PIDEx : public PID {
 public:
  PIDEx(void) : PID(0, 0, 0, 0, 0) {}
  PIDEx(float kp, float ki, float kd, float i_max, float out_max,
        float d_filter_k = 0.1f);
  PIDEx(float kp, float ki, float kd, float i_max, float out_max,
        bool if_trapezoidal_integrator, bool if_derivative_on_feedback,
        bool if_different_integration_rate, float integrator_lower_threshold,
        float integrator_upper_threshold, float d_filter_k = 0.1f);

  float calc(float ref, float fdb);

 private:
  float last_fdb_;

  bool if_trapezoidal_integrator_;
  bool if_derivative_on_feedback_;
  bool if_different_integration_rate_;
  float i_lower_thres_, i_upper_thres_;
};

#endif  // PID_H
