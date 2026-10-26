/**
 ******************************************************************************
 * @file    filter.cpp/h
 * @brief   Filters. 滤波算法实现
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef FILTER_H
#define FILTER_H

class LowPassFilter {
 public:
  LowPassFilter(void) : LowPassFilter(1.f, 0) {}
  LowPassFilter(float k) : LowPassFilter(k, 0) {}
  LowPassFilter(float k, float init) : k_(k), output_(init) {}

  float update(float input);
  inline void setK(float k) { k_ = k; }
  inline void reset(float val = 0) {
    input_ = val;
    output_ = val;
  }

 private:
  float k_;
  float input_, output_;
};

// todo: add other filter

#endif  // FILTER_H