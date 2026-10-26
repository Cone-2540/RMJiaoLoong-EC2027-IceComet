/**
 ******************************************************************************
 * @file    filter.cpp/h
 * @brief   Filters. 滤波算法实现
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "filter.h"

float LowPassFilter::update(float input) {
  input_ = input;
  output_ = output_ * (1 - k_) + input_ * k_;
  return output_;
}
