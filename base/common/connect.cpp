/**
 ******************************************************************************
 * @file    connect.cpp/h
 * @brief   Connect state handle. 连接状态处理
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "base/common/connect.h"
#include "main.h"

// Check connect status. Return true-connected, false-unconnected
// 检测连接状态
bool Connect::check(void) {
  if (HAL_GetTick() - last_tick_ > timeout_) {
    if (flag_) {
      edge_ = DISCONNECT;
      freq_ = 0;
    } else {
      edge_ = UNCHANGED;
    }
    flag_ = false;
  } else {
    if (flag_) {
      edge_ = UNCHANGED;
    }
  }
  return flag_;
}

// Refresh connect status(Commonly called in rx callback).
// Return true-unconnected->connected, false-keep connected
// 刷新连接状态(一般在接收回调函数调用)
// 返回 true-未连接->连接，false-保持连接
ConnectEdge_e Connect::refresh(void) {
  if (!flag_) {
    edge_ = CONNECT;
    flag_ = true;
  } else {
    edge_ = UNCHANGED;
  }
  if (HAL_GetTick() - last_tick_ < 1) {
    freq_ = 1e3f;
  } else {
    freq_ = 1e3f / (HAL_GetTick() - last_tick_);
  }
  last_tick_ = HAL_GetTick();
  return edge_;
}
