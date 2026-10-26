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

#ifndef CONNECT_H
#define CONNECT_H

#include <stdint.h>

// Connect edge(change) judgement 连接状态变化边沿
typedef enum ConnectEdge {
  UNCHANGED,   // unchanged
  CONNECT,     // unconnect->connect
  DISCONNECT,  // connect->unconnect
} ConnectEdge_e;

// Connect class 连接类
class Connect {
 public:
  Connect(uint32_t timeout)
      : flag_(false), last_tick_(0), timeout_(timeout), freq_(0) {}

  // Check connect status. Return true-connected, false-unconnected
  // 检测连接状态
  bool check(void);

  // Refresh connect status(Commonly called in rx callback).
  // Return connet edge(state change)
  // 刷新连接状态(一般在接收回调函数调用)，返回连接状态的变化
  ConnectEdge_e refresh(void);

  // Last refresh tick
  // 最后一次刷新时间
  uint32_t lastTick(void) { return last_tick_; }

  // connect frequency
  // 连接频率
  float freq(void) { return freq_; }

 private:
  bool flag_;           // connect state flag. true-connected, false-unconnected
  ConnectEdge_e edge_;  // connect edge
  uint32_t last_tick_;  // last update tick(system clock, ms)
  uint32_t timeout_;    // t>timeout, flag=false
  float freq_;          // frequency(Hz)
};

#endif  // CONNECT_H
