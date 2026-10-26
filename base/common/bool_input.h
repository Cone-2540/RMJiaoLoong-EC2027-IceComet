/**
 ******************************************************************************
 * @file    bool_input.cpp/h
 * @brief   Bool input handle. 布尔输入处理
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef BOOL_INPUT_H
#define BOOL_INPUT_H

#include <stdint.h>

// IO(bool) input class IO(布尔)输入类
class BoolInput {
 public:
 public:
  BoolInput(uint32_t timeout = UINT32_MAX, uint32_t t_input = 10)
      : mode_(NORMAL),
        input_(false),
        input_edge_(UNCHANGED),
        input_tick_(0),
        input_edge_tick_(0),
        t_input_(t_input),
        cmd_cnt_(0),
        cmd_timeout_(timeout) {}

  // Input input state. handle input status, exeute callback funciton
  // 输入触发状态(bool)，处理输入状态，调用回调函数
  void handle(bool input_source);

 public:
  // Callback function
  void (*inputCallback)(bool);      // 输入状态回调
  void (*inputEdgeCallback)(bool);  // 输入边沿回调
  void (*cmdInputCallback)(bool);   // 指令内输入状态回调

  void (*cmdStartCallback)(void);      // 指令开始回调
  void (*cmdFinishCallback)(uint8_t);  // 指令结束回调

 private:
  // IO input mode
  enum InputMode {
    NORMAL,   // 普通输入
    COMMAND,  // 指令输入
  } mode_;

  bool input_;      // input status
  enum InputEdge {  // input edge
    UNCHANGED,
    F2T,  // false to true
    T2F,  // true to false
  } input_edge_;
  uint32_t input_tick_;       // tick of input (ms)
  uint32_t input_edge_tick_;  // tick of input edge (ms)
  uint32_t t_input_;          // time for eliminate dithering (ms)

  uint8_t cmd_cnt_;       // count for command input
  uint32_t cmd_timeout_;  // command input timeout
};

#endif  // BOOL_INPUT_H