/**
 ******************************************************************************
 * @file    io_input.cpp/h
 * @brief   IO(bool) input handle. IO(布尔)输入处理
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "base/common/bool_input.h"
#include "main.h"

// Input source state, handle input status, exeute callback funciton
// 输入触发状态(bool)，处理输入状态，调用回调函数
void BoolInput::handle(bool input_source) {
  // Handle input: edge detection, eliminate dithering
  // 触发处理，边缘检测，消抖
  if (input_source == input_) {
    input_tick_ = HAL_GetTick();
    input_edge_ = UNCHANGED;
  } else if (HAL_GetTick() - input_tick_ < t_input_) {
    input_edge_ = UNCHANGED;
  } else {
    input_ = input_source;
    input_edge_tick_ = HAL_GetTick();
    if (input_) {
      input_edge_ = F2T;  // false to true
      if (inputEdgeCallback != nullptr) {
        // 触发边沿回调
        inputEdgeCallback(true);
      }
    } else {
      input_edge_ = T2F;  // true to false
      if (inputEdgeCallback != nullptr) {
        // 触发边沿回调
        inputEdgeCallback(false);
      }
    }
  }

  // Handle command: manage state, count command
  // 处理指令状态和指令输入
  if (mode_ == NORMAL) {
    // 普通模式触发回调
    if (inputCallback != nullptr) {
      inputCallback(input_);
    }

    if (input_ && HAL_GetTick() - input_edge_tick_ > cmd_timeout_) {
      // 普通输入模式保持触发源为true进入指令输入模式
      mode_ = COMMAND;
      cmd_cnt_ = 0;
      if (cmdStartCallback != nullptr) {
        cmdStartCallback();
      }
    }
  } else if (mode_ == COMMAND) {
    // 指令模式触发回调
    if (cmdInputCallback != nullptr) {
      cmdInputCallback(input_);
    }

    if (!input_ && HAL_GetTick() - input_edge_tick_ > cmd_timeout_) {
      // 指令输入模式保持触发源为false进入普通输入模式
      mode_ = NORMAL;
      if (cmdFinishCallback != nullptr) {
        cmdFinishCallback(cmd_cnt_);
      }
    } else if (input_edge_ == F2T) {
      // 指令输入模式检测到F->T边沿，cmd计数+1
      cmd_cnt_++;
    }
  }
}
