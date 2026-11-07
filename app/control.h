/**
 ******************************************************************************
 * @file    control.cpp/h
 * @brief   Robot control design. 机器人控制设计（模式/键位）
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

// C 板 RGB 灯状态；Scope 中可查看 led_status。
enum BoardLedStatus {
  LED_RC_LOST = 0,
  LED_STOPPED,
  LED_J0_LOST,
  LED_J1_LOST,
  LED_BOTH_MOTORS_LOST,
  LED_FEEDBACK_INVALID,
  LED_NEED_CALIBRATION,
  LED_CALIBRATED,
  LED_DISABLED,
  LED_POSITION,
  LED_SPEED,
  LED_CURRENT,
  LED_HOLDING,
};
extern uint32_t led_status;

// 控制初始化
void controlInit(void);
// 控制主循环
void controlLoop(void);

#endif  // CONTROL_H
