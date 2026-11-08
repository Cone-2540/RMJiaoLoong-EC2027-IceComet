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
  LED_SQUARE,
};
extern uint32_t led_status;
extern uint8_t arm_step;              // 0 空闲，1 预备，2/3/4/5 前往 B/C/D/A，6 调试回卷
extern volatile bool arm_start_ok;    // 自动动作最近一次启动/续段是否成功

// Scope 单次试验入口：仅已使能的左上右中保持挡接受 1 开始、2 取消、3 画完后回卷。
// 参数先写，request 最后写；不改遥控数据，不自动使能或标定。
struct SquareDebug {
  volatile uint32_t request;
  volatile uint32_t status;  // 0 空闲，1 运行，2 完成，3 拒绝，4 取消/故障
  float speed_gain[2];      // J0/J1 速度前馈倍率，启动时采纳
};
extern SquareDebug square_debug;

// 控制初始化
void controlInit(void);
// 控制主循环
void controlLoop(void);

#endif  // CONTROL_H
