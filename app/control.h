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
#include "app/arm_trajectory.h"

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
extern uint8_t arm_step; // 0 空闲，1~8 尖角方形/圆，9/10 任意轨迹，11/12 圆角方形预备/绘制
extern volatile bool arm_start_ok;    // 自动动作最近一次启动/续段是否成功

// Scope 单次试验入口：仅已使能的左上右中保持挡接受 1 开始、2 取消、3 画完后回卷。
// 参数先写，request 最后写；不改遥控数据，不自动使能或标定。
struct SquareDebug {
  volatile uint32_t request;
  volatile uint32_t status;  // 0 空闲，1 运行，2 完成，3 拒绝，4 取消/故障
  float speed_gain[2];      // J0/J1 速度前馈倍率，启动时采纳
  float edge_duration;      // 等效单边用时 s；圆角整圈为 4 倍，启动时锁存
  float moving_deadband[2]; // 正方形/回卷期间的外环死区 °，结束或取消后恢复
  uint32_t cycle_time_ms;   // 板上计时：第一边启动到第四边结束，不含预备及回卷
  float command_peak_a[2];  // 绘制四边期间每 1 ms 统计，结束后保留
  float feedback_peak_a[2]; // 电调反馈电流峰值，与指令峰值分开记录
  float corner_radius;     // 四角共用圆角半径 m，0 为尖角，启动锁存；追加以保留原成员偏移
};
extern SquareDebug square_debug;

// 网页经 Scope 上传；upload_lock 先置 1，count 最后提交，运行中禁止修改。
struct FreehandPath {
  uint32_t version;
  volatile uint32_t upload_lock;
  volatile uint32_t count;
  float speed_mps;
  volatile uint32_t status; // 0 未运行，1 预备，2 绘制，3 完成，4 拒绝，5 取消/故障
  float duration_s;        // 绘制用时，不含到起点的预备动作
  ArmTrajectory::Point points[ArmTrajectory::max_path_points];
};
extern FreehandPath freehand_path;

// 控制初始化
void controlInit(void);
// 控制主循环
void controlLoop(void);

#endif  // CONTROL_H
