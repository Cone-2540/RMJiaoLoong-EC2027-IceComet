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
  LED_BENCH,
};
extern uint32_t led_status;
extern uint8_t arm_step; // 0 空闲，1~8 方形/圆，9/10 任意轨迹，11/12 圆角方形，13/14 交龙，15/16 RM R 预备/绘制
extern volatile bool arm_start_ok;    // 自动动作最近一次启动/续段是否成功

// 无手柄任务二调试；request 最后写入。心跳停止 600 ms 后撤掉电流。
struct BenchDebug {
  volatile uint32_t request; // 1 在当前位置使能位置双环，2 停机
  volatile uint32_t heartbeat;
  uint32_t active, status;  // status: 0 空闲，1 运行，2 主动停机，3 拒绝，4 掉线/故障
  uint32_t fault; // 0 无，1 心跳超时，2 反馈/模式异常，3 检测到持续振荡，4 遥控停机
  uint32_t fault_axes; // 振荡触发轴位图：1=J0，2=J1，3=两轴；停机后保留
};
extern BenchDebug bench_debug;

// 原生 Scope Watch：1 = 该轴目标增加 10°（只执行一次），2 = 双轴停机；读取后自动清零。
// 首次触发进入无手柄任务二位置模式，之后保持位置，便于在线调 PID / Freeze Telemetry。
extern volatile uint32_t j0_step_request;
extern volatile uint32_t j1_step_request;

// Scope 单次试验入口：仅已使能的左上右中保持挡接受 1 开始、2 取消、3 画完后回卷。
// 参数先写，request 最后写；不改遥控数据，不自动使能或标定。
struct SquareDebug {
  volatile uint32_t request;
  volatile uint32_t status;  // 0 空闲，1 运行，2 完成，3 拒绝，4 取消/故障
  float speed_gain[2];      // J0/J1 速度前馈倍率，启动时采纳
  float edge_duration;      // 等效单边用时 s；圆角整圈为 4 倍，启动时锁存
  float moving_deadband[2]; // 正方形/回卷期间的外环死区 °，结束或取消后恢复
  uint32_t cycle_time_ms;   // 板上计时：整圈绘制，不含预检、预备及回卷
  float command_peak_a[2];  // 绘制期间每 1 ms 统计，结束后保留
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

// 内置 Logo 复用绘图 PID、运动死区与前馈；速度在启动时锁存。
struct LogoDebug {
  float speed_mps;          // 大致平均线速度 m/s，默认来自 logo_config.h
  float duration_s;         // 本次规划绘制时间，不含预检和预备
  volatile uint32_t status; // 0 空闲，1 预检/预备，2 绘制，3 完成，4 拒绝，5 取消/故障
  uint32_t cycle_time_ms;   // 本次整条轮廓的板上绘制计时
};
extern LogoDebug logo_debug;
extern LogoDebug rm_r_debug; // RM R 的速度、状态和计时独立，字段定义同上

// 控制初始化
void controlInit(void);
// 控制主循环
void controlLoop(void);

#endif  // CONTROL_H
