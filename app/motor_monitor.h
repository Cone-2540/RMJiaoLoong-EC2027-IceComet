/**
 ******************************************************************************
 * @file    motor_monitor.cpp/h
 * @brief   Motor parameter, id config and management. 电机参数id配置和统一管理
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef MOTOR_MONITOR_H
#define MOTOR_MONITOR_H

#include "base/motor/driver/dji_motor_driver.h"
#include "base/motor/driver/mit_motor_driver.h"
#include "base/motor/motor.h"

// Manage all motors' mode. Called in controlTask.
// 统一操作电机模式，在controlTask中调用
void allMotorsStopShutoff(void);
void allMotorsShutOff(void);
void allMotorsStop(void);
void allMotorsOn(void);

// Reset all motors.
// 电机统一初始化
void allMotorsInit(void);

// Handle all motors. Called after Arm::handle() in controlTask
// 电机统一处理
void allMotorsHandle(void);

// 左上右上使用任务二参数；退出后恢复原 PID 和 KF 参数。
void setJointStepProfile(bool enabled);

// Check CAN channel and id of received CAN message
// 校验接收信息的CAN通道和ID，调用对应回调函数
void motorsCanRxMsgHandle(CAN_HandleTypeDef* hcan,
                          CAN_RxHeaderTypeDef rx_header, uint8_t* rx_data);

// extern Motor CMFL, CMFR, CMBL, CMBR, GMY, GMP, FRICL, FRICR, STIR, m1;
extern Motor J0, J1;
extern Motor* can1_dji_motor[11];
extern Motor* can2_dji_motor[11];

// Scope 观测值：角度 deg，速度 deg/s，电流指令 A，反馈电流保留原始值。
struct MotorDebugData {
  float target_deg, angle_deg, error_deg;
  float target_dps, speed_dps, raw_speed_dps;
  float command_a, feedback_current_raw, temperature;
};
extern MotorDebugData j0_debug, j1_debug;
extern uint32_t manual_mode, calibration_ok, rc_connected;

// 板上记录控制周期，绕过 ST-Link/HTTP 的采样延迟；只记录，不生成运动。
struct StepTraceSample {
  uint32_t tick_ms;
  float target[2], angle[2], raw_angle[2], speed[2], current[2];
};
struct StepTrace {
  uint32_t request;  // 0 不请求，1 立即采样，2 等待目标角跳变后采样
  uint32_t status;   // 0 空闲，1 等待阶跃，2 采集中，3 完成，4 模式退出中止
  uint32_t count, stride; // stride=1 为 1 kHz；改成 4 可记录约 1 s
  uint32_t monitor_status, monitor_ms, start_ms, elapsed_ms, max_gap_ms, last_outside_ms[2];
  float band_deg; // 连续稳定判据，默认任务二 2%：10° × 2% = 0.2°
  float max_error_after_100ms[2], peak_current[2];
  StepTraceSample samples[256];
};
extern StepTrace step_trace;

struct StepFeedforward {
  float positive_a[2], negative_a[2], applied_a[2]; // 单向阶跃保持前馈，A；默认 0
};
extern StepFeedforward step_feedforward;

#endif  // MOTOR_MONITOR_H
