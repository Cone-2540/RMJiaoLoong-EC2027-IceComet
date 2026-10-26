/**
 ******************************************************************************
 * @file    motor.cpp/h
 * @brief   Motor control. 电机控制
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "base/common/connect.h"
#include "base/common/kalman_filter.h"
#include "base/common/pid.h"

class Motor {
 public:
  // 电机型号
  typedef enum Type {
    // DJI motor
    M3508,
    M2006,
    GM6020,
    // other motor
    MIT,
  } Type_e;

  typedef enum ControlMethod {
    POSITION_SPEED,  // 位置+速度控制
    POSITION,        // 位置控制
    SPEED,           // 速度控制
    TORQUE,          // 力矩控制
  } ControlMethod_e;

  typedef enum Mode {
    POWEROFF,  // 断电
    STOP,      // 停止
    INIT,      // 初始化
    WORKING,   // 工作
  } Mode_e;

  // motor kalman filter parameters(Q & R)
  typedef struct KFParam {
    KFParam(const float& Q11, const float& Q22, const float& R11) {
      Q[0] = Q11;
      Q[1] = 0;
      Q[2] = 0;
      Q[3] = Q22;
      R[0] = R11;
    }

    float Q[4];
    float R[1];
  } KFParam_t;

 public:
  Motor(const Type_e& type, const float& ratio, const ControlMethod_e& method,
        const PID& ppid, const PID& spid, bool use_kf = false,
        const KFParam_t& kf_param = KFParam_t(2, 1e4, 1),
        float (*model)(const float&, const float&) = nullptr);

  // Reset motor
  // 初始化/重置电机
  void reset(void);

  // Receive data callback, update connect status
  // 反馈信息接受回调，更新连接状态
  void rxCallback(void);

  // Control
  // 电机控制
  void handle(void);

  // Configure CAN id
  // 配置CAN通道和id
  void CANIdConfig(const uint8_t& can_channel, const uint8_t& id);

  // Set target angle, feedforward speed/intensity
  // 设置目标角度，角速度/输出前馈
  void setAngleSpeed(const float& target_angle, const float& ff_speed = 0,
                     const float& ff_intensity = 0);

  // Set target angle, feedforward intensity
  // 设置目标角度，输出前馈
  void setAngle(const float& target_angle, const float& ff_intensity = 0);

  // Set target speed, feedforward intensity
  // 设置目标角速度，输出前馈
  void setSpeed(const float& target_speed, const float& ff_intensity = 0);

  // Set control feedback data source (source unit should be: deg, dps)
  // 设置控制反馈数据源 (数据源变量单位必须为: deg, dps)
  // nullptr: use motor_data_ as feedback source
  // 空指针: 使用motor_data_作为反馈数据源
  void setFdbSrc(float* fdb_angle_src, float* fdb_speed_src);

  // Reset control feedback angle to specific value
  // 将控制反馈角度(control_data_.angle)重置为特定角度
  void resetFeedbackAngle(const float& angle);

  // 旧版框架代码风格接口
  float& targetAngle(void) { return control_data_.target_angle; }
  float& targetSpeed(void) { return control_data_.target_speed; }
  float& targetTorque(void) { return control_data_.target_torque; }
  float& realAngle(void) { return control_data_.fdb_angle; }
  float& realSpeed(void) { return control_data_.fdb_speed; }

  // Kalman filter state data
  // 卡尔曼滤波状态量
  float& kfAngle(void) { return kf_data_.x[0]; }
  float& kfSpeed(void) { return kf_data_.x[1]; }

 public:
  struct MotoInfo {
    Motor::Type_e type;   // 型号
    uint8_t can_channel;  // CAN通道1/2
    uint8_t id;           // id
    float max_intensity;  // 控制信号限幅
  } info_;

  Connect connect_;
  Motor::Mode_e mode_;
  Motor::ControlMethod_e method_;
  float ratio_;  // 减速比

  // control value 输出控制量
  float intensity_float_;
  int16_t intensity_;

  // control data 电机控制数据(目标&反馈)
  struct ControlData {
    // unit: deg, dps, (N*m or none)
    // target 目标值
    float target_angle, target_speed, target_torque;
    // feedback 反馈值
    float fdb_angle, fdb_speed, last_fdb_angle;
    // feedback source 反馈数据源
    float *fdb_angle_src, *fdb_speed_src;
    // feedforward rotational speed / control value 前馈
    float feedforward_speed, feedforward_intensity;
  } control_data_;

  // motor data 电机自身角度/转速信息，在update中更新
  struct MotorData {
    float angle;           // deg 电机累计角度
    float ecd_angle;       // deg 编码器角度
    float last_ecd_angle;  // deg 前一次编码器角度
    float rotate_speed;    // dps, 反馈角速度
    float current;         // 转矩电流反馈
    float temp;            // 温度反馈
  } motor_data_;

  // algorithm 控制算法
  PID ppid_, spid_;  // 位置/速度pid
  // Kalman filter 卡尔曼滤波相关
  bool use_kf_ = false;
  KalmanFilter kf_;
  struct KalmanFilterData_t {
    float x[2];
    float u[1];
    float z[2];
  } kf_data_;

  // output model, control_value = model(torque,rotate_speed,...)
  // 电机输出模型
  float (*model_)(const float&, const float&);  // todo
};

#endif  // MOTOR_H
