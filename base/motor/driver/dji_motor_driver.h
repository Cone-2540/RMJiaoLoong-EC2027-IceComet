/**
 ******************************************************************************
 * @file    dji_motor_driver.cpp/h
 * @brief   DJI motor driver(M3508, M2006, GM6020, etc). DJI电机驱动
 * @author  Spoon Guan
 * @note
 * 不同型号电机存在差异，如控制电流/电压值范围不同(M2006:[-10000,10000]，3508:[-16384，16384]，
 * GM6020:[-30000,30000])，M2006电机没有力矩电流和温度反馈，使用前请阅读电机电调相关文档
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef DJI_MOTOR_DRIVER_H
#define DJI_MOTOR_DRIVER_H

#include "can.h"
#include "hardware_config.h"

#include "base/motor/motor.h"

namespace djimotor {

typedef enum CANIDRange {
  ID_1_4,
  ID_5_8,
  ID_9_11,
} CANIDRange_e;

typedef enum CANRxError {
  NO_ERROR,
  ID_OUTRANGE,
  MOTOR_UNDEFINED,
  TYPE_MISMATCH,
} CANRxError_e;

// 电调反馈数据
typedef struct RawData {
  int16_t ecd;               // encoder value(0~8191) 编码器值(0~8191)
  int16_t rotate_speed_rpm;  // rotational speed(rpm) 转速(单位rpm)
  int16_t torq_current;      // torque current 转矩电流
  int8_t temp;               // temperature 温度
} RawData_t;

};  // namespace djimotor

// DJI motor driver class
// 大疆电机驱动封装
class DJIMotorDriver {
 public:
  DJIMotorDriver(Motor* can1_motor[11], Motor* can2_motor[11]) {
    memcpy(can1_motor_, can1_motor, 11 * sizeof(Motor*));
    memcpy(can2_motor_, can2_motor, 11 * sizeof(Motor*));
  }

  // Configure motor id.
  // 配置电机id
  void idConfig(void);

  // Transmit control value package. Called in canTask.
  // 控制量打包发送，在CANTask中和其他CAN通信函数统一管理调用
  void canTxMsg(uint8_t can_channel, djimotor::CANIDRange_e id_range);

  // Check CAN channel and CAN message header id
  // 校验CAN通道和ID
  bool canRxMsgCheck(CAN_HandleTypeDef* hcan, CAN_RxHeaderTypeDef rx_header);

  // Receive feedback data message callback. Called in
  // HAL_CAN_RxFifo0MsgPendingCallback()
  // 电调反馈信息接受回调，在HAL_CAN_RxFifo0MsgPendingCallback中调用
  void canRxMsgCallback(CAN_HandleTypeDef* hcan, CAN_RxHeaderTypeDef rx_header,
                        uint8_t rx_data[8]);

 private:
  // Motor pointer group
  Motor* can1_motor_[11];
  Motor* can2_motor_[11];

  // transmit
  CAN_HandleTypeDef* hcan_;
  CAN_TxHeaderTypeDef can_tx_header_;
  uint8_t can_tx_data_[8];
  uint32_t can_tx_mail_box_;
  uint32_t motor_id_base_;

  // receive
  uint32_t motor_id_;
  djimotor::RawData_t can1_raw_data_[11];  // 原始数据
  djimotor::RawData_t can2_raw_data_[11];  // 原始数据
  djimotor::CANRxError_e error_state_;
};

#endif  // DJI_MOTOR_DRIVER_H
