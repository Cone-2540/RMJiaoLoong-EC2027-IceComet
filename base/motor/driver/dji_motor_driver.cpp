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

#include "base/motor/driver/dji_motor_driver.h"
#include "base/common/math.h"
#include "lib/arm_math/arm_math.h"

// Configure motor id. 配置电机id
void DJIMotorDriver::idConfig(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_motor_[i] != nullptr) {
      can1_motor_[i]->CANIdConfig(1, i + 1);
    }
    if (can2_motor_[i] != nullptr) {
      can2_motor_[i]->CANIdConfig(2, i + 1);
    }
  }
}

// Transmit control value package. Called in canTask.
// 控制量打包发送，在CANTask中和其他CAN通信函数统一管理调用
void DJIMotorDriver::canTxMsg(uint8_t can_channel,
                              djimotor::CANIDRange_e id_range) {
  Motor** can_motor;
  if (can_channel == 1) {
    hcan_ = &hcan1;
    can_motor = can1_motor_;
  } else if (can_channel == 2) {
    hcan_ = &hcan2;
    can_motor = can2_motor_;
  } else {
    return;
  }

  // Pack transmit message.
  can_tx_header_.IDE = CAN_ID_STD;
  can_tx_header_.RTR = CAN_RTR_DATA;
  can_tx_header_.DLC = 8;
  if (id_range == djimotor::ID_1_4) {
    can_tx_header_.StdId = 0x200;
    motor_id_base_ = 0;
  } else if (id_range == djimotor::ID_5_8) {
    can_tx_header_.StdId = 0x1ff;
    motor_id_base_ = 4;
  } else if (id_range == djimotor::ID_9_11) {
    can_tx_header_.StdId = 0x2ff;
    motor_id_base_ = 8;
  }
  for (int i = 0; i < 4; i++) {
    if (can_motor[i + motor_id_base_] == nullptr) {
      can_tx_data_[2 * i] = 0;
      can_tx_data_[2 * i + 1] = 0;
    } else {
      can_tx_data_[2 * i] = (can_motor[i + motor_id_base_]->intensity_ >> 8);
      can_tx_data_[2 * i + 1] = (can_motor[i + motor_id_base_]->intensity_);
    }
  }

  // transmit
  HAL_CAN_AddTxMessage(hcan_, &can_tx_header_, can_tx_data_, &can_tx_mail_box_);
}

// Transmit control value package. Called in canTask.
// 校验CAN通道和ID
bool DJIMotorDriver::canRxMsgCheck(CAN_HandleTypeDef* hcan,
                                   CAN_RxHeaderTypeDef rx_header) {
  return rx_header.StdId >= 0x201 && rx_header.StdId <= 0x20b;
}

// Receive feedback data message callback. Called in
// HAL_CAN_RxFifo0MsgPendingCallback()
// 电调反馈信息接收回调，在HAL_CAN_RxFifo0MsgPendingCallback中调用
void DJIMotorDriver::canRxMsgCallback(CAN_HandleTypeDef* hcan,
                                      CAN_RxHeaderTypeDef rx_header,
                                      uint8_t rx_data[8]) {
  // Judge CAN message id
  // 判断CAN数据包id是否属于DJI电机id范围
  if (rx_header.StdId < 0x201 || rx_header.StdId > 0x20b) {
    error_state_ = djimotor::ID_OUTRANGE;
    return;
  }

  // Select motor and raw data ptr according to pack id
  // 选择电机和原始数据结构体指针
  Motor** can_motor;
  uint8_t id = rx_header.StdId - 0x200;
  djimotor::RawData_t* raw_data;
  if (hcan == &hcan1) {
    can_motor = can1_motor_;
    raw_data = &can1_raw_data_[id - 1];
  } else if (hcan == &hcan2) {
    can_motor = can2_motor_;
    raw_data = &can2_raw_data_[id - 1];
  }

  // Update raw data
  // 更新原始数据
  raw_data->ecd = (uint16_t)(rx_data[0] << 8 | rx_data[1]);
  raw_data->rotate_speed_rpm = (uint16_t)(rx_data[2] << 8 | rx_data[3]);
  raw_data->torq_current = (uint16_t)(rx_data[4] << 8 | rx_data[5]);
  raw_data->temp = rx_data[6];

  // Get Motor entity's pointer(return if motor undefined)
  // 获取Motor对象指针
  if (can_motor[id - 1] == nullptr) {
    error_state_ = djimotor::MOTOR_UNDEFINED;
    return;
  }
  Motor* m = can_motor[id - 1];

  // Update motor feedback data (raw_data->motor_data)
  // 电机反馈数据更新 (raw_data->motor_data)
  if (m->info_.type == Motor::M3508 || m->info_.type == Motor::M2006 ||
      m->info_.type == Motor::GM6020) {
    // Encoder angle(deg)
    // 编码器角度
    m->motor_data_.ecd_angle = math::ecd2deg(raw_data->ecd, 8192);
    // Use incremental calculation to deals with encoder overflow and underflow
    // 增量计算处理编码器上下溢问题
    float delta = m->motor_data_.ecd_angle - m->motor_data_.last_ecd_angle;
    delta = math::degNormalize180(delta) / m->ratio_;
    m->motor_data_.angle += delta;  // deg
    // feedback rotational speed
    // 反馈转速
    m->motor_data_.rotate_speed =
        math::rpm2dps(raw_data->rotate_speed_rpm) / m->ratio_;  // dps
    // Update current and temperature
    // 更新转矩电流和温度
    m->motor_data_.current = raw_data->torq_current;
    m->motor_data_.temp = raw_data->temp;
    // update encoder angle record
    // 更新编码器角度记录
    m->motor_data_.last_ecd_angle = m->motor_data_.ecd_angle;
  } else {
    error_state_ = djimotor::TYPE_MISMATCH;
  }

  m->rxCallback();
  error_state_ = djimotor::NO_ERROR;
}
