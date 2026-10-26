/**
 ******************************************************************************
 * @file    mit_motor_driver.cpp/h
 * @brief   Driver program for motor use MIT protocol. MIT协议电机驱动
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "base/motor/driver/mit_motor_driver.h"
#include "base/common/math.h"

namespace mitmotor {
// Convert float to unsigned int, given range and number of bits
uint16_t float2uint(float x, float x_min, float x_max, uint8_t bits);
// Convert unsigned int to float, given range and number of bits
float uint2float(int x_int, float x_min, float x_max, int bits);
};  // namespace mitmotor

MITMotorDriver::MITMotorDriver(Motor* motor, CAN_HandleTypeDef* hcan,
                               uint32_t master_id, uint32_t slave_id,
                               float p_min, float p_max, float v_min,
                               float v_max, float kp_min, float kp_max,
                               float kv_min, float kv_max, float t_ff_min,
                               float t_ff_max, float t_min, float t_max)
    : motor_(motor), hcan_(hcan), master_id_(master_id), slave_id_(slave_id) {
  if (hcan_ == &hcan1) {
    motor_->CANIdConfig(1, slave_id_);
  } else if (hcan_ == &hcan2) {
    motor_->CANIdConfig(2, slave_id_);
  }
  param_.p_min = p_min;
  param_.p_max = p_max;
  param_.v_min = v_min;
  param_.v_max = v_max;
  param_.kp_min = kp_min;
  param_.kp_max = kp_max;
  param_.kv_min = kv_min;
  param_.kv_max = kv_max;
  param_.t_ff_min = t_ff_min;
  param_.t_ff_max = t_ff_max;
  param_.t_min = t_min;
  param_.t_max = t_max;
}

// deal with error situations and call motor::handle
void MITMotorDriver::handle() {
  motor_->handle();
  if (rx_data_.state != mitmotor::DISABLE &&
      rx_data_.state != mitmotor::ENABLE) {
    setCmd(mitmotor::CLEAR_ERROR);
  }
  if (!motor_->connect_.check() || rx_data_.state != mitmotor::ENABLE) {
    setCmd(mitmotor::MOTOR_MODE);
  }
}

// Set motor mode command
bool MITMotorDriver::setCmd(mitmotor::CmdType_e cmd) {
  if (cmd_.cnt > 5 || (cmd_.cnt > 0 && cmd_.list[cmd_.cnt - 1] == cmd)) {
    return false;
  }
  cmd_.list[cmd_.cnt] = cmd;
  cmd_.cnt++;
  return true;
}

// Set motor control parameter
// p: target position(rad)
// v: target velocity(rad/s)
// kp/kv: control gain
// t: feedforward torque
// T = kp*(p-p_fdb)+kv*(v-v_fdb)+t
void MITMotorDriver::setControlParam(float p, float v, float kp, float kv,
                                     float t_ff) {
  // control parameter
  p = math::limit(p, param_.p_min, param_.p_max);
  v = math::limit(v, param_.v_min, param_.v_max);
  kp = math::limit(kp, param_.kp_min, param_.kp_max);
  kv = math::limit(kv, param_.kv_min, param_.kv_max);
  t_ff = math::limit(t_ff, param_.t_ff_min, param_.t_ff_max);

  // tx data pack
  tx_data_.p = mitmotor::float2uint(p, param_.p_min, param_.p_max, 16);
  tx_data_.v = mitmotor::float2uint(v, param_.v_min, param_.v_max, 12);
  tx_data_.kp = mitmotor::float2uint(kp, param_.kp_min, param_.kp_max, 12);
  tx_data_.kv = mitmotor::float2uint(kv, param_.kv_min, param_.kv_max, 12);
  tx_data_.t_ff =
      mitmotor::float2uint(t_ff, param_.t_ff_min, param_.t_ff_max, 12);
}

// Set torque
void MITMotorDriver::setTorque(const float& torque) {
  setControlParam(0, 0, 0, 0, torque);
}

// Transmit CAN message
void MITMotorDriver::canTxMsg(void) {
  // can message header
  can_tx_header_.IDE = CAN_ID_STD;
  can_tx_header_.RTR = CAN_RTR_DATA;
  can_tx_header_.DLC = 8;
  can_tx_header_.StdId = slave_id_;

  // transmit command pack
  if (cmd_.cnt > 0) {
    if (HAL_GetTick() - cmd_.tick > cmd_.interval) {
      setTorque(0);
      memset(can_tx_data_, 0xff, 8);
      can_tx_data_[7] = cmd_.list[0];
      HAL_CAN_AddTxMessage(hcan_, &can_tx_header_, can_tx_data_,
                           &can_tx_mail_box_);
      memmove(cmd_.list, (mitmotor::CmdType_e*)cmd_.list + 1,
              4 * sizeof(mitmotor::CmdType_e));
      cmd_.list[cmd_.cnt - 1] = mitmotor::NO_CMD;
      cmd_.cnt--;
      cmd_.tick = HAL_GetTick();
    } else {
      setTorque(0);
      can_tx_data_[0] = tx_data_.p >> 8;
      can_tx_data_[1] = tx_data_.p & 0xff;
      can_tx_data_[2] = tx_data_.v >> 4;
      can_tx_data_[3] = ((tx_data_.v & 0xf) << 4) | (tx_data_.kp >> 8);
      can_tx_data_[4] = tx_data_.kp & 0xff;
      can_tx_data_[5] = tx_data_.kv >> 4;
      can_tx_data_[6] = ((tx_data_.kv & 0xf) << 4) | (tx_data_.t_ff >> 8);
      can_tx_data_[7] = tx_data_.t_ff & 0xff;
      HAL_CAN_AddTxMessage(hcan_, &can_tx_header_, can_tx_data_,
                           &can_tx_mail_box_);
    }
  }
  // transmit control pack
  else {
    setTorque(motor_->intensity_float_);
    can_tx_data_[0] = tx_data_.p >> 8;
    can_tx_data_[1] = tx_data_.p & 0xff;
    can_tx_data_[2] = tx_data_.v >> 4;
    can_tx_data_[3] = ((tx_data_.v & 0xf) << 4) | (tx_data_.kp >> 8);
    can_tx_data_[4] = tx_data_.kp & 0xff;
    can_tx_data_[5] = tx_data_.kv >> 4;
    can_tx_data_[6] = ((tx_data_.kv & 0xf) << 4) | (tx_data_.t_ff >> 8);
    can_tx_data_[7] = tx_data_.t_ff & 0xff;
    HAL_CAN_AddTxMessage(hcan_, &can_tx_header_, can_tx_data_,
                         &can_tx_mail_box_);
  }
}

// Check CAN channel and CAN message header id
bool MITMotorDriver::canRxMsgCheck(CAN_HandleTypeDef* hcan,
                                   CAN_RxHeaderTypeDef rx_header) {
  return hcan == hcan_ && rx_header.StdId == master_id_;
}

// Receive feedback data message callback. Called in
// HAL_CAN_RxFifo0MsgPendingCallback()
void MITMotorDriver::canRxMsgCallback(CAN_HandleTypeDef* hcan,
                                      CAN_RxHeaderTypeDef rx_header,
                                      uint8_t rx_data[8]) {
  if (motor_->info_.type != Motor::MIT || (rx_data[0] & 0x0F) != slave_id_) {
    return;
  }

  // unpack feedback data
  // 反馈数据解包
  uint16_t uint_p, uint_v, uint_t;
  uint_p = (rx_data[1] << 8) | (rx_data[2]);
  uint_v = (rx_data[3] << 4) | (rx_data[4] >> 4);
  uint_t = ((rx_data[4] & 0x0F) << 8) | (rx_data[5]);

  rx_data_.id = rx_data[0] & 0x0F;
  rx_data_.state = mitmotor::StateType_e(rx_data[0] >> 4);
  rx_data_.position =
      mitmotor::uint2float(uint_p, param_.p_min, param_.p_max, 16);
  rx_data_.velocity =
      mitmotor::uint2float(uint_v, param_.v_min, param_.v_max, 12);
  rx_data_.torq = mitmotor::uint2float(uint_t, param_.t_min, param_.t_max, 12);
  rx_data_.mos_temp = (float)rx_data[6];
  rx_data_.coil_temp = (float)rx_data[7];

  // Encoder angle(deg)
  // 编码器角度
  motor_->motor_data_.ecd_angle = math::rad2deg(rx_data_.position);
  // Use incremental calculation to deals with encoder overflow and underflow
  // 增量计算处理编码器上下溢问题
  float delta =
      motor_->motor_data_.ecd_angle - motor_->motor_data_.last_ecd_angle;
  delta = math::degNormalize180(delta) / motor_->ratio_;
  motor_->motor_data_.angle += delta;  // deg
  // feedback rotational speed
  // 反馈转速
  motor_->motor_data_.rotate_speed =
      math::radps2dps(rx_data_.velocity) / motor_->ratio_;  // dps
  // Update current
  // 更新转矩电流
  motor_->motor_data_.current = rx_data_.torq;
  // Update temperature
  // 更新温度
  motor_->motor_data_.temp = rx_data_.coil_temp;
  // update encoder angle record
  // 更新编码器角度记录
  motor_->motor_data_.last_ecd_angle = motor_->motor_data_.ecd_angle;

  motor_->rxCallback();
}

// Convert float to unsigned int, given range and number of bits
uint16_t mitmotor::float2uint(float x, float x_min, float x_max, uint8_t bits) {
  float span = x_max - x_min;
  float offset = x_min;
  return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

// Convert unsigned int to float, given range and number of bits
float mitmotor::uint2float(int x_int, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
