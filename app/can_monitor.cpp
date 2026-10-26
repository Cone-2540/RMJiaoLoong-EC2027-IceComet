/**
 ******************************************************************************
 * @file    can_monitor.cpp/h
 * @brief   CAN communication transmit manage. CAN通信发送管理
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/can_monitor.h"

#include "can.h"
#include "cmsis_os.h"

#include "app/motor_monitor.h"

extern DJIMotorDriver dji_motor_driver;
extern MITMotorDriver mit_motor_driver[];

// CAN filter初始化
void canFilterInit(void) {
  CAN_FilterTypeDef filter;
  filter.FilterActivation = ENABLE;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = 0x0000;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.SlaveStartFilterBank = 14;

  filter.FilterBank = 0;
  HAL_CAN_ConfigFilter(&hcan1, &filter);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

//   filter.FilterBank = 14;
//   HAL_CAN_ConfigFilter(&hcan2, &filter);
//   HAL_CAN_Start(&hcan2);
//   HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

// CAN通信发送管理
void canTxMonitor(void) {
  // note: 每个通道一次只能发送3个包
  dji_motor_driver.canTxMsg(1, djimotor::ID_1_4);   // can1, id: 0x200
//   dji_motor_driver.canTxMsg(1, djimotor::ID_5_8);   // can1, id: 0x1ff
//   dji_motor_driver.canTxMsg(1, djimotor::ID_9_11);  // can1, id: 0x2ff

//   dji_motor_driver.canTxMsg(2, djimotor::ID_1_4);  // can2, id: 0x200
//   dji_motor_driver.canTxMsg(2, djimotor::ID_5_8);  // can2, id: 0x1ff
  // dji_motor_driver.canTxMsg(2, DJIMotor::ID_9_11);  // can2, id: 0x2ff

//  mit_motor_driver[0].canTxMsg();

  osDelay(1);

  // dji_motor_driver.canTxMsg(1, DJIMotor::ID_1_4);  // can1, id: 0x200
  // dji_motor_driver.canTxMsg(1, DJIMotor::ID_5_8);  // can1, id: 0x1ff
  // capacity.canTxMsg();                       // can1, id: 0x301

  // dji_motor_driver.canTxMsg(2, DJIMotor::ID_1_4);  // can2, id: 0x200
  // dji_motor_driver.canTxMsg(2, DJIMotor::ID_5_8);  // can2, id: 0x1ff
  // LK9025.canTxMsg();                         // can2, id: 0x280

  // osDelay(1);
}
