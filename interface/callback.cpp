/**
 ******************************************************************************
 * @file    callback.cpp/h
 * @brief   Interrupt request callback. 中断回调管理
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "interface/callback.h"

#include "can.h"
#include "gpio.h"
#include "tim.h"

#include "hardware_config.h"

#include "app/motor_monitor.h"
#include "base/remote/remote.h"
#include "base/remote/wfly.h"

#ifdef RF206S
extern WFLY rc;
#endif
#ifdef DR16
extern RC rc;
#endif

// CAN receive callback
// CAN接收回调
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

  motorsCanRxMsgHandle(hcan, rx_header, rx_data);
//   if (ultra_cap.canRxMsgCheck(hcan, rx_header)) {
//     ultra_cap.canRxMsgCallback(hcan, rx_header, rx_data);
//   }
}

// UART transmit callback
// UART发送中断回调
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {}

// UART receive callback
// UART接收中断回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  if (rc.uartCheck(huart)) {
    rc.rxCallback();
  }
}

// UART idle callback. Called in stm32f4xx_it.c USARTx_IRQHandler()
// UART空闲中断处理，在stm32f4xx_it.c的USARTx_IRQHandler()函数中调用
void User_UART_IdleHandler(UART_HandleTypeDef* huart) {
  // Judge if idle enabled. 判断空闲中断是否使能
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET) {
    // Clear idle flag. 清除空闲中断标记
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    // idle中断回调
    if (rc.uartCheck(huart)) {
      rc.idleCallback();
    }
  }
}

// USB CDC. Called in usbd_cdc_if CDC_Receive_FS()
// USB接收回调函数，在usbd_cdc_if的CDC_Receive_FS()函数中调用
void User_USB_CDC_RxMsgCallback(uint8_t* Buf, uint32_t* Len) {}

// EXTI callback
// 外部中断回调
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {}
