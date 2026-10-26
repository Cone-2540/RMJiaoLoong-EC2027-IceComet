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

#ifndef CALLBACK_H
#define CALLBACK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"

// UART idle callback. Called in stm32f4xx_it.c USARTx_IRQHandler()
// UART空闲中断处理，在stm32f4xx_it.c的USARTx_IRQHandler()函数中调用
void User_UART_IdleHandler(UART_HandleTypeDef* huart);

// USB CDC. Called in usbd_cdc_if CDC_Receive_FS()
// USB接收回调函数，在usbd_cdc_if的CDC_Receive_FS()函数中调用
void User_USB_CDC_RxMsgCallback(uint8_t* Buf, uint32_t* Len);

#ifdef __cplusplus
}
#endif

#endif  // CALLBACK_H
