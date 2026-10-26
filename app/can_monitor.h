/**
 ******************************************************************************
 * @file    can_monitor.cpp/h
 * @brief   CAN communication transmit manage. CAN通信发送管理
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef CAN_MONITOR_H
#define CAN_MONITOR_H

// CAN filter初始化
void canFilterInit(void);
// CAN通信发送管理
void canTxMonitor(void);

#endif  // CAN_MONITOR_H
