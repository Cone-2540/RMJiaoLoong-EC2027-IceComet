/**
 ******************************************************************************
 * @file    robot.cpp/h
 * @brief   Main program. 主程序
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef ROBOT_H
#define ROBOT_H

#ifdef __cplusplus
extern "C" {
#endif

// Create RTOS tasks. Called in freertos.c MX_FREERTOS_Init()
// 创建RTOS任务。在freertos.c的MX_FREERTOS_Init()中调用
void rtosTaskInit(void);

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_H