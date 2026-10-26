/**
 ******************************************************************************
 * @file    hardware_config.cpp/h
 * @brief   Hardware configuration. 硬件配置（端口和硬件相关宏定义）
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

// 开发板型号
#define DBC

// 遥控器串口
#define RC_UART &huart3
// 遥控器型号
// #define RF206S
#define DR16
// 调试串口
// #define DEBUG_UART &huart1
#define DEBUG_USB

// imu相关(spi/i2c/tim)
#define BMI088_SPI &hspi1
#define IST8310_I2C &hi2c3
#define BOARD_IMU_HEAT_TIM &htim10
#define BOARD_IMU_HEAT_CHANNEL TIM_CHANNEL_1

#ifdef __cplusplus
}
#endif

#endif  // HARDWARE_CONFIG_H