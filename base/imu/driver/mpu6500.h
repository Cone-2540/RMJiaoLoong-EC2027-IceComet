/**
 ******************************************************************************
 * @file    mpu6500.cpp/h
 * @brief   MPU6500(6-axis-IMU) driver. MPU6500(6轴IMU)驱动
 * @note    todo: ist8310未成功读取
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef MPU6500_H
#define MPU6500_H

#include <stdint.h>
#include "hardware_config.h"

typedef struct MPU6500_IST8310_REAL_DATA {
  float gyro[3];
  float accel[3];
  float mag[3];
  float temp;
} mpu6500_ist8310_real_data_t;

uint8_t mpu_device_init(void);
void mpu_get_data(float gyro[3], float accel[3], float mag[3],
                  float* temperate);

#endif  // MPU6500_H