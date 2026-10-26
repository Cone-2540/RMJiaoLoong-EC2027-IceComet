/**
 ******************************************************************************
 * @file    ist8310.cpp/h
 * @brief   IST8310(magnetometer) driver. IST8310(磁力计)驱动
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef IST8310_H
#define IST8310_H

#include <stdint.h>
#include "hardware_config.h"

#define IST8310_IIC_ADDRESS (0x0E << 1)  // IST8310的IIC地址
#define IST8310_IIC_READ_MSB (0x80)  // IST8310的SPI读取发送第一个bit为1

#define IST8310_DATA_READY_BIT 2

#define IST8310_NO_ERROR 0x00
#define IST8310_NO_SENSOR 0x40

typedef struct ist8310_real_data_t {
  uint8_t status;
  float mag[3];
} ist8310_real_data_t;

uint8_t ist8310_init(void);
void ist8310_read_over(uint8_t* status_buf,
                       ist8310_real_data_t* mpu6500_real_data);
void ist8310_read_mag(float mag[3]);

#endif  // IST8310_H