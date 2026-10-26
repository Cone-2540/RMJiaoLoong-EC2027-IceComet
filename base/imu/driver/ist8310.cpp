/**
 ******************************************************************************
 * @file    ist8310.cpp/h
 * @brief   IST8310(magnetometer) driver. IST8310(磁力计)驱动
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "ist8310.h"
#include "i2c.h"
#include "main.h"

static uint32_t fac_us = 168;

void ist8310_delay_init(void) {
  fac_us = SystemCoreClock / 1000000;
}

void ist8310_GPIO_init(void) {}

void ist8310_com_init(void) {}

uint8_t ist8310_IIC_read_single_reg(uint8_t reg) {
  uint8_t res;
#if (defined IST8310_I2C)
  HAL_I2C_Mem_Read(IST8310_I2C, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT,
                   &res, 1, 100);
#endif
  return res;
}

void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data) {
#if (defined IST8310_I2C)
  HAL_I2C_Mem_Write(IST8310_I2C, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT,
                    &data, 1, 100);
#endif
}

void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len) {
#if (defined IST8310_I2C)
  HAL_I2C_Mem_Read(IST8310_I2C, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT,
                   buf, len, 100);
#endif
}

void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t* data, uint8_t len) {
#if (defined IST8310_I2C)
  HAL_I2C_Mem_Write(IST8310_I2C, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT,
                    data, len, 100);
#endif
}

void ist8310_delay_us(uint16_t us) {
  uint32_t ticks = 0;
  uint32_t told = 0;
  uint32_t tnow = 0;
  uint32_t tcnt = 0;
  uint32_t reload = 0;
  reload = SysTick->LOAD;
  ticks = us * fac_us;
  told = SysTick->VAL;
  while (1) {
    tnow = SysTick->VAL;
    if (tnow != told) {
      if (tnow < told) {
        tcnt += told - tnow;
      } else {
        tcnt += reload - tnow + told;
      }
      told = tnow;
      if (tcnt >= ticks) {
        break;
      }
    }
  }
}

void ist8310_delay_ms(uint16_t ms) {
  while (ms--) {
    ist8310_delay_us(1000);
  }
}

void ist8310_RST_H(void) {
  HAL_GPIO_WritePin(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin, GPIO_PIN_SET);
}

void ist8310_RST_L(void) {
  HAL_GPIO_WritePin(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin, GPIO_PIN_RESET);
}

#define MAG_SEN 0.3f  // 转换成 uT

#define IST8310_WHO_AM_I 0x00        // ist8310 who am I 寄存器
#define IST8310_WHO_AM_I_VALUE 0x10  // 设备 ID

#define IST8310_WRITE_REG_NUM 4  // IST8310需要设置的寄存器数目

static const uint8_t ist8310_write_reg_data_error[IST8310_WRITE_REG_NUM][3] = {
    {0x0B, 0x08, 0x01},
    {0x41, 0x09, 0x02},
    {0x42, 0xC0, 0x03},
    {0x0A, 0x0B, 0x04}};

uint8_t ist8310_init(void) {
  static const uint8_t wait_time = 1;
  static const uint8_t sleepTime = 50;
  uint8_t res = 0;
  uint8_t writeNum = 0;

  ist8310_delay_init();

  ist8310_GPIO_init();
  ist8310_com_init();

  //    ist8310_RST_L();
  ist8310_delay_ms(sleepTime);
  ist8310_RST_H();
  ist8310_delay_ms(sleepTime);

  res = ist8310_IIC_read_single_reg(IST8310_WHO_AM_I);
  if (res != IST8310_WHO_AM_I_VALUE) {
    return IST8310_NO_SENSOR;
  }
  ist8310_delay_ms(wait_time);
  // set mpu6500 sonsor config and check
  for (writeNum = 0; writeNum < IST8310_WRITE_REG_NUM; writeNum++) {
    ist8310_IIC_write_single_reg(ist8310_write_reg_data_error[writeNum][0],
                                 ist8310_write_reg_data_error[writeNum][1]);
    ist8310_delay_ms(wait_time);
    res =
        ist8310_IIC_read_single_reg(ist8310_write_reg_data_error[writeNum][0]);
    ist8310_delay_ms(wait_time);
    if (res != ist8310_write_reg_data_error[writeNum][1]) {
      return ist8310_write_reg_data_error[writeNum][2];
    }
  }

  return IST8310_NO_ERROR;
}

void ist8310_read_over(uint8_t* status_buf,
                       ist8310_real_data_t* ist8310_real_data) {
  if (status_buf[0] & 0x01) {
    int16_t temp_ist8310_data = 0;
    ist8310_real_data->status |= 1 << IST8310_DATA_READY_BIT;

    temp_ist8310_data = (int16_t)((status_buf[2] << 8) | status_buf[1]);
    ist8310_real_data->mag[0] = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data = (int16_t)((status_buf[4] << 8) | status_buf[3]);
    ist8310_real_data->mag[1] = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data = (int16_t)((status_buf[6] << 8) | status_buf[5]);
    ist8310_real_data->mag[2] = MAG_SEN * temp_ist8310_data;
  } else {
    ist8310_real_data->status &= ~(1 << IST8310_DATA_READY_BIT);
  }
}

void ist8310_read_mag(float mag[3]) {
  uint8_t buf[6];
  int16_t temp_ist8310_data = 0;
  ist8310_IIC_read_muli_reg(0x03, buf, 6);

  temp_ist8310_data = (int16_t)((buf[1] << 8) | buf[0]);
  mag[0] = MAG_SEN * temp_ist8310_data;
  temp_ist8310_data = (int16_t)((buf[3] << 8) | buf[2]);
  mag[1] = MAG_SEN * temp_ist8310_data;
  temp_ist8310_data = (int16_t)((buf[5] << 8) | buf[4]);
  mag[2] = MAG_SEN * temp_ist8310_data;
}
