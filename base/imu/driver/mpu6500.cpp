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

#include "mpu6500.h"
#include "ist8310_reg.h"
#include "mpu6500_reg.h"
#include "spi.h"

static uint8_t tx, rx;
static uint8_t tx_buff[14] = {0xff};
uint8_t mpu_buff[14]; /* buffer to save imu raw data */
uint8_t ist_buff[6];  /* buffer to save IST8310 raw data */

float MPU6500_ACCEL_SEN = 0.0093523f;
float MPU6500_GYRO_SEN = 0.001065264436f;
float IST8310_MAG_SEN = 0.3f;

void MPU_DELAY(uint32_t ms) {
  HAL_Delay(ms);
}

void MPU_NSS_LOW(void) {
#if (defined NSS_GPIO_Port) and (defined NSS_Pin)
  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
#endif
}

void MPU_NSS_HIGH(void) {
#if (defined NSS_GPIO_Port) and (defined NSS_Pin)
  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
#endif
}

/**
 * @brief  write a byte of data to specified register
 * @param  reg:  the address of register to be written
 *         data: data to be written
 */
uint8_t mpu_write_byte(uint8_t const reg, uint8_t const data) {
  MPU_NSS_LOW();
  tx = reg & 0x7F;
#if defined MPU6500_SPI
  HAL_SPI_TransmitReceive(MPU6500_SPI, &tx, &rx, 1, 55);
  tx = data;
  HAL_SPI_TransmitReceive(MPU6500_SPI, &tx, &rx, 1, 55);
#endif
  MPU_NSS_HIGH();
  return 0;
}

/**
 * @brief  read a byte of data from specified register
 * @param  reg: the address of register to be read
 */
uint8_t mpu_read_byte(uint8_t const reg) {
  MPU_NSS_LOW();
  tx = reg | 0x80;
#if defined MPU6500_SPI
  HAL_SPI_TransmitReceive(MPU6500_SPI, &tx, &rx, 1, 55);
  HAL_SPI_TransmitReceive(MPU6500_SPI, &tx, &rx, 1, 55);
#endif
  MPU_NSS_HIGH();
  return rx;
}

/**
 * @brief  read bytes of data from specified register
 * @param  reg: address from where data is to be written
 */
uint8_t mpu_read_bytes(uint8_t const regAddr, uint8_t* pData, uint8_t len) {
  MPU_NSS_LOW();
  tx = regAddr | 0x80;
  tx_buff[0] = tx;
#if defined MPU6500_SPI
  HAL_SPI_TransmitReceive(MPU6500_SPI, &tx, &rx, 1, 55);
  HAL_SPI_TransmitReceive(MPU6500_SPI, tx_buff, pData, len, 55);
#endif
  MPU_NSS_HIGH();
  return 0;
}

/**
 * @brief  write IST8310 register through MPU6500's I2C master
 * @param  addr: the address to be written of IST8310's register
 *         data: data to be written
 */
static void ist_reg_write_by_mpu(uint8_t addr, uint8_t data) {
  /* turn off slave 1 at first */
  mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
  MPU_DELAY(2);
  mpu_write_byte(MPU6500_I2C_SLV1_REG, addr);
  MPU_DELAY(2);
  mpu_write_byte(MPU6500_I2C_SLV1_DO, data);
  MPU_DELAY(2);
  /* turn on slave 1 with one byte transmitting */
  mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
  /* wait longer to ensure the data is transmitted from slave 1 */
  MPU_DELAY(10);
}

/**
 * @brief  write IST8310 register through MPU6500's I2C Master
 * @param  addr: the address to be read of IST8310's register
 */
static uint8_t ist_reg_read_by_mpu(uint8_t addr) {
  uint8_t retval;
  mpu_write_byte(MPU6500_I2C_SLV4_REG, addr);
  MPU_DELAY(10);
  mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x80);
  MPU_DELAY(10);
  retval = mpu_read_byte(MPU6500_I2C_SLV4_DI);
  /* turn off slave4 after read */
  mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
  MPU_DELAY(10);
  return retval;
}

/**
 * @brief    initialize the MPU6500 I2C Slave 0 for I2C reading.
 * @param    device_address: slave device address, Address[6:0]
 */
static void mpu_master_i2c_auto_read_config(uint8_t device_address,
                                            uint8_t reg_base_addr,
                                            uint8_t data_num) {
  /*
   * configure the device address of the IST8310
   * use slave1, auto transmit single measure mode
   */
  mpu_write_byte(MPU6500_I2C_SLV1_ADDR, device_address);
  MPU_DELAY(2);
  mpu_write_byte(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
  MPU_DELAY(2);
  mpu_write_byte(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
  MPU_DELAY(2);

  /* use slave0,auto read data */
  mpu_write_byte(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
  MPU_DELAY(2);
  mpu_write_byte(MPU6500_I2C_SLV0_REG, reg_base_addr);
  MPU_DELAY(2);

  /* every eight mpu6500 internal samples one i2c master read */
  mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x03);
  MPU_DELAY(2);
  /* enable slave 0 and 1 access delay */
  mpu_write_byte(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
  MPU_DELAY(2);
  /* enable slave 1 auto transmit */
  mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
  /* Wait 6ms (minimum waiting time for 16 times internal average setup) */
  MPU_DELAY(6);
  /* enable slave 0 with data_num bytes reading */
  mpu_write_byte(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
  MPU_DELAY(2);
}

namespace mpu6500 {

/**
 * @brief  Initializes the IST8310 device
 * @param
 */
uint8_t ist8310_init() {
  /* enable iic master mode */
  mpu_write_byte(MPU6500_USER_CTRL, 0x30);
  MPU_DELAY(10);
  /* enable iic 400khz */
  mpu_write_byte(MPU6500_I2C_MST_CTRL, 0x0d);
  MPU_DELAY(10);

  /* turn on slave 1 for ist write and slave 4 to ist read */
  mpu_write_byte(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);
  MPU_DELAY(10);
  mpu_write_byte(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
  MPU_DELAY(10);

  /* IST8310_R_CONFB 0x01 = device rst */
  ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
  MPU_DELAY(10);
  if (IST8310_DEVICE_ID_A != ist_reg_read_by_mpu(IST8310_WHO_AM_I))
    return 1;

  /* soft reset */
  ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
  MPU_DELAY(10);

  /* config as ready mode to access register */
  ist_reg_write_by_mpu(IST8310_R_CONFA, 0x00);
  if (ist_reg_read_by_mpu(IST8310_R_CONFA) != 0x00)
    return 2;
  MPU_DELAY(10);

  /* normal state, no int */
  ist_reg_write_by_mpu(IST8310_R_CONFB, 0x00);
  if (ist_reg_read_by_mpu(IST8310_R_CONFB) != 0x00)
    return 3;
  MPU_DELAY(10);

  /* config low noise mode, x,y,z axis 16 time 1 avg */
  ist_reg_write_by_mpu(IST8310_AVGCNTL, 0x24);  // 100100
  if (ist_reg_read_by_mpu(IST8310_AVGCNTL) != 0x24)
    return 4;
  MPU_DELAY(10);

  /* Set/Reset pulse duration setup,normal mode */
  ist_reg_write_by_mpu(IST8310_PDCNTL, 0xc0);
  if (ist_reg_read_by_mpu(IST8310_PDCNTL) != 0xc0)
    return 5;
  MPU_DELAY(10);

  /* turn off slave1 & slave 4 */
  mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
  MPU_DELAY(10);
  mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
  MPU_DELAY(10);

  /* configure and turn on slave 0 */
  mpu_master_i2c_auto_read_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
  MPU_DELAY(100);
  return 0;
}

/**
 * @brief  get the data of IST8310
 * @param  buff: the buffer to save the data of IST8310
 */
void ist8310_get_data(uint8_t* buff) {
  mpu_read_bytes(MPU6500_EXT_SENS_DATA_00, buff, 6);
}

}  // namespace mpu6500

/**
 * @brief  get the data of imu
 * @param
 */
void mpu_get_data(float gyro[3], float accel[3], float mag[3],
                  float* temperate) {
  mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);
  int16_t mpu_raw_temp;

  mpu_raw_temp = (int16_t)(mpu_buff[0] << 8 | mpu_buff[1]);
  accel[0] = mpu_raw_temp * MPU6500_ACCEL_SEN;
  mpu_raw_temp = (int16_t)(mpu_buff[2] << 8 | mpu_buff[3]);
  accel[1] = mpu_raw_temp * MPU6500_ACCEL_SEN;
  mpu_raw_temp = (int16_t)(mpu_buff[4] << 8 | mpu_buff[5]);
  accel[2] = mpu_raw_temp * MPU6500_ACCEL_SEN;
  mpu_raw_temp = (int16_t)(mpu_buff[6] << 8 | mpu_buff[7]);
  *temperate = 21.f + mpu_raw_temp * 0.0029518f;

  mpu_raw_temp = (mpu_buff[8] << 8 | mpu_buff[9]);
  gyro[0] = mpu_raw_temp * MPU6500_GYRO_SEN;
  mpu_raw_temp = (mpu_buff[10] << 8 | mpu_buff[11]);
  gyro[1] = mpu_raw_temp * MPU6500_GYRO_SEN;
  mpu_raw_temp = (mpu_buff[12] << 8 | mpu_buff[13]);
  gyro[2] = mpu_raw_temp * MPU6500_GYRO_SEN;

  //  ist8310_get_data(ist_buff);
  mpu_raw_temp = (int16_t)((ist_buff[1] << 8) | ist_buff[0]);
  mag[0] = IST8310_MAG_SEN * mpu_raw_temp;
  mpu_raw_temp = (int16_t)((ist_buff[3] << 8) | ist_buff[2]);
  mag[1] = IST8310_MAG_SEN * mpu_raw_temp;
  mpu_raw_temp = (int16_t)((ist_buff[5] << 8) | ist_buff[4]);
  mag[2] = IST8310_MAG_SEN * mpu_raw_temp;
}

/**
 * @brief  set imu 6500 gyroscope measure range
 * @param  fsr: range(0,±250dps;1,±500dps;2,±1000dps;3,±2000dps)
 */
uint8_t mpu_set_gyro_fsr(uint8_t fsr) {
  return mpu_write_byte(MPU6500_GYRO_CONFIG, fsr << 3);
}

/**
 * @brief  set imu 6050/6500 accelerate measure range
 * @param  fsr: range(0,±2g;1,±4g;2,±8g;3,±16g)
 */
uint8_t mpu_set_accel_fsr(uint8_t fsr) {
  return mpu_write_byte(MPU6500_ACCEL_CONFIG, fsr << 3);
}

/**
 * @brief  initialize imu mpu6500 and magnet meter ist3810
 * @param
 */
uint8_t mpu_id;
uint8_t mpu_device_init(void) {
  MPU_DELAY(100);

  mpu_id = mpu_read_byte(MPU6500_WHO_AM_I);
  uint8_t i = 0;
  // 0: 250hz; 1: 184hz; 2: 92hz; 3: 41hz; 4: 20hz; 5: 10hz; 6: 5hz; 7: 3600hz
  uint8_t MPU6500_Init_Data[10][2] = {
      {MPU6500_PWR_MGMT_1, 0x80},     /* Reset Device */
      {MPU6500_PWR_MGMT_1, 0x03},     /* Clock Source - Gyro-Z */
      {MPU6500_PWR_MGMT_2, 0x00},     /* Enable Acc & Gyro */
      {MPU6500_CONFIG, 0x02},         /* LPF 92Hz */
      {MPU6500_GYRO_CONFIG, 0x18},    /* +-2000dps */
      {MPU6500_ACCEL_CONFIG, 0x10},   /* +-8G */
      {MPU6500_ACCEL_CONFIG_2, 0x04}, /* enable LowPassFilter. Set Acc LPF */
      {MPU6500_USER_CTRL, 0x20},
  }; /* Enable AUX */
  for (i = 0; i < 10; i++) {
    mpu_write_byte(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
    MPU_DELAY(1);
  }

  mpu_set_gyro_fsr(3);
  mpu_set_accel_fsr(2);

  mpu6500::ist8310_init();
  return 0;
}
