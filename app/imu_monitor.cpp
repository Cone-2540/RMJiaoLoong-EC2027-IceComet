/**
 ******************************************************************************
 * @file    imu_monitor.cpp/h
 * @brief   IMU管理
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/imu_monitor.h"
#include "base/common/math.h"
#include "base/imu/driver/bmi088.h"
#include "base/imu/driver/ist8310.h"
#include "base/imu/driver/mpu6500.h"

// 板载IMU
namespace boardimu {

// 板载IMU传感器初始化函数
void initSensor(imu::RawData_t& raw_data);
// 板载imu传感器数据读取函数
void readSensor(imu::RawData_t& raw_data);

// 传感器开关
const bool bmi088_enabled = true;
const bool ist8310_enabled = false;
const bool mpu6500_enabled = true;

// imu参数
struct Param {
  // 姿态解算算法参数
  const float dt = 1e-3f;
  const float kg = 5e-3f;
  const float km = 0;
  const float g_thres = 0.3f;

  // 传感器方向变换矩阵(对应不同安装方向，默认为I(3))
  const float R_imu[3][3] = {{1.0f, 0.0f, 0.0f},
                             {0.0f, 1.0f, 0.0f},
                             {0.0f, 0.0f, 1.0f}};
  const float R_mag[3][3] = {{0.93f, 0.0f, 0.0f},
                             {0.0f, 1.03f, 0.0f},
                             {0.0f, 0.0f, 1.0f}};

  // 传感器偏移
  const float gyro_bias[3] = {2.6e-3f, 3e-4f, 3.5e-4f};
  const float accel_bias[3] = {0, 0, 0};
  const float mag_bias[3] = {-13.0f, -0.5f, 30.0f};
  // 加速度计比例
  const float accel_sen = 1;
} param;

// IMU温度控制
imu::TempControl temp(BOARD_IMU_HEAT_TIM, BOARD_IMU_HEAT_CHANNEL,
                      PID(5000, 0, 0, 500, 5000));

}  // namespace boardimu

// 板载IMU对象
IMU board_imu(boardimu::param.dt, boardimu::param.kg, boardimu::param.km,
              boardimu::param.g_thres, boardimu::param.R_imu,
              boardimu::param.R_mag, boardimu::param.gyro_bias,
              boardimu::param.accel_bias, boardimu::param.mag_bias,
              boardimu::param.accel_sen, &boardimu::readSensor);

// 外置IMU
namespace extimu {
// todo
}  // namespace extimu

// 初始化板载IMU传感器
void boardimu::initSensor(imu::RawData_t& raw_data) {
#ifdef DBC
  if (boardimu::bmi088_enabled) {
    imu::delay(100);
    while (BMI088_init()) {
      imu::delay(10);
    }
    BMI088_read(raw_data.gyro, raw_data.accel, raw_data.temp);
  }
  if (boardimu::ist8310_enabled) {
    while (ist8310_init()) {
      imu::delay(10);
    }
    ist8310_read_mag(raw_data.mag);
  }
  // 使用预校准参数，不进行开机校准
  board_imu.init(EulerAngle_t(0, 0, 0), true);
#elif defined DBA
  if (boardimu::mpu6500_enabled) {
    mpu_device_init();
  }
  // 开机校准500ms
  board_imu.init(EulerAngle_t(0, 0, 0), false, 500);
#endif  // DBC / DBA
  boardimu::temp.init();
}

// 读取板载IMU传感器数据
void boardimu::readSensor(imu::RawData_t& raw_data) {
#ifdef DBC
  if (boardimu::bmi088_enabled) {
    BMI088_read(raw_data.gyro, raw_data.accel, raw_data.temp);
  }
  if (boardimu::ist8310_enabled) {
    ist8310_read_mag(raw_data.mag);
  }
#elif defined DBA
  if (boardimu::mpu6500_enabled) {
    mpu_get_data(raw_data.gyro, raw_data.accel, raw_data.mag, raw_data.temp);
  }
#endif  // DBC/DBA
}

// 加热电阻输出端口PWM初始化
void imu::TempControl::init(void) {
  HAL_TIM_Base_Start(htim_);
  HAL_TIM_PWM_Start(htim_, tim_channel_);
}

// IMU加热电阻PWM控温
void imu::TempControl::handle(float temp_ref, float temp_fdb) {
  pwm_ = (uint16_t)math::limitMin(pid_.calc(temp_ref, temp_fdb), 0);
  __HAL_TIM_SetCompare(htim_, tim_channel_, pwm_);
}

// 全部IMU初始化
void imu::initAll(void) {
  boardimu::initSensor(board_imu.raw_data_);
}

// 全部IMU处理
void imu::handleAll(void) {
  board_imu.update();
  boardimu::temp.handle(40, board_imu.raw_data_.temp[0]);
}
