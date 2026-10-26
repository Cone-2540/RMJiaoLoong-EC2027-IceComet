/**
 ******************************************************************************
 * @file    imu.cpp/h
 * @brief   IMU数据处理
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "base/imu/imu.h"
#include <string.h>
#include "base/common/math.h"
#include "lib/arm_math/arm_math.h"
#include "main.h"

// Quaternion to Euler angle(rad). 四元数转欧拉角
void quat2eulerRad(float q[4], EulerAngle_t& euler) {
  euler.yaw = atan2f(2.0f * (q[1] * q[2] + q[0] * q[3]),
                     1 - 2.0f * (q[2] * q[2] + q[3] * q[3]));
  euler.pitch = asinf(2.0f * (q[0] * q[2] - q[1] * q[3]));
  euler.roll = atan2f(2.0f * (q[2] * q[3] + q[0] * q[1]),
                      1 - 2.0f * (q[1] * q[1] + q[2] * q[2]));
}

// Euler angle(rad) to Quaternion. 欧拉角转换为四元数
void eulerRad2quat(EulerAngle_t& euler, float q[4]) {
  // cos(*/2)
  float c[3] = {cosf(0.5f * euler.yaw), cosf(0.5f * euler.pitch),
                cosf(0.5f * euler.roll)};
  // sin(*/2)
  float s[3] = {sinf(0.5f * euler.yaw), sinf(0.5f * euler.pitch),
                sinf(0.5f * euler.roll)};
  q[0] = c[0] * c[1] * c[2] + s[0] * s[1] * s[2];  // q0=cos(θ/2)
  q[1] = c[0] * c[1] * s[2] - s[0] * s[1] * c[2];  // q1=rx*sin(θ/2)
  q[2] = c[0] * s[1] * c[2] + s[0] * c[1] * s[2];  // q2=ry*sin(θ/2)
  q[3] = s[0] * c[1] * c[2] - c[0] * s[1] * s[2];  // q3=rz*sin(θ/2)
}

// 陀螺仪毫秒延时
void imu::delay(uint16_t ms) {
  HAL_Delay(ms);
}

// Set transform matrix, Mahony parameter, pre-calibrated bias, temperature
// control pid & port
// 设置坐标变换矩阵，Mahony算法参数，零漂预校准参数，温控参数和输出端口
IMU::IMU(const float& dt, const float& kg, const float& km,
         const float& g_thres, const float R_imu[3][3], const float R_mag[3][3],
         const float gyro_bias[3], const float accel_bias[3],
         const float mag_bias[3], const float accel_sen,
         void (*readSensor)(imu::RawData_t&))
    : readSensor_(readSensor), mahony_(dt, kg, km, g_thres) {
  memcpy(R_imu_, R_imu, 9 * sizeof(float));
  memcpy(R_mag_, R_mag, 9 * sizeof(float));
  memcpy(gyro_bias_, gyro_bias, 3 * sizeof(float));
  memcpy(accel_bias_, accel_bias, 3 * sizeof(float));
  memcpy(mag_bias_, mag_bias, 3 * sizeof(float));
}

// IMU sensor initialization, set initial pose, calibrate bias
// IMU传感器初始化，设置初始姿态，校准偏移
void IMU::init(EulerAngle_t euler_deg_init, bool is_calibrated,
               uint16_t calibrate_time_ms) {
  // 初始化姿态角
  euler_deg_ = euler_deg_init;
  euler_rad_.yaw = math::deg2rad(euler_deg_.yaw);
  euler_rad_.pitch = math::deg2rad(euler_deg_.pitch);
  euler_rad_.roll = math::deg2rad(euler_deg_.roll);
  eulerRad2quat(euler_rad_, q_);

  is_calibrated_ = is_calibrated;
  // 传感器偏移校准
  if (!is_calibrated_) {
    calibrateGyroBias(calibrate_time_ms);
  }
}

// Update IMU data, period should match dt_. 更新IMU数据，运行周期需与dt_匹配
// Called after read data from sensor
void IMU::update(void) {
  // Read sensor data. 传感器读取
  if (readSensor_ != nullptr) {
    readSensor_(raw_data_);
  }
  // Sensor data transform(orientation&bias). 坐标变换(方向&偏移)
  sensorDataTransform();

  // Mahony algorithm
  mahony_.update(q_, gyro_sensor_, accel_sensor_, mag_sensor_);

  // Update data
  quat2eulerRad(q_, euler_rad_);
  euler_deg_.yaw = math::rad2deg(euler_rad_.yaw);
  euler_deg_.pitch = math::rad2deg(euler_rad_.pitch);
  euler_deg_.roll = math::rad2deg(euler_rad_.roll);
  memcpy(gyro_world_, mahony_.ww_, 3 * sizeof(float));
  memcpy(accel_world_, mahony_.aw_, 3 * sizeof(float));
  memcpy(mag_world_, mahony_.mw_, 3 * sizeof(float));
  for (int i = 0; i < 3; i++) {
    gyro_sensor_dps_[i] = math::radps2dps(gyro_sensor_[i]);
    gyro_world_dps_[i] = math::radps2dps(gyro_world_[i]);
  }
}

// Calibrate gyro bias, keep IMU remain still
// 校准陀螺仪零漂，期间保持IMU静止(BMI088可预先校准，无需开机校准)
void IMU::calibrateGyroBias(uint16_t ms) {
  float gyro_bias[3] = {0};
  float accel_sen = 0;
  for (int i = 0; i < ms; i++) {
    update();
    gyro_bias[0] += gyro_sensor_[0] / (float)ms;
    gyro_bias[1] += gyro_sensor_[1] / (float)ms;
    gyro_bias[2] += gyro_sensor_[2] / (float)ms;
    accel_sen += gravity_accel /
                 sqrtf(accel_sensor_[0] * accel_sensor_[0] +
                       accel_sensor_[1] * accel_sensor_[1] +
                       accel_sensor_[2] * accel_sensor_[2]) /
                 (float)ms;
    imu::delay(1);
  }
  gyro_bias_[0] = gyro_bias[0];
  gyro_bias_[1] = gyro_bias[1];
  gyro_bias_[2] = gyro_bias[2];
  accel_sen_ = accel_sen;
  is_calibrated_ = true;
}

// Sensor data transform(orientation&bias). 坐标变换(方向&偏移)
void IMU::sensorDataTransform(void) {
  for (uint8_t i = 0; i < 3; i++) {
    gyro_sensor_[i] = raw_data_.gyro[0] * R_imu_[i][0] +
                      raw_data_.gyro[1] * R_imu_[i][1] +
                      raw_data_.gyro[2] * R_imu_[i][2] - gyro_bias_[i];
    accel_sensor_[i] = accel_sen_ * (raw_data_.accel[0] * R_imu_[i][0] +
                                     raw_data_.accel[1] * R_imu_[i][1] +
                                     raw_data_.accel[2] * R_imu_[i][2]) -
                       accel_bias_[i];
    mag_sensor_[i] = raw_data_.mag[0] * R_mag_[i][0] +
                     raw_data_.mag[1] * R_mag_[i][1] +
                     raw_data_.mag[2] * R_mag_[i][2] - mag_bias_[i];
  }
}
