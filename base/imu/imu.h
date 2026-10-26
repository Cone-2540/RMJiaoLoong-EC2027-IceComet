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

#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include "base/imu/mahony.h"

// 欧拉角
typedef struct EulerAngle {
  float yaw;
  float pitch;
  float roll;
  EulerAngle(float y = 0, float p = 0, float r = 0)
      : yaw(y), pitch(p), roll(r) {}
} EulerAngle_t;

namespace imu {

// 传感器原始数据
typedef struct rawData {
  float gyro[3];
  float accel[3];
  float mag[3];
  float temp[1];
} RawData_t;

// 陀螺仪毫秒延时
void delay(uint16_t ms);

}  // namespace imu

class IMU {
 public:
  // Set integral time step, sensor transform matrix(for different installation
  // direction, default I(3)), pre-calibrate bias input
  // 设置更新算法参数，传感器方向变换矩阵(对应不同安装方向，默认为I(3))，输入预校准传感器偏移*
  IMU(const float& dt, const float& kg, const float& km, const float& g_thres,
      const float R_imu[3][3], const float R_mag[3][3],
      const float gyro_bias[3], const float accel_bias[3],
      const float mag_bias[3], const float accel_sen,
      void (*readSensor)(imu::RawData_t&) = nullptr);

  // IMU sensor initialization, bias calibrate, set initial pose
  // IMU传感器初始化，校准偏移，设置初始姿态
  void init(EulerAngle_t euler_deg_init, bool is_calibrated,
            uint16_t calibrate_time_ms = 0);

  // Update IMU data. 更新IMU数据
  void update(void);

  // Sensor data transform(orientation&bias). 传感器数据坐标变换(方向&偏移)
  void sensorDataTransform(void);

  // Calibrate gyro bias, keep IMU static. 校准陀螺仪数据偏移
  void calibrateGyroBias(uint16_t ms);

  float& yaw(void) { return euler_deg_.yaw; }
  float& pitch(void) { return euler_deg_.pitch; }
  float& roll(void) { return euler_deg_.roll; }
  float& quat(uint8_t i) { return q_[i]; }
  float& wxSensor(void) { return gyro_sensor_dps_[0]; }
  float& wySensor(void) { return gyro_sensor_dps_[1]; }
  float& wzSensor(void) { return gyro_sensor_dps_[2]; }
  float& wxWorld(void) { return gyro_world_dps_[0]; }
  float& wyWorld(void) { return gyro_world_dps_[1]; }
  float& wzWorld(void) { return gyro_world_dps_[2]; }
  float& axSensor(void) { return accel_sensor_[0]; }
  float& aySensor(void) { return accel_sensor_[1]; }
  float& azSensor(void) { return accel_sensor_[2]; }
  float& axWorld(void) { return accel_world_[0]; }
  float& ayWorld(void) { return accel_world_[1]; }
  float& azWorld(void) { return accel_world_[2]; }

 public:
  // euler angle 欧拉角(ZY'X'')
  EulerAngle_t euler_deg_, euler_rad_;
  // quaternion 单位四元数
  float q_[4] = {1, 0, 0, 0};
  // angular velocity(rad/s) 角速度
  float gyro_sensor_[3], gyro_world_[3];
  // angular velocity(dps) 角速度
  float gyro_sensor_dps_[3], gyro_world_dps_[3];
  // acceleration(m/s^2) 加速度
  float accel_sensor_[3], accel_world_[3];
  // magnet 磁场
  float mag_sensor_[3], mag_world_[3];

  // 传感器原始数据
  imu::RawData_t raw_data_;

  // 传感器读取函数，在外部定义
  void (*readSensor_)(imu::RawData_t&);

 private:
  // 传感器变换矩阵(对应不同安装方向，默认为I(3))
  float R_imu_[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  float R_mag_[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  // 传感器偏移
  float gyro_bias_[3], accel_bias_[3], mag_bias_[3];
  float accel_sen_ = 1;
  bool is_calibrated_ = false;

  Mahony mahony_;
};

#endif  // IMU_H