/**
 ******************************************************************************
 * @file    mahony.cpp/h
 * @brief   Mahony algorithm. Mahony算法实现
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef MAHONY_H
#define MAHONY_H

const float gravity_accel = 9.8f;

class Mahony {
 public:
  // Initialize & config parameter(time step & fusion coefficient)
  // 初始化，配置参数（时间步长，数据融合系数）
  Mahony(float dt, float kg, float km, float g_thres)
      : dt_(dt), kg_(kg), km_(km), g_thres_(g_thres) {}

  // Mahony algorithm, sensor data fusion, update quaternion
  // Mahony算法，传感器数据融合，四元数更新
  void update(float q[4], float ws[3], float as[3], float ms[3]);

  // Set dt
  // 设置时间步长
  void setDt(float dt);

  // Set sensor fusion corfficient
  // 设置数据融合系数
  void setK(float kg, float km);

 public:
  float q_[4];                   // quaternion
  float ws_[3], as_[3], ms_[3];  // gyro/accleration/magnet(sensor)
  float R_[3][3];                // rotate matrix
  float ww_[3], aw_[3], mw_[3];  // gyro & acceleration(no g) & magnet (world)

 private:
  float dq_[4];          // quaternion increment
  float RT_[3][3];       // rotate matrix transpose
  float _gs_[3];         // minus gravityaccleration (sensor)
  float mr_[3];          // magnet reference
  float eg_[3], em_[3];  // error vector, ea=cross(-am,g), em=cross(um,um~)
  float wu_[3], we_[3];  // wu(update)=ws+kg*ea+km*em

  float dt_;       // update time step;
  float kg_, km_;  // data fusion coefficient

  const float _gw_[3] = {0, 0, gravity_accel};  // -g(world)
  const float g_thres_;
};

#endif  // MAHONY_H