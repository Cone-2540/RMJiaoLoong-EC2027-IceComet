/**
 ******************************************************************************
 * @file    kalman_filter.cpp/h
 * @brief   Kalman filter. 卡尔曼滤波实现
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "matrix.h"

class KalmanFilter {
 public:
  // Initialize vectors/matrices, allocate storage space, set initial
  // vector/constant matrices 初始化矩阵/向量，分配内存，设置初值
  KalmanFilter(uint16_t x_size, uint16_t u_size, uint16_t z_size,
               const float* F, const float* B, const float* H, const float* Q,
               const float* R, const float* x0);
  // default: 1-dimension
  KalmanFilter(const float* F, const float* B, const float* H, const float* Q,
               const float* R, const float* x0)
      : KalmanFilter(1, 1, 1, F, B, H, Q, R, x0) {}

  // Free storage ptr
  // 释放内存
  ~KalmanFilter(void);

  // Kalman filter update. 卡尔曼滤波更新
  void update(float* x, float* u, float* z, uint16_t x_size, uint16_t u_size,
              uint16_t z_size);
  // Time-invariant system Kalman filter update, using pre-calculate K.
  // 时不变系统卡尔曼滤波离线更新(使用预先计算的K)
  void updateOffline(float* x, float* u, float* z, float* K, uint16_t x_size,
                     uint16_t u_size, uint16_t z_size);
  // 1-dimension update
  void update(float* x, float* u, float* z);
  void updateOffline(float* x, float* u, float* z, float* K);

  // Set system model. 设置系统模型
  void setModel(const float* A, const float* B, const float* H);
  // Set noise covariance matrix Q & R. Q 应半正定，R 应正定
  void setQR(const float* Q, const float* R);
  // 重置状态和协方差，P 恢复为构造时的零矩阵
  void reset(const float* x);
  // Set estimated state vector x(k|k)
  void setXhat(const float* x);

 private:
  // x(k|k-1)=Fx(k-1|k-1)+Bu
  void updateXminus(float* u, uint16_t u_size);
  // P(k|k-1)=FP(k-1|k-1)F^T+Q
  void updatePminus(void);
  // K=P(k|k-1)H^T*(HP(k|k-1)H^T+R)^-1
  void updateK(void);
  // x(k|k)=x(k|k-1)+K(z-Hx(k|k-1))
  void updateXhat(float* z, uint16_t z_size);
  // P(k|k)=(I-KH)P(k|k-1)
  void updateP(void);

 private:
  // vector size
  uint16_t x_size_;
  uint16_t u_size_;
  uint16_t z_size_;
  // ARM matrix
  struct {
    Matrix x_hat, x_minus;  // state vector x(k|k) & x(k|k-1)
    Matrix u;               // control vector u
    Matrix z;               // observation vector z
    Matrix F, FT;           // state transition matrix A & transpose A^T
    Matrix B;               // control matrix B
    Matrix H, HT;           // observation matrix H & transpose H^T
    Matrix P, P_minus;      // error covariance matrix P(k|k) & P(k|k-1)
    Matrix Q, R;            // noise covariance matrix Q & R
    Matrix K;               // kalman gain matrix K
    Matrix tmp[2];          // temporary matrix
  } mat_;
  // Matrix data storage ptr
  struct {
    float *x_hat, *x_minus;
    float* u;
    float* z;
    float *F, *FT;
    float* B;
    float *H, *HT;
    float *P, *P_minus;
    float *Q, *R;
    float* K;
    float* tmp[2];
  } data_;
  // Error status of matrix calculation
  arm_status mat_status_;
};

#endif  // KALMAN_FILTER_H
