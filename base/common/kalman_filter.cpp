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

#include "kalman_filter.h"

// Initialize vectors/matrices, allocate storage space, set initial
// vector/constant matrices 初始化矩阵/向量，分配内存，设置初值
KalmanFilter::KalmanFilter(uint16_t x_size, uint16_t u_size, uint16_t z_size,
                           const float* F, const float* B, const float* H,
                           const float* Q, const float* R, const float* x0)
    : x_size_(x_size), u_size_(u_size), z_size_(z_size) {
  // prior estimated state vector x_hat, x(k|k)=x0
  data_.x_hat = new float[x_size_];
  MatrixInit(&mat_.x_hat, x_size_, 1, (float*)data_.x_hat);
  memcpy(data_.x_hat, x0, x_size_ * sizeof(float));
  // posterior estimated state vector x_minus, x(k|k-1)
  data_.x_minus = new float[x_size_];
  MatrixInit(&mat_.x_minus, x_size_, 1, (float*)data_.x_minus);
  // control vector u
  data_.u = new float[u_size_];
  MatrixInit(&mat_.u, u_size_, 1, (float*)data_.u);
  // observation vector z
  data_.z = new float[z_size_];
  MatrixInit(&mat_.z, z_size_, 1, (float*)data_.z);
  // state transition matrix F & transpose F^T
  data_.F = new float[x_size_ * x_size_];
  MatrixInit(&mat_.F, x_size_, x_size_, (float*)data_.F);
  memcpy(data_.F, F, x_size_ * x_size_ * sizeof(float));
  data_.FT = new float[x_size_ * x_size_];
  MatrixInit(&mat_.FT, x_size_, x_size_, (float*)data_.FT);
  MatrixTrans(&mat_.F, &mat_.FT);
  // control matrix B
  data_.B = new float[x_size_ * u_size_];
  MatrixInit(&mat_.B, x_size_, u_size_, (float*)data_.B);
  memcpy(data_.B, B, x_size_ * u_size_ * sizeof(float));
  // observation matrix H & transpose H^T
  data_.H = new float[z_size_ * x_size_];
  MatrixInit(&mat_.H, z_size_, x_size_, (float*)data_.H);
  memcpy(data_.H, H, z_size_ * x_size_ * sizeof(float));
  data_.HT = new float[z_size_ * x_size_];
  MatrixInit(&mat_.HT, x_size_, z_size_, (float*)data_.HT);
  MatrixTrans(&mat_.H, &mat_.HT);
  // prior estimated error covariance matrix P(k|k)
  data_.P = new float[x_size_ * x_size_];
  MatrixInit(&mat_.P, x_size_, x_size_, (float*)data_.P);
  VectorFill(0, data_.P, x_size_ * x_size_);
  // posterior estimated error covariance matrix P(k|k-1)
  data_.P_minus = new float[x_size_ * x_size_];
  MatrixInit(&mat_.P_minus, x_size_, x_size_, (float*)data_.P_minus);
  // process noise covariance matrix Q
  data_.Q = new float[x_size_ * x_size_];
  MatrixInit(&mat_.Q, x_size_, x_size_, (float*)data_.Q);
  memcpy(data_.Q, Q, x_size_ * x_size_ * sizeof(float));
  // osbervation noise covariance matrix R
  data_.R = new float[z_size_ * z_size_];
  MatrixInit(&mat_.R, z_size_, z_size_, (float*)data_.R);
  memcpy(data_.R, R, z_size_ * z_size_ * sizeof(float));
  // kalman gain matrix K
  data_.K = new float[x_size_ * z_size_];
  MatrixInit(&mat_.K, x_size_, z_size_, (float*)data_.K);
  // tmporary marix (tmp[0]-xs*xs, tmp[1][2]-zs*zs)
  data_.tmp[0] = new float[x_size_ * x_size_];
  MatrixInit(&mat_.tmp[0], x_size_, x_size_, (float*)data_.tmp[0]);
  data_.tmp[1] = new float[x_size_ * x_size_];
  MatrixInit(&mat_.tmp[1], x_size_, x_size_, (float*)data_.tmp[1]);
}

// Free data storage ptr
// 释放内存
KalmanFilter::~KalmanFilter(void) {
  delete[] data_.x_hat;
  data_.x_hat = nullptr;
  delete[] data_.x_minus;
  data_.x_minus = nullptr;
  delete[] data_.u;
  data_.u = nullptr;
  delete[] data_.z;
  data_.z = nullptr;
  delete[] data_.F;
  data_.F = nullptr;
  delete[] data_.FT;
  data_.FT = nullptr;
  delete[] data_.B;
  data_.B = nullptr;
  delete[] data_.H;
  data_.H = nullptr;
  delete[] data_.HT;
  data_.HT = nullptr;
  delete[] data_.P;
  data_.P = nullptr;
  delete[] data_.P_minus;
  data_.P_minus = nullptr;
  delete[] data_.Q;
  data_.Q = nullptr;
  delete[] data_.R;
  data_.R = nullptr;
  delete[] data_.K;
  data_.K = nullptr;
  delete[] data_.tmp[0];
  data_.tmp[0] = nullptr;
  delete[] data_.tmp[1];
  data_.tmp[1] = nullptr;
}

// update kalman filter 卡尔曼滤波更新
void KalmanFilter::update(float* x, float* u, float* z, uint16_t x_size,
                          uint16_t u_size, uint16_t z_size) {
  if (x_size != x_size_ || u_size != u_size_ || z_size != z_size_) {
    return;
  }
  updateXminus(u, u_size);  // x(k|k-1)=Ax(k-1|k-1)+Bu
  updatePminus();           // P(k|k-1)=AP(k-1|k-1)F^T+Q
  updateK();                // K=P(k|k-1)H^T*(HP(k|k-1)H^T+R)^-1
  updateXhat(z, z_size);    // x(k|k)=x(k|k-1)+K(z-Hx(k|k-1))
  updateP();                // P(k|k)=(I-KH)P(k|k-1)
  memcpy(x, data_.x_hat, x_size_ * sizeof(float));
}

// Time-invariant system Kalman filter update, using pre-calculate K.
// 时不变系统卡尔曼滤波离线更新(使用预先计算的K)
void KalmanFilter::updateOffline(float* x, float* u, float* z, float* K,
                                 uint16_t x_size, uint16_t u_size,
                                 uint16_t z_size) {
  if (x_size != x_size_ || u_size != u_size_ || z_size != z_size_) {
    return;
  }
  updateXminus(u, u_size);  // x(k|k-1)=Fx(k-1|k-1)+Bu
  memcpy(data_.K, K, x_size_ * z_size_ * sizeof(float));
  updateXhat(z, z_size);  // x(k|k)=x(k|k-1)+K(z-Hx(k|k-1))
  memcpy(x, data_.x_hat, x_size_ * sizeof(float));
} 

// 1-dimension Kalman fiter update 默认按一维卡尔曼滤波更新
void KalmanFilter::update(float* x, float* u, float* z) {
  update(x, u, z, 1, 1, 1);
}

// 1-dimension Kalman fiter update offline 一维卡尔曼滤波离线更新
void KalmanFilter::updateOffline(float* x, float* u, float* z, float* K) {
  updateOffline(x, u, z, K, 1, 1, 1);
}

// Set system model. 设置系统模型
void KalmanFilter::setModel(const float* F, const float* B, const float* H) {
  memcpy(data_.F, F, x_size_ * x_size_ * sizeof(float));
  memcpy(data_.B, B, x_size_ * u_size_ * sizeof(float));
  memcpy(data_.H, H, z_size_ * x_size_ * sizeof(float));
}

// Set noise covariance matrix Q & R. 设置QR矩阵
void KalmanFilter::setQR(const float* Q, const float* R) {
  memcpy(data_.Q, Q, x_size_ * x_size_ * sizeof(float));
  memcpy(data_.R, R, z_size_ * z_size_ * sizeof(float));
}

// Reset state and covariance. 重置状态及协方差。
void KalmanFilter::reset(const float* x) {
  setXhat(x);
  VectorFill(0, data_.P, x_size_ * x_size_);
}

// Set estimated state vector x(k|k)
void KalmanFilter::setXhat(const float* x) {
  memcpy(data_.x_hat, x, x_size_ * sizeof(float));
}

// x(k|k-1)=Fx(k-1|k-1)+Bu
void KalmanFilter::updateXminus(float* u, uint16_t u_size) {
  if (u_size != u_size_)
    return;
  memcpy(data_.u, u, u_size_ * sizeof(float));
  MatrixResize(&mat_.tmp[0], x_size_, 1);
  // Fx(k-1|k-1)
  mat_status_ = MatrixMult(&mat_.F, &mat_.x_hat, &mat_.x_minus);
  // Bu
  mat_status_ = MatrixMult(&mat_.B, &mat_.u, &mat_.tmp[0]);
  // Fx(k-1|k-1)+Bu
  mat_status_ = MatrixAdd(&mat_.x_minus, &mat_.tmp[0], &mat_.x_minus);
}

// P(k|k-1)=AP(k-1|k-1)F^T+Q
void KalmanFilter::updatePminus() {
  MatrixTrans(&mat_.F, &mat_.FT);
  MatrixResize(&mat_.tmp[0], x_size_, x_size_);
  // FP(k-1|k-1)
  mat_status_ = MatrixMult(&mat_.F, &mat_.P, &mat_.tmp[0]);
  // FP(k-1|k-1)F^T
  mat_status_ = MatrixMult(&mat_.tmp[0], &mat_.FT, &mat_.P_minus);
  // FP(k-1|k-1)F^T+Q
  mat_status_ = MatrixAdd(&mat_.P_minus, &mat_.Q, &mat_.P_minus);
}

// K=P(k|k-1)H^T*(HP(k|k-1)H^T+R)^-1
void KalmanFilter::updateK() {
  MatrixTrans(&mat_.H, &mat_.HT);
  MatrixResize(&mat_.tmp[0], x_size_, z_size_);
  MatrixResize(&mat_.tmp[1], z_size_, z_size_);
  // tmp[0]=P(k|k-1)H^T
  mat_status_ = MatrixMult(&mat_.P_minus, &mat_.HT, &mat_.tmp[0]);
  // tmp[1]=HP(k|k-1)H^T
  mat_status_ = MatrixMult(&mat_.H, &mat_.tmp[0], &mat_.tmp[1]);
  // HP(k|k-1)H^T+R
  mat_status_ = MatrixAdd(&mat_.tmp[1], &mat_.R, &mat_.tmp[1]);
  // (HP(k|k-1)H^T+R)^-1
  if (z_size_ == 1) {
    // 单观测的创新协方差是标量；有效 Q、R 下 S > 0，无需堆分配求逆
    mat_.tmp[1].pData[0] = 1.f / mat_.tmp[1].pData[0];
  } else {
    mat_status_ = MatrixInv(&mat_.tmp[1], &mat_.tmp[1]);
  }
  // P(k|k-1)H^T*(HP(k|k-1)H^T+R)^-1
  mat_status_ = MatrixMult(&mat_.tmp[0], &mat_.tmp[1], &mat_.K);
}

// x(k|k)=x(k|k-1)+K(z-Hx(k|k-1))
void KalmanFilter::updateXhat(float* z, uint16_t z_size) {
  if (z_size != z_size_)
    return;
  memcpy(data_.z, z, z_size_ * sizeof(float));
  MatrixResize(&mat_.tmp[1], z_size_, 1);
  // Hx(k|k-1)
  mat_status_ = MatrixMult(&mat_.H, &mat_.x_minus, &mat_.tmp[1]);
  // z-Hx(k|k-1)
  mat_status_ = MatrixSub(&mat_.z, &mat_.tmp[1], &mat_.tmp[1]);
  // K(z-Hx(k|k-1))
  mat_status_ = MatrixMult(&mat_.K, &mat_.tmp[1], &mat_.x_hat);
  // x(k|k-1)+K(z-Hx(k|k-1))
  mat_status_ = MatrixAdd(&mat_.x_minus, &mat_.x_hat, &mat_.x_hat);
}

// P(k|k)=(I-KH)P(k|k-1)
void KalmanFilter::updateP() {
  MatrixResize(&mat_.tmp[0], x_size_, x_size_);
  MatrixResize(&mat_.tmp[1], x_size_, x_size_);
  // KH
  mat_status_ = MatrixMult(&mat_.K, &mat_.H, &mat_.tmp[0]);
  // KHP(k|k-1)
  mat_status_ = MatrixMult(&mat_.tmp[0], &mat_.P_minus, &mat_.tmp[1]);
  // P(k|k-1)-KHP(k|k-1)
  mat_status_ = MatrixSub(&mat_.P_minus, &mat_.tmp[1], &mat_.P);
}
