/**
 ******************************************************************************
 * @file    matrix.cpp/h
 * @brief   Matrix calculation. 矩阵运算
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "matrix.h"

// Pay attention to the size of storage space
arm_status MatrixResize(Matrix* mat, uint16_t nRows, uint16_t nCols) {
  mat->numRows = nRows;
  mat->numCols = nCols;
  return ARM_MATH_SUCCESS;
}

// Matrix inverse calculation (Gaussian elimination)
arm_status MatrixInv(const Matrix* pSrc, Matrix* pDst) {
  // Check matrix size
  if (pSrc->numRows != pDst->numRows || pSrc->numCols != pDst->numCols ||
      pSrc->numRows != pSrc->numCols) {
    return ARM_MATH_SIZE_MISMATCH;
  }
  int n = pSrc->numRows;

  // extended matrix [A|I]
  Matrix ext_mat;
  float* ext_mat_data = new float[2 * n * n];
  MatrixInit(&ext_mat, n, 2 * n, ext_mat_data);
  VectorFill(0, ext_mat_data, 2 * n * n);
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      ext_mat_data[2 * n * i + j] = *(pSrc->pData + i * n + j);
    }
    ext_mat_data[2 * n * i + n + i] = 1;
  }

  // elimination
  for (int i = 0; i < n; i++) {
    // find maximum absolute value in the first column in lower right block
    float abs_max = fabs(ext_mat_data[2 * n * i + i]);
    int abs_max_row = i;
    for (int row = i; row < n; row++) {
      if (abs_max < fabs(ext_mat_data[2 * n * row + i])) {
        abs_max = fabs(ext_mat_data[2 * n * row + i]);
        abs_max_row = row;
      }
    }
    if (abs_max < 1e-12f) {  // singular
      delete[] ext_mat_data;
      return ARM_MATH_SINGULAR;
    }
    if (abs_max_row != i) {  // row exchange
      float tmp;
      for (int j = i; j < 2 * n; j++) {
        tmp = ext_mat_data[2 * n * i + j];
        ext_mat_data[2 * n * i + j] = ext_mat_data[2 * n * abs_max_row + j];
        ext_mat_data[2 * n * abs_max_row + j] = tmp;
      }
    }
    float k = 1.f / ext_mat_data[2 * n * i + i];
    for (int col = i; col < 2 * n; col++) {
      ext_mat_data[2 * n * i + col] *= k;
    }
    for (int row = 0; row < n; row++) {
      if (row == i) {
        continue;
      }
      k = ext_mat_data[2 * n * row + i];
      for (int j = i; j < 2 * n; j++) {
        ext_mat_data[2 * n * row + j] -= k * ext_mat_data[2 * n * i + j];
      }
    }
  }

  // inv = ext_mat(:,n+1:2n)
  for (int row = 0; row < n; row++) {
    memcpy((float*)pDst->pData + n * row, &ext_mat_data[2 * n * row + n],
           n * sizeof(float));
  }

  delete[] ext_mat_data;
  return ARM_MATH_SUCCESS;
}

void Matrix33fTrans(float mat[3][3], float res[3][3]) {
  res[0][0] = mat[0][0];
  res[0][1] = mat[1][0];
  res[0][2] = mat[2][0];
  res[1][0] = mat[0][1];
  res[1][1] = mat[1][1];
  res[1][2] = mat[2][1];
  res[2][0] = mat[0][2];
  res[2][1] = mat[1][2];
  res[2][2] = mat[2][2];
}

void Matrix33fMultVector3f(float mat[3][3], const float vec[3], float res[3]) {
  res[0] = mat[0][0] * vec[0] + mat[0][1] * vec[1] + mat[0][2] * vec[2];
  res[1] = mat[1][0] * vec[0] + mat[1][1] * vec[1] + mat[1][2] * vec[2];
  res[2] = mat[2][0] * vec[0] + mat[2][1] * vec[1] + mat[2][2] * vec[2];
}

void Vector3fAdd(const float a[3], const float b[3], float res[3]) {
  res[0] = a[0] + b[0];
  res[1] = a[1] + b[1];
  res[2] = a[2] + b[2];
}

void Vector3fSub(const float a[3], const float b[3], float res[3]) {
  res[0] = a[0] - b[0];
  res[1] = a[1] - b[1];
  res[2] = a[2] - b[2];
}

void Vector3fScale(const float k, const float vec[3], float res[3]) {
  res[0] = k * vec[0];
  res[1] = k * vec[1];
  res[2] = k * vec[2];
}

float Vector3fDot(const float a[3], const float b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void Vector3fCross(const float a[3], const float b[3], float res[3]) {
  res[0] = a[1] * b[2] - a[2] * b[1];
  res[1] = a[2] * b[0] - a[0] * b[2];
  res[2] = a[0] * b[1] - a[1] * b[0];
}

float Vector3fNorm(const float vec[3]) {
  return sqrtf(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
}

float Vector4fNorm(const float vec[4]) {
  return sqrtf(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2] +
               vec[3] * vec[3]);
}

void Vector3fUnit(const float vec[3], float res[3]) {
  float norm = Vector3fNorm(vec);
  if (norm > 1e-8f) {
    res[0] = vec[0] / norm;
    res[1] = vec[1] / norm;
    res[2] = vec[2] / norm;
  } else {
    res[0] = vec[0];
    res[1] = vec[1];
    res[2] = vec[2];
  }
}

void Vector4fUnit(const float vec[4], float res[4]) {
  float norm = Vector4fNorm(vec);
  if (norm > 1e-8f) {
    res[0] = vec[0] / norm;
    res[1] = vec[1] / norm;
    res[2] = vec[2] / norm;
    res[3] = vec[3] / norm;
  } else {
    res[0] = vec[0];
    res[1] = vec[1];
    res[2] = vec[2];
    res[3] = vec[3];
  }
}
