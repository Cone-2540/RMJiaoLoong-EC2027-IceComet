/**
 ******************************************************************************
 * @file    wfly.cpp/h
 * @brief   WFLY Remote control. 天地飞遥控器
 * @author  Albert Wang
 ******************************************************************************
 * Copyright (c) 2024 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "base/remote/wfly.h"
#include <string.h>

// max time to wait for connect(ms)
const uint32_t rc_connect_timeout = 100;

const int16_t rc_ch_offset = 1024;

WFLY::WFLY(UART_HandleTypeDef* huart)
        : huart_(huart), connect_(rc_connect_timeout) {
    rx_len_ = 0;
    switch_.l = DOWN;
    switch_.r = DOWN;
}

// Start UART(SBUS) receive. 打开UART接收
void WFLY::init(void) {
    rx_len_ = 0;
    reset();
    if (huart_ != nullptr) {
        __HAL_UART_ENABLE_IT(huart_, UART_IT_IDLE);
        HAL_UART_Receive_DMA(huart_, rx_buf_, 1);
    }
}

// Reset WFLY data. 重置遥控器数据
void WFLY::reset(void) {
    rc_raw_.ch[0] = 0;
    rc_raw_.ch[1] = 0;
    rc_raw_.ch[2] = 0;
    rc_raw_.ch[3] = 0;
    rc_raw_.s[0] = 0;
    rc_raw_.s[1] = 0;
    rc_raw_.optional[0] = 0;
    rc_raw_.optional[1] = 0;

    channel_.r_row = 0;
    channel_.r_col = 0;
    channel_.l_row = 0;
    channel_.l_col = 0;
}

// Unpack data. 数据解包
void WFLY::handle(void) {
    // Check connection
    if (!connect_.check()) {
        reset();
        return;
    }

    // Unpack data. 数据解包
  rc_raw_.ch[0] = (rx_data_[1] | rx_data_[2] << 8) & 0x07ff;       // Channel 1
  rc_raw_.ch[1] = (rx_data_[2] >> 3 | rx_data_[3] << 5) & 0x07ff;  // Channel 2
  rc_raw_.ch[2] = (rx_data_[3] >> 6 | rx_data_[4] << 2 | rx_data_[5] << 10) &
                  0x07ff;                                          // Channel 3
  rc_raw_.ch[3] = (rx_data_[5] >> 1 | rx_data_[6] << 7) & 0x07ff;  // Channel 4
  rc_raw_.s[0] = (rx_data_[6] >> 4 | rx_data_[7] << 4) & 0x07ff;   // Channel 5
  rc_raw_.s[1] = (rx_data_[7] >> 7 | rx_data_[8] << 1 | rx_data_[9] << 9) &
                 0x07ff;   // Channel 6
  rc_raw_.optional[0] = (rx_data_[9] >> 2 | rx_data_[10] << 6) & 0x07ff;   // Channel 7
  rc_raw_.optional[1] = (rx_data_[10] >> 5 | rx_data_[11] << 3) & 0x07ff;  // Channel 8
  rc_raw_.reserved[0] = (rx_data_[12] | rx_data_[13] << 8) & 0x07ff;
  rc_raw_.reserved[1] = (rx_data_[13] >> 3 | rx_data_[14] << 5) & 0x07ff;
  rc_raw_.reserved[2] = (rx_data_[14] >> 6 | rx_data_[15] << 2 | rx_data_[16] << 10) &
                        0x07ff;
  rc_raw_.reserved[3] = (rx_data_[16] >> 1 | rx_data_[17] << 7) & 0x07ff;
  rc_raw_.reserved[4] = (rx_data_[17] >> 4 | rx_data_[18] << 4) & 0x07ff;
  rc_raw_.reserved[5] = (rx_data_[18] >> 7 | rx_data_[19] << 1 | rx_data_[20] << 9) &
                        0x07ff;
  rc_raw_.reserved[6] = (rx_data_[20] >> 2 | rx_data_[21] << 6) & 0x07ff;
  rc_raw_.reserved[7] = (rx_data_[21] >> 5 | rx_data_[22] << 3) & 0x07ff;

  channel_.r_row = rc_raw_.ch[0] - rc_ch_offset;
  channel_.l_col = rc_raw_.ch[1] - rc_ch_offset;
  channel_.r_col = rc_raw_.ch[2] - rc_ch_offset;
  channel_.l_row = rc_raw_.ch[3] - rc_ch_offset;
  if (rc_raw_.s[0] == 97) {
    switch_.l = UP;
  } else if (rc_raw_.s[0] == 158) {
    switch_.l = DOWN;
  } else if (rc_raw_.s[0] == 0) {
    switch_.l = MID;
  }
  if (rc_raw_.s[1] == 97) {
    switch_.r = UP;
  } else if (rc_raw_.s[1] == 158) {
    switch_.r = DOWN;
  } else if (rc_raw_.s[1] == 0) {
    switch_.r = MID;
  }
}

// Update connect status, restart UART(SBUS) receive.
// 更新连接状态，重新打开UART(SBUS)接收
void WFLY::rxCallback(void) {
    rx_len_++;
    if (huart_ != nullptr) {
        HAL_UART_Receive_DMA(huart_, rx_buf_ + rx_len_, 1);
    }
}

// Idle callback, update connect status.
// 空闲中断，判断数据长度，更新连接状态，重新打开UART(SBUS)接收
void WFLY::idleCallback(void) {
    // Check frame length. 判断帧数据长度
    if (rx_len_ >= WFLY_FRAME_LEN) {
        if (rx_buf_[23] == 0) {
            connect_.refresh();
        }
        memcpy(rx_data_, rx_buf_ + rx_len_ - WFLY_FRAME_LEN, WFLY_FRAME_LEN);
    }
    rx_len_ = 0;
    if (huart_ != nullptr) {
        HAL_UART_AbortReceive(huart_);
        HAL_UART_Receive_DMA(huart_, rx_buf_, 1);
    }
}
