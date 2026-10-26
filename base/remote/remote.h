/**
 ******************************************************************************
 * @file    remote.cpp/h
 * @brief   Remote control. 遥控器
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef REMOTE_H
#define REMOTE_H

#include "base/common/connect.h"
#include "usart.h"

#define RC_RX_BUF_SIZE 36u
#define RC_FRAME_LEN 18u

typedef uint16_t RCKey;

// keyboard definition 键盘按键
#define KEY_W 0x0001u
#define KEY_S 0x0002u
#define KEY_A 0x0004u
#define KEY_D 0x0008u
#define KEY_SHIFT 0x0010u
#define KEY_CTRL 0x0020u
#define KEY_Q 0x0040u
#define KEY_E 0x0080u
#define KEY_R 0x0100u
#define KEY_F 0x0200u
#define KEY_G 0x0400u
#define KEY_Z 0x0800u
#define KEY_X 0x1000u
#define KEY_C 0x2000u
#define KEY_V 0x4000u
#define KEY_B 0x8000u

class RC {
 public:
  // remote switch 遥控器拨挡
  typedef enum RCSwitchState {
    UP,
    MID,
    DOWN,
  } RCSwitchState_e;

 public:
  RC(UART_HandleTypeDef* huart = nullptr);

  void init(void);
  void reset(void);
  void handle(void);

  bool uartCheck(UART_HandleTypeDef* huart) { return huart == huart_; }
  void rxCallback(void);
  void idleCallback(void);

 public:
  // connect state 遥控器连接状态
  Connect connect_;
  // remote channel 遥控器通道
  struct RCChannel {
    int16_t r_row;
    int16_t r_col;
    int16_t l_row;
    int16_t l_col;
    int16_t dial_wheel;
  } channel_;
  // remote switch 遥控器拨挡
  struct RCSwitch {
    RCSwitchState_e l;
    RCSwitchState_e r;
  } switch_;
  // mouse 鼠标
  struct Mouse {
    int16_t x;
    int16_t y;
    int16_t z;
    bool press_l;
    bool press_r;
  } mouse_;
  RCKey key_;

 private:
  UART_HandleTypeDef* huart_;
  uint8_t rx_buf_[RC_RX_BUF_SIZE], rx_data_[RC_FRAME_LEN];
  volatile uint8_t rx_len_;

  struct RCRaw {
    int16_t ch[5];
    uint8_t s[2];
  } rc_raw_;
};

#endif  // REMOTE_H