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

#ifndef WFLY_H
#define WFLY_H

#include "base/common/connect.h"
#include "usart.h"

#define WFLY_RX_BUF_SIZE 36u
#define WFLY_FRAME_LEN 25u

class WFLY {
public:
    // remote switch 遥控器拨挡
    typedef enum RCSwitchState {
        UP,
        MID,
        DOWN,
    } RCSwitchState_e;

public:
    WFLY(UART_HandleTypeDef* huart = nullptr);

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
    } channel_;
    // remote switch 遥控器拨挡
    struct RCSwitch {
        RCSwitchState_e l;
        RCSwitchState_e r;
    } switch_;

private:
    UART_HandleTypeDef* huart_;
    uint8_t rx_buf_[WFLY_RX_BUF_SIZE], rx_data_[WFLY_FRAME_LEN];
    volatile uint8_t rx_len_;

    struct RCRaw {
        int16_t ch[5];
        uint8_t s[2];
        int16_t optional[2];
        int16_t reserved[8];
    } rc_raw_;
};

#endif  // WFLY_H