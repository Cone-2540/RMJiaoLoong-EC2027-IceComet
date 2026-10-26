/**
 ******************************************************************************
 * @file    bsp_led.cpp/h
 * @brief   Development board LED. 板载LED
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef BSP_LED_H
#define BSP_LED_H

#include <stdint.h>
#include "hardware_config.h"

#ifdef DBC

#include "tim.h"

class BoardLed {
 public:
  BoardLed(void);

  void init(void);

  void setColor(uint8_t r, uint8_t g, uint8_t b);

  void setModeOff(void) { mode_ = OFF; }
  void setModeOn(void) { mode_ = ON; }
  void setModeBreath(uint32_t period = 1000);
  void setModeBlink(uint8_t times = 1, uint32_t dt0 = 200, uint32_t dt1 = 1000);

  void handle(void);

 private:
  // color(r, g, b, alpha)
  uint8_t r_, g_, b_, a_;

  enum LEDMode {
    OFF,
    ON,
    BREATH,
    BLINK,
  } mode_;
  uint32_t breath_period_;  // ms
  struct BlinkParam {
    uint8_t times;
    uint32_t dt[2];  // ms
    uint32_t period;
    uint32_t period_start_tick;
  } blink_param_;

  TIM_HandleTypeDef* htim_;
  uint32_t r_ch_, g_ch_, b_ch_;
};

#elif defined DBA

class BoardLed {
 public:
  BoardLed(void) {}
  void init(void) {}
  void setLED(bool red, bool green, uint8_t x = 0);
};

#endif

#endif  // BSP_LED_H
