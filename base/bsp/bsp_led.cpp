/**
 ******************************************************************************
 * @file    bsp_led.cpp/h
 * @brief   Development board LED. 板载LED
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "base/bsp/bsp_led.h"
#include "base/common/math.h"

#ifdef DBC

TIM_HandleTypeDef* led_htim = &htim5;
const uint32_t r_ch = TIM_CHANNEL_3;
const uint32_t g_ch = TIM_CHANNEL_2;
const uint32_t b_ch = TIM_CHANNEL_1;

BoardLed::BoardLed(void)
    : htim_(led_htim), r_ch_(r_ch), g_ch_(g_ch), b_ch_(b_ch) {}

void BoardLed::init(void) {
  HAL_TIM_PWM_Start(htim_, r_ch_);
  HAL_TIM_PWM_Start(htim_, g_ch_);
  HAL_TIM_PWM_Start(htim_, b_ch_);
}

void BoardLed::setColor(uint8_t r, uint8_t g, uint8_t b) {
  r_ = r;
  g_ = g;
  b_ = b;
}

void BoardLed::setModeBreath(uint32_t period) {
  mode_ = BREATH;
  breath_period_ = period;
}

void BoardLed::setModeBlink(uint8_t blink_times, uint32_t dt0, uint32_t dt1) {
  mode_ = BLINK;
  blink_param_.times = blink_times;
  blink_param_.dt[0] = dt0;
  blink_param_.dt[1] = dt1;
  blink_param_.period =
      2 * blink_param_.times * blink_param_.dt[0] + blink_param_.dt[1];
}

void BoardLed::handle(void) {
  if (mode_ == OFF) {
    // 常暗
    a_ = 0;
  } else if (mode_ == ON) {
    // 常亮
    a_ = 255;
  } else if (mode_ == BREATH) {
    // 呼吸灯
    float t = math::loopLimit(HAL_GetTick(), 0, breath_period_);
    if (t < breath_period_ / 2) {
      a_ = (uint8_t)(t / breath_period_ * 2 * 255);
    } else {
      a_ = (uint8_t)(255 - t / breath_period_ * 2 * 255);
    }
  } else if (mode_ == BLINK) {
    // 闪烁
    if (HAL_GetTick() - blink_param_.period_start_tick > blink_param_.period) {
      blink_param_.period_start_tick = HAL_GetTick();
    } else if (HAL_GetTick() - blink_param_.period_start_tick >
               2 * blink_param_.times * blink_param_.dt[0]) {
      a_ = 0;
    } else if ((HAL_GetTick() - blink_param_.period_start_tick) %
                   (2 * blink_param_.dt[0]) >
               blink_param_.dt[0]) {
      a_ = 255;
    } else {
      a_ = 0;
    }
  }

  // 设置PWM占空比
  uint16_t r_pwm, g_pwm, b_pwm;
  r_pwm = r_ * a_;
  g_pwm = g_ * a_;
  b_pwm = b_ * a_;
  __HAL_TIM_SetCompare(htim_, r_ch_, r_pwm);
  __HAL_TIM_SetCompare(htim_, g_ch_, g_pwm);
  __HAL_TIM_SetCompare(htim_, b_ch_, b_pwm);
}

#elif defined DBA

#include "main.h"

void BoardLed::setLED(bool red, bool green, uint8_t x) {
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, (GPIO_PinState)(!red));
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,
                    (GPIO_PinState)(!green));
  HAL_GPIO_WritePin(LED0_GPIO_Port, 0x1fe, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED0_GPIO_Port, 0x1fe >> (8 - x), GPIO_PIN_RESET);
}

#endif
