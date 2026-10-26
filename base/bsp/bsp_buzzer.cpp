/**
 ******************************************************************************
 * @file    bsp_buzzer.cpp/h
 * @brief   Buzzer 板载蜂鸣器
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "base/bsp/bsp_buzzer.h"
#include <string.h>
#include "cmsis_os.h"

#ifdef DBC
TIM_HandleTypeDef* board_buzzer_tim = &htim4;
const uint32_t board_buzzer_channel = TIM_CHANNEL_3;
#elif defined DBA
TIM_HandleTypeDef* board_buzzer_tim = &htim12;
const uint32_t board_buzzer_channel = TIM_CHANNEL_1;
#endif

#if (defined DBC) or (defined DBA)

void buzzerDelay(uint16_t ms) {
  // HAL_Delay(ms);
  osDelay(ms);
}

BoardBuzzer::BoardBuzzer(void)
    : htim_(board_buzzer_tim), ch_(board_buzzer_channel) {}

void BoardBuzzer::init(void) {
  HAL_TIM_PWM_Start(htim_, ch_);
}

void BoardBuzzer::playNote(MusicNote_t note) {
  note_ = note;
  stop_tick_ = HAL_GetTick() + note.time;
  if (note_.note == 0) {
    __HAL_TIM_SET_AUTORELOAD(htim_, 0);
  } else {
    __HAL_TIM_SET_AUTORELOAD(htim_, 1e6 / note_.note);
    __HAL_TIM_SET_COMPARE(htim_, ch_, 1e5 / note_.note);
  }
  while (HAL_GetTick() < stop_tick_) {
    buzzerDelay(10);
  }
}

void BoardBuzzer::playMusic(const MusicNote_t* notes, uint16_t len) {
  for (int i = 0; i < len; i++) {
    memcpy(&note_, (MusicNote_t*)notes + i, sizeof(MusicNote_t));
    playNote(note_);
  }
}

#endif
