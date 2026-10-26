/**
 ******************************************************************************
 * @file    bsp_buzzer.cpp/h
 * @brief   Buzzer 板载蜂鸣器
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include "hardware_config.h"
#include "tim.h"

#define L1 262
#define L1U 277
#define L2 294
#define L2U 311
#define L3 330
#define L4 349
#define L4U 370
#define L5 392
#define L5U 415
#define L6 440
#define L6U 466
#define L7 494

#define M1 523
#define M1U 554
#define M2 587
#define M2U 622
#define M3 659
#define M4 698
#define M4U 740
#define M5 784
#define M5U 831
#define M6 880
#define M6U 932
#define M7 988

#define H1 1046
#define H1U 1109
#define H2 1175
#define H2U 1245
#define H3 1318
#define H4 1397
#define H4U 1480
#define H5 1568
#define H5U 1661
#define H6 1760
#define H6U 1865
#define H7 1976

typedef struct MusicNote {
  uint16_t note;
  uint32_t time;
} MusicNote_t;

// music notes of Super Mario
const MusicNote_t music_super_mario[] = {
    {H3, 100}, {0, 50}, {H3, 250}, {0, 50},   {H3, 100}, {0, 50}, {0, 150},
    {H1, 100}, {0, 50}, {H3, 250}, {0, 50},   {H5, 250}, {0, 50}, {0, 300},
    {M5, 250}, {0, 50}, {0, 200},  {H1, 300}, {0, 50}};

#if (defined DBC) or (defined DBA)

class BoardBuzzer {
 public:
  BoardBuzzer(void);

  void init(void);
  void playNote(MusicNote_t note);
  void playMusic(const MusicNote_t* notes, uint16_t len);

 private:
  TIM_HandleTypeDef* htim_;
  uint32_t ch_;
  MusicNote_t note_;
  uint32_t stop_tick_;
};

#endif

#endif  // BSP_BUZZER_H
