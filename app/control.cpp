/**
 ******************************************************************************
 * @file    control.cpp/h
 * @brief   Robot control design. 机器人控制设计（模式/键位）
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/control.h"
#include "app/arm.h"
#include "app/manual_config.h"

#include "iwdg.h"

// #include "app/imu_monitor.h"
#include "app/motor_monitor.h"
// #include "base/bsp/bsp_buzzer.h"
#include "base/bsp/bsp_led.h"
#include "base/common/math.h"
#include "base/remote/remote.h"
#include "base/remote/wfly.h"

void iwdgHandler(bool iwdg_refresh_flag);
void robotPowerStateFSM(bool stop_flag);
void robotReset(void);
bool robotStartup(void);
void robotControl(void);
void boardLedHandle(void);

#ifdef RF206S
extern WFLY rc;
#elif defined(DR16)
extern RC rc;
#endif
// extern IMU board_imu;

extern Arm arm;
BoardLed led;

// 上电状态
enum RobotPowerState_e {
  STOP = 0,
  STARTUP = 1,
  WORKING = 2,
} robot_state;
// 初始化标志
bool robot_init_flag = false;
// 遥控器挡位记录
#ifdef RF206S
WFLY::RCSwitch last_rc_switch;
#endif
#ifdef DR16
RC::RCSwitch last_rc_switch;
#endif

uint8_t arm_step = 0;  // 0 空闲，1 预备，2~5 依次走向 B、C、D、A
volatile bool arm_start_ok = false;  // 最近一次轨迹是否启动成功
uint32_t manual_mode = 0;  // 0 失能，1 位置，2 速度，3 电流
uint32_t calibration_ok = 0;
uint32_t rc_connected = 0;

// 控制初始化
void controlInit(void) {
  (RC_UART)->Init.BaudRate = 100000;
  (RC_UART)->Init.WordLength = UART_WORDLENGTH_9B;
#ifdef RF206S
  (RC_UART)->Init.StopBits = UART_STOPBITS_2;
#endif
#ifdef DR16
  (RC_UART)->Init.StopBits = UART_STOPBITS_1;
#endif
  (RC_UART)->Init.Parity = UART_PARITY_EVEN;
  (RC_UART)->Init.Mode = UART_MODE_RX;
  (RC_UART)->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  (RC_UART)->Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(RC_UART) != HAL_OK)
  {
      Error_Handler();
  }
  led.init();
  robot_state = STOP;
}

// 控制主循环
void controlLoop(void) {
  rc_connected = rc.connect_.check() ? 1u : 0u;
  iwdgHandler(rc.connect_.check());
  robotPowerStateFSM(!rc.connect_.check() || rc.switch_.r == decltype(rc)::DOWN);

  if (robot_state == STOP) {
    robotReset();
  } else if (robot_state == STARTUP) {
    robot_init_flag = robotStartup();  // 开机状态判断
  } else if (robot_state == WORKING) {
    robotControl();  // 机器人控制
  }
  arm.handle();
  boardLedHandle();
}

// IWDG处理，true持续刷新，false进入STOP状态并停止刷新
void iwdgHandler(bool iwdg_refresh_flag) {
  if (!iwdg_refresh_flag) {
    robot_state = STOP;
  } else {
    HAL_IWDG_Refresh(&hiwdg);
  }
}

// STOP(断电，安全模式)/开机/正常运行 状态机
void robotPowerStateFSM(bool stop_flag) {
  if (robot_state == STOP) {
    if (!stop_flag) {
      robot_state = STARTUP;
    }
  } else if (robot_state == STARTUP) {
    if (stop_flag) {
      robot_state = STOP;
    } else if (robot_init_flag) {
      // 初始化/复位完成
      robot_state = WORKING;
    }
  } else if (robot_state == WORKING) {
    if (stop_flag) {
      robot_state = STOP;
    }
  }
}

// 重置各功能状态
void robotReset(void) {
  robot_init_flag = false;
  arm_step = 0;
  arm_start_ok = false;
  arm.disable();
  last_rc_switch = rc.switch_;
}

// 开机上电启动处理
bool robotStartup(void) {
  bool flag = true;

  return flag;
}

// 机器人控制
void robotControl(void) {
  const bool changed = rc.switch_.l != last_rc_switch.l ||
                       rc.switch_.r != last_rc_switch.r;
  const bool centered = fabsf(static_cast<float>(rc.channel_.l_col)) <= manual::stick_deadband &&
                        fabsf(static_cast<float>(rc.channel_.r_col)) <= manual::stick_deadband;
  // 两根竖向摇杆，死区外线性映射到 [-1, 1]。
  const auto stick = [](float value) {
    if (fabsf(value) <= manual::stick_deadband) return 0.f;
    return math::limit((value - math::sign(value) * manual::stick_deadband) /
                       (manual::stick_range - manual::stick_deadband), -1.f, 1.f);
  };
  const float input0 = stick(rc.channel_.l_col);
  const float input1 = stick(rc.channel_.r_col);
  const bool calibrate = rc.switch_.l == decltype(rc)::MID &&
                         rc.switch_.r == decltype(rc)::MID;
  Arm::Mode requested = Arm::Mode::Disabled;
  if (rc.switch_.r == decltype(rc)::UP) {
    if (rc.switch_.l == decltype(rc)::UP) requested = Arm::Mode::Joint;
    if (rc.switch_.l == decltype(rc)::DOWN) requested = Arm::Mode::Speed;
  } else if (rc.switch_.r == decltype(rc)::MID &&
             rc.switch_.l == decltype(rc)::DOWN) {
    requested = Arm::Mode::Current;
  }

  if (calibrate) {
    arm.disable();
    if (changed) arm.calibrate(0.f, 0.f);
  } else if (requested != Arm::Mode::Disabled) {
    // 切换模式时摇杆需回中；上电、掉线恢复不会自行重新使能。
    if (changed) {
      arm.disable();
      if (centered) arm.enable(requested);
    }
    if (arm.mode() == requested) {
      if (requested == Arm::Mode::Joint) {
        arm.addJointReference(math::deg2rad(input0 * manual::position_speed),
                              math::deg2rad(input1 * manual::position_speed), manual::dt);
      } else if (requested == Arm::Mode::Speed) {
        arm.setJointSpeed(math::deg2rad(input0 * manual::speed_limit),
                           math::deg2rad(input1 * manual::speed_limit));
      } else {
        arm.setJointCurrent(input0 * manual::current_command * manual::current_to_raw,
                             input1 * manual::current_command * manual::current_to_raw);
      }
    }
  } else {
    // 其他挡位保持当前位置；从速度/电流模式退出时重新捕获位置。
    if (arm.mode() == Arm::Mode::Speed || arm.mode() == Arm::Mode::Current)
      arm.enable(Arm::Mode::Joint);
    arm.trajAbort();
  }
  last_rc_switch = rc.switch_;
}

// 板载LED指示灯效果
void boardLedHandle(void) {
#ifdef DBC
  if (robot_state == STOP) {
    led.setColor(255, 0, 0);  // red
    led.setModeOn();
  } else if (robot_state == STARTUP) {
    led.setColor(150, 150, 0);  // yellow
    led.setModeBreath();
  } else if (robot_state == WORKING) {
    led.setColor(0, 0, 255);  // blue
    if (rc.switch_.l == decltype(rc)::UP && rc.switch_.r == decltype(rc)::UP) {
      // 遥控器挡位左上右上
      led.setModeBlink(1);
    } else if (rc.switch_.l == decltype(rc)::MID && rc.switch_.r == decltype(rc)::UP) {
      // 遥控器挡位左中右上
      led.setModeBlink(2);
    } else if (rc.switch_.l == decltype(rc)::DOWN && rc.switch_.r == decltype(rc)::UP) {
      // 遥控器挡位左下右上
      led.setModeBlink(3);
    } else if (rc.switch_.l == decltype(rc)::UP && rc.switch_.r == decltype(rc)::MID) {
      // 遥控器挡位左上右中
      led.setModeBlink(4);
    } else if (rc.switch_.l == decltype(rc)::MID && rc.switch_.r == decltype(rc)::MID) {
      // 遥控器挡位左中右中
      led.setModeBlink(5);
    } else if (rc.switch_.l == decltype(rc)::DOWN && rc.switch_.r == decltype(rc)::MID) {
      // 遥控器挡位左下右中
      led.setModeBlink(6);
    }
  }
  led.handle();
#elif defined DBA
  if (robot_state == STOP) {
    led.setLED(true, false, 0);  // red
  } else if (robot_state == STARTUP) {
    led.setLED(true, true, 0);  // red+green
  } else if (robot_state == WORKING) {
    if (rc.switch_.l == decltype(rc)::UP && rc.switch_.r == decltype(rc)::UP) {
      // 遥控器挡位左上右上
      led.setLED(false, true, 1);  // green
    } else if (rc.switch_.l == decltype(rc)::MID && rc.switch_.r == decltype(rc)::UP) {
      // 遥控器挡位左中右上
      led.setLED(false, true, 2);  // green
    } else if (rc.switch_.l == decltype(rc)::DOWN && rc.switch_.r == decltype(rc)::UP) {
      // 遥控器挡位左下右上
      led.setLED(false, true, 3);  // green
    } else if (rc.switch_.l == decltype(rc)::UP && rc.switch_.r == decltype(rc)::MID) {
      // 遥控器挡位左上右中
      led.setLED(false, true, 4);  // green
    } else if (rc.switch_.l == decltype(rc)::MID && rc.switch_.r == decltype(rc)::MID) {
      // 遥控器挡位左中右中
      led.setLED(false, true, 5);  // green
    } else if (rc.switch_.l == decltype(rc)::DOWN && rc.switch_.r == decltype(rc)::MID) {
      // 遥控器挡位左下右中
      led.setLED(false, true, 6);  // green
    }
  }
#endif
}
