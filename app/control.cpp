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
#include "app/square_config.h"
#include <cmath>

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

uint8_t arm_step = 0;  // 0 空闲，1~8 尖角方形/圆，9/10 任意轨迹，11/12 圆角方形
volatile bool arm_start_ok = false;  // 最近一次轨迹是否启动成功
SquareDebug square_debug{0, 0, {square::speed_ff_gain[0], square::speed_ff_gain[1]},
                         square::edge_duration,
                         {square::moving_deadband[0], square::moving_deadband[1]},
                         0, {0, 0}, {0, 0}, square::corner_radius};
static uint32_t square_cycle_start_tick = 0;
static float square_edge_duration = square::edge_duration;
static float square_corner_radius = square::corner_radius;
static float square_hold_deadband[2]{};
static bool square_pid_active = false;
static void squareRestorePid();
static float square_return_q[2]{};
static bool square_return_ready = false;
static bool draw_trigger_ready = false;
FreehandPath freehand_path{1, 0, 0, square::side_length/square::edge_duration,
                           0, 0.f, {}};
uint32_t manual_mode = 0;  // 0 失能，1 位置，2 速度，3 电流
uint32_t calibration_ok = 0;
uint32_t rc_connected = 0;
uint32_t led_status = LED_RC_LOST;

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
  if ((arm_step >= 2 && arm_step <= 5) || arm_step == 8 || arm_step == 12) {
    Motor* joints[2] = {&J0, &J1};
    for (int i = 0; i < 2; ++i) {
      const float command = fabsf(static_cast<float>(joints[i]->intensity_)) / manual::current_to_raw;
      const float feedback = fabsf(joints[i]->motor_data_.current) / manual::current_to_raw;
      if (command > square_debug.command_peak_a[i]) square_debug.command_peak_a[i] = command;
      if (feedback > square_debug.feedback_peak_a[i]) square_debug.feedback_peak_a[i] = feedback;
    }
  }
  if (arm_step == 0) squareRestorePid();
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
  squareRestorePid();
  setJointStepProfile(false);
  if (arm_step != 0 || square_debug.request != 0) square_debug.status = 4;
  square_debug.request = 0;
  square_return_ready = false;
  draw_trigger_ready = false;
  if (arm_step == 9 || arm_step == 10) freehand_path.status = 5;
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

// 先检查四条边，再用关节空间轨迹到达 A 点的正肘支姿态。
static bool squareUseMovingPid() {
  for (int i = 0; i < 2; ++i)
    if (!std::isfinite(square_debug.moving_deadband[i]) || square_debug.moving_deadband[i] < 0.f)
      return false;
  if (!square_pid_active) {
    square_hold_deadband[0] = J0.ppid_.deadband_;
    square_hold_deadband[1] = J1.ppid_.deadband_;
    square_pid_active = true;
  }
  J0.ppid_.deadband_ = square_debug.moving_deadband[0];
  J1.ppid_.deadband_ = square_debug.moving_deadband[1];
  J0.ppid_.reset(); J1.ppid_.reset();
  return true;
}

static void squareRestorePid() {
  if (!square_pid_active) return;
  J0.ppid_.deadband_ = square_hold_deadband[0];
  J1.ppid_.deadband_ = square_hold_deadband[1];
  J0.ppid_.reset(); J1.ppid_.reset();
  square_pid_active = false;
}

static bool squareStart(bool circle = false) {
  if (!arm.setTrajectorySpeedGain(square_debug.speed_gain[0], square_debug.speed_gain[1]))
    return false;
  if (!std::isfinite(square::prepare_duration) || square::prepare_duration <= 0.f ||
      !std::isfinite(square_debug.edge_duration) || square_debug.edge_duration <= 0.f) return false;
  square_edge_duration = square_debug.edge_duration;
  square_corner_radius = circle ? 0.f : square_debug.corner_radius;
  if (!std::isfinite(square_corner_radius) || square_corner_radius < 0.f ||
      square_corner_radius > square::half_side ||
      square_edge_duration > 2147483.f / 4.f) return false;
  if (square_corner_radius > 0.f) {
    if (!arm.trajPrepareRoundedSquare(square::side_length,square_corner_radius,
                                      4.f*square_edge_duration,square::prepare_duration)) return false;
    if (!squareUseMovingPid()) { arm.trajAbort(); return false; }
    square_return_ready = false;
    return true;
  }
  for (int i = 0; i < 4; ++i) {
    CartesianReference start{}, end{};
    start.x = square::vertices[i][0]; start.y = square::vertices[i][1];
    end.x = square::vertices[(i + 1) % 4][0]; end.y = square::vertices[(i + 1) % 4][1];
    if (!ArmKinematics::lineReachable(start, end)) return false;
  }
  CartesianReference a{};
  a.x = square::vertices[0][0]; a.y = circle ? 0.f : square::vertices[0][1];
  float q[2];
  if (!ArmKinematics::inverse(a, arm.state().q, 1, q)) return false;
  if (!squareUseMovingPid()) return false;
  if (!arm.trajStartJoint(q[0], q[1], square::prepare_duration)) return false;
  square_return_q[0] = q[0]; square_return_q[1] = q[1];
  square_return_ready = false;
  return true;
}

static bool freehandStart() {
  if (freehand_path.upload_lock || freehand_path.version != 1 ||
      freehand_path.count < 2 || freehand_path.count > ArmTrajectory::max_path_points ||
      !arm.setTrajectorySpeedGain(square_debug.speed_gain[0], square_debug.speed_gain[1]))
    return false;
  if (!arm.trajPreparePath(freehand_path.points, freehand_path.count,
                           freehand_path.speed_mps, square::prepare_duration)) return false;
  if (!squareUseMovingPid()) { arm.trajAbort(); return false; }
  freehand_path.duration_s = arm.pathDuration();
  freehand_path.status = 1;
  return true;
}

static void squareHandle() {
  if (arm_step == 0) return;
  if (arm.mode() != Arm::Mode::Joint || !arm.calibrated() ||
      (!arm.trajRunning() && !arm.trajFinished())) {
    // 掉线或轨迹被取消后，本挡位内不自动重启。
    arm.trajAbort();
    arm_step = 0;
    arm_start_ok = false;
    square_debug.status = 4;
    if (freehand_path.status == 1 || freehand_path.status == 2) freehand_path.status = 5;
    return;
  }
  if (!arm.trajFinished()) return;
  if (arm_step == 9) {
    arm_start_ok = arm.trajStartPath();
    if (arm_start_ok) { arm_step=10; freehand_path.status=2; }
    else { arm.trajAbort(); arm_step=0; freehand_path.status=5; }
    return;
  }
  if (arm_step == 10) {
    arm_step=0; freehand_path.status=3; square_debug.status=2;
    return;
  }
  if (arm_step == 5 || arm_step == 6 || arm_step == 8 || arm_step == 12) {
    if (arm_step != 6) square_debug.cycle_time_ms = HAL_GetTick() - square_cycle_start_tick;
    square_return_ready = arm_step != 6;
    arm_step = 0;  // 整次动作结束，保持终点，等待回中后重新触发。
    square_debug.status = 2;
    return;
  }
  const int vertex = arm_step % 4;  // 预备结束→B，随后 C、D、A。
  if (arm_step == 11) {
    // 预检采用连续展开的累计角；保存实际预备终点作为本次回卷目标。
    square_return_q[0]=arm.reference().q[0]; square_return_q[1]=arm.reference().q[1];
  }
  arm_start_ok = arm_step == 11 ? arm.trajStartRoundedSquare() :
      arm_step == 7 ? arm.trajStartCircle(4.f * square_edge_duration) :
      arm.trajStart(square::vertices[vertex][0], square::vertices[vertex][1], square_edge_duration);
  if (arm_start_ok) {
    if (arm_step == 1 || arm_step == 7 || arm_step == 11) {
      square_cycle_start_tick = HAL_GetTick();
      square_debug.cycle_time_ms = 0;
      for (int i = 0; i < 2; ++i) {
        square_debug.command_peak_a[i] = 0.f;
        square_debug.feedback_peak_a[i] = 0.f;
      }
    }
    ++arm_step;
  } else {
    arm.trajAbort();
    arm_step = 0;
    square_debug.status = 4;
  }
}

// 机器人控制
void robotControl(void) {
  const bool changed = rc.switch_.l != last_rc_switch.l ||
                       rc.switch_.r != last_rc_switch.r;
  if (changed) {
    draw_trigger_ready = false;
    // 先恢复绘图死区，再切换参数，避免覆盖任务二的独立死区。
    squareRestorePid();
    setJointStepProfile(rc.switch_.l == decltype(rc)::UP && rc.switch_.r == decltype(rc)::UP);
  }
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
  const bool draw_square = rc.switch_.l == decltype(rc)::MID &&
                           rc.switch_.r == decltype(rc)::UP;
  const bool tune_hold = rc.switch_.l == decltype(rc)::UP &&
                         rc.switch_.r == decltype(rc)::MID;
  if (!tune_hold && square_debug.request != 0) {
    square_debug.request = 0;
    square_debug.status = 3;
  }
  if (changed && !draw_square) {
    if (arm_step != 0) {
      if (arm_step == 9 || arm_step == 10) freehand_path.status=5;
      arm.trajAbort();
      square_debug.status = 4;
    }
    arm_step = 0;
    arm_start_ok = false;
  }
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
  } else if (draw_square) {
    if (changed) {
      arm.disable();
      arm_step = 0;
      arm_start_ok = false;
      const bool enabled = centered && arm.enable(Arm::Mode::Joint);
      square_debug.status = enabled ? 0 : 3;
    }
    // 右摇杆下满杆取消任意轨迹并保持当前规划目标。
    if ((arm_step == 9 || arm_step == 10) &&
        rc.channel_.r_col <= -manual::stick_range * .9f) {
      arm.trajAbort(); arm_step=0; arm_start_ok=false;
      freehand_path.status=5; draw_trigger_ready=false;
    }
    // 回中后才接受下一次端点动作；忙碌期间不缓存摇杆请求。
    if (arm_step == 0 && arm.mode() == Arm::Mode::Joint && arm.calibrated()) {
      if (centered) draw_trigger_ready = true;
      if (draw_trigger_ready && fabsf(static_cast<float>(rc.channel_.l_col)) <= manual::stick_deadband &&
          rc.channel_.r_col >= manual::stick_range * .9f) {
        draw_trigger_ready=false;
        arm_start_ok=freehandStart();
        if (arm_start_ok) { arm_step=9; square_debug.status=1; }
        else freehand_path.status=4;
      }
      if (draw_trigger_ready && fabsf(static_cast<float>(rc.channel_.r_col)) <= manual::stick_deadband &&
          fabsf(static_cast<float>(rc.channel_.l_col)) >= manual::stick_range * 0.9f) {
        draw_trigger_ready = false;
        const bool circle = rc.channel_.l_col < 0;
        arm_start_ok = squareStart(circle);
        if (arm_start_ok) arm_step = circle ? 7 : (square_corner_radius > 0.f ? 11 : 1);
        square_debug.status = arm_start_ok ? 1 : 3;
      }
    }
    squareHandle();
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
        arm.setJointCurrent(input0 * manual::j0_current_limit * manual::current_to_raw,
                             input1 * manual::j1_current_limit * manual::current_to_raw);
      }
    }
  } else {
    // 其他挡位保持当前位置；从速度/电流模式退出时重新捕获位置。
    if (arm.mode() == Arm::Mode::Speed || arm.mode() == Arm::Mode::Current)
      arm.enable(Arm::Mode::Joint);
    const uint32_t request = square_debug.request;
    square_debug.request = 0;
    if (request == 2) {
      arm.trajAbort();
      arm_step = 0;
      arm_start_ok = false;
      square_debug.status = 4;
    } else if (request != 0) {
      if (tune_hold && centered && arm_step == 0 &&
          arm.mode() == Arm::Mode::Joint && arm.calibrated() &&
          (request == 1 || (request == 3 && square_return_ready))) {
        if (request == 1) {
          arm_start_ok = squareStart();
          if (arm_start_ok) arm_step = square_corner_radius > 0.f ? 11 : 1;
        } else {
          // 只回到本次 A 点预备角，不用电脑逐点写目标，避免目标阶梯激励 D 项。
          arm_start_ok = squareUseMovingPid() && arm.setTrajectorySpeedGain(1.f, 1.f) &&
              arm.trajStartJoint(square_return_q[0], square_return_q[1], square::return_duration);
          if (arm_start_ok) {
            arm_step = 6;
            square_return_ready = false;
          }
        }
        square_debug.status = arm_start_ok ? 1 : 3;
      } else {
        square_debug.status = 3;
      }
    }
    if (arm_step != 0 && !centered) {
      arm.disable();  // 调试中出现摇杆输入，立即结束；不自动重启。
      arm_step = 0;
      arm_start_ok = false;
      square_debug.status = 4;
    }
    if (arm_step != 0) squareHandle();
    else arm.trajAbort();
  }
  last_rc_switch = rc.switch_;
}

// 先显示阻止使能的原因，再显示实际运行模式。
void boardLedHandle(void) {
  const bool motor0_ready = J0.connect_.check() && J0.mode_ != Motor::INIT;
  const bool motor1_ready = J1.connect_.check() && J1.mode_ != Motor::INIT;
  if (!rc_connected) {
    led_status = LED_RC_LOST;
  } else if (robot_state == STOP || rc.switch_.r == decltype(rc)::DOWN) {
    led_status = LED_STOPPED;
  } else if (!motor0_ready && !motor1_ready) {
    led_status = LED_BOTH_MOTORS_LOST;
  } else if (!motor0_ready) {
    led_status = LED_J0_LOST;
  } else if (!motor1_ready) {
    led_status = LED_J1_LOST;
  } else if (!std::isfinite(J0.realAngle()) || !std::isfinite(J1.realAngle()) ||
             !std::isfinite(J0.realSpeed()) || !std::isfinite(J1.realSpeed())) {
    led_status = LED_FEEDBACK_INVALID;
  } else if (!arm.calibrated()) {
    led_status = LED_NEED_CALIBRATION;
  } else if (rc.switch_.l == decltype(rc)::MID && rc.switch_.r == decltype(rc)::MID) {
    led_status = LED_CALIBRATED;
  } else if (arm.mode() == Arm::Mode::Joint) {
    if (arm_step != 0) {
      led_status = LED_SQUARE;
    } else {
      led_status = rc.switch_.l == decltype(rc)::UP && rc.switch_.r == decltype(rc)::UP
                     ? LED_POSITION : LED_HOLDING;
    }
  } else if (arm.mode() == Arm::Mode::Speed) {
    led_status = LED_SPEED;
  } else if (arm.mode() == Arm::Mode::Current) {
    led_status = LED_CURRENT;
  } else {
    led_status = LED_DISABLED;
  }

#ifdef DBC
  led.setModeOn();
  switch (led_status) {
    case LED_RC_LOST:
      led.setColor(255, 0, 0);       // 红色快闪：未收到遥控数据
      led.setModeBlink(1, 100, 100);
      break;
    case LED_STOPPED:
      led.setColor(255, 0, 0);       // 红色常亮：遥控在线，右下失能
      break;
    case LED_J0_LOST:
    case LED_J1_LOST:
    case LED_BOTH_MOTORS_LOST:
      led.setColor(255, 80, 0);      // 橙色：每组闪 1/2/3 次表示 J0/J1/两台无反馈
      led.setModeBlink(led_status == LED_J0_LOST ? 1 :
                       led_status == LED_J1_LOST ? 2 : 3);
      break;
    case LED_FEEDBACK_INVALID:
      led.setColor(255, 80, 0);      // 橙色常亮：角度或速度不是有效数值
      break;
    case LED_NEED_CALIBRATION:
      led.setColor(255, 255, 0);     // 黄色：反馈正常，尚未标定
      break;
    case LED_CALIBRATED:
      led.setColor(0, 255, 255);     // 青色：零位标定成功，仍处于失能状态
      break;
    case LED_POSITION:
      led.setColor(0, 255, 0);       // 绿色：位置双环已使能
      break;
    case LED_SPEED:
      led.setColor(0, 0, 255);       // 蓝色：速度环已使能
      break;
    case LED_CURRENT:
      led.setColor(255, 0, 255);     // 紫色：直接电流已使能
      break;
    case LED_HOLDING:
      led.setColor(0, 255, 0);       // 绿色呼吸：保持位置
      led.setModeBreath();
      break;
    case LED_SQUARE:
      led.setColor(255, 255, 255);   // 白色呼吸：预备动作或正在绘图
      led.setModeBreath();
      break;
    default:
      led.setColor(255, 255, 255);   // 白色闪烁：已标定但未使能，回中后重新拨挡
      led.setModeBlink();
      break;
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
