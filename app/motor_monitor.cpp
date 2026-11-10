/**
 ******************************************************************************
 * @file    motor_monitor.cpp/h
 * @brief   Motor parameter, id config and management. 电机参数id配置和统一管理
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/motor_monitor.h"
#include "app/manual_config.h"
#include "app/step_config.h"
#include "app/step_feedforward.h"
#include "app/arm.h"
#include "app/control.h"
#include "base/common/math.h"
#include "lib/arm_math/arm_math.h"
#include <limits>

// Motor parameter config
// 电机参数配置

// Chassis motor 底盘电机
// const PID chassis_wheel_spid(40, 1, 10, 1000, 16384);
// Motor CMFL(Motor::M3508, 3591.f / 187.f, Motor::SPEED,  // type, ratio, method
//            PID(), PID(chassis_wheel_spid));             // ppid, spid
// Motor CMFR(Motor::M3508, 3591.f / 187.f, Motor::SPEED,  // type, ratio, method
//            PID(), PID(chassis_wheel_spid));             // ppid, spid
// Motor CMBL(Motor::M3508, 3591.f / 187.f, Motor::SPEED,  // type, ratio, method
//            PID(), PID(chassis_wheel_spid));             // ppid, spid
// Motor CMBR(Motor::M3508, 3591.f / 187.f, Motor::SPEED,  // type, ratio, method
//            PID(), PID(chassis_wheel_spid));             // ppid, spid

// Gimbal motor 云台电机
// Motor GMY(Motor::GM6020, 1, Motor::POSITION_SPEED,   // type, ratio, method
//           PID(25, 0.1, 10, 10, 1800),                // ppid
//           PID(400, 2, 0, 500, 30000),                // spid
//           true);                                     // use kf
// Motor GMP(Motor::GM6020, -1, Motor::POSITION_SPEED,  // type, ratio, method
//           PID(25, 0.1, 10, 10, 1800),                // ppid
//           PID(400, 2, 0, 500, 30000),                // spid
//           true);                                     // use kf

// Friction wheel motors
// const PID fric_spid(6, 0.05, 2, 200, 16384);
// Motor FRICL(Motor::M3508, 1, Motor::SPEED,  // type, ratio, method
//             PID(), PID(fric_spid));         // ppid, spid
// Motor FRICR(Motor::M3508, 1, Motor::SPEED,  // type, ratio, method
//             PID(), PID(fric_spid));         // ppid, spid

// Stir motor
// Motor STIR(Motor::M2006, 36, Motor::POSITION_SPEED,  // type, ratio, method
//            PID(20, 0.1, 10, 10, 2500),               // ppid
//            PID(60, 0.1, 200, 1000, 10000));          // spid

// 两关节配置：CAN1 上 J1 ID=1、J0 ID=2；现有 PID 参数作为调试起点。
// Other motors
// 在线 KF Q 为 deg²、(deg/s)²，R 为 deg²
const Motor::KFParam_t joint_kf_param(5.f, 6e5f, 1.f);
// D 项 EMA 系数，范围 (0, 1]；越小越平滑，1 表示不滤波。
constexpr float joint_d_filter_k = 0.1f;
// 参数顺序：Kp、Ki、Kd、积分误差累加上限、输出上限、D 项 EMA 系数、误差软死区。
const PID J0_spid(30,0.01,0,0,manual::j0_current_limit * manual::current_to_raw,joint_d_filter_k);
const PID J0_ppid(75,0,230,0,std::numeric_limits<float>::max(),joint_d_filter_k,manual::j0_position_deadband);
// 同步轮实测 40.43 / 64.68 mm，暂沿用原方向符号。
Motor J0(Motor::M3508, manual::j0_ratio, Motor::POSITION_SPEED,
         J0_ppid,           // ppid
         J0_spid,           // spid
         true, joint_kf_param);  // 启用在线 KF

const PID J1_spid(12,0,50,0,manual::j1_current_limit * manual::current_to_raw,joint_d_filter_k);
const PID J1_ppid( 50,0,75,0,std::numeric_limits<float>::max(),joint_d_filter_k,manual::j1_position_deadband);
Motor J1(Motor::M3508, manual::j1_ratio, Motor::POSITION_SPEED,
         J1_ppid,
         J1_spid,
         true, joint_kf_param);

MotorDebugData j0_debug{}, j1_debug{};
StepTrace step_trace{};
StepFeedforward step_feedforward{};
static bool step_profile_active = false;
static StepFeedforwardLatch step_bias[2];
extern Arm arm;

static void updateStepTrace() {
  static float previous_target[2]{};
  static uint32_t phase = 0, stride = 1;
  static uint32_t last_tick = 0;
  const bool ready = (rc_connected || bench_debug.active) && arm.calibrated() &&
      arm.mode() == Arm::Mode::Joint && J0.mode_ == Motor::WORKING &&
      J1.mode_ == Motor::WORKING;
  if (!ready) {
    if (step_trace.request || step_trace.status == 1 || step_trace.status == 2)
      step_trace.status = 4;
    step_trace.request = 0;
    if (step_trace.monitor_status == 1) step_trace.monitor_status = 3;
    return;
  }
  if (step_trace.request) {
    const uint32_t request = step_trace.request;
    step_trace.request = 0;
    step_trace.count = 0;
    step_trace.monitor_status = 0;
    phase = 0;
    stride = step_trace.stride >= 1 && step_trace.stride <= 10 ? step_trace.stride : 1;
    step_trace.stride = stride;
    previous_target[0] = J0.targetAngle();
    previous_target[1] = J1.targetAngle();
    step_trace.status = request == 2 ? 1 : 2;
  }
  if (step_trace.status == 1 &&
      (fabsf(J0.targetAngle() - previous_target[0]) > 1.f ||
       fabsf(J1.targetAngle() - previous_target[1]) > 1.f))
    step_trace.status = 2;
  const uint32_t now = HAL_GetTick();
  if (step_trace.status == 2 && step_trace.count == 0 && phase == 0) {
    step_trace.start_ms = last_tick = now;
    step_trace.elapsed_ms = step_trace.max_gap_ms = 0;
    step_trace.monitor_status = 1;
    if (step_trace.monitor_ms < 500 || step_trace.monitor_ms > 10000)
      step_trace.monitor_ms = 2500;
    if (!std::isfinite(step_trace.band_deg) || step_trace.band_deg <= 0.f)
      step_trace.band_deg = .2f;
    for (int j = 0; j < 2; ++j) {
      step_trace.last_outside_ms[j] = 0;
      step_trace.max_error_after_100ms[j] = step_trace.peak_current[j] = 0;
    }
    previous_target[0] = J0.targetAngle();
    previous_target[1] = J1.targetAngle();
  }
  if (step_trace.monitor_status == 1) {
    const uint32_t elapsed = now - step_trace.start_ms, gap = now - last_tick;
    step_trace.elapsed_ms = elapsed;
    if (gap > step_trace.max_gap_ms) step_trace.max_gap_ms = gap;
    last_tick = now;
    Motor* monitored[2] = {&J0, &J1};
    for (int j = 0; j < 2; ++j) {
      Motor& motor = *monitored[j];
      if (fabsf(motor.targetAngle() - previous_target[j]) > .001f) {
        step_trace.monitor_status = 3; // 目标再次改变，此段统计作废。
        break;
      }
      const float raw_angle = motor.realAngle() + motor.motor_data_.angle -
                              *motor.control_data_.fdb_angle_src;
      const float error = fabsf(motor.targetAngle() - raw_angle);
      if (error > step_trace.band_deg) step_trace.last_outside_ms[j] = elapsed;
      if (elapsed >= 100 && error > step_trace.max_error_after_100ms[j])
        step_trace.max_error_after_100ms[j] = error;
      const float current = fabsf(motor.intensity_ / manual::current_to_raw);
      if (current > step_trace.peak_current[j]) step_trace.peak_current[j] = current;
    }
    if (step_trace.monitor_status == 1 && elapsed >= step_trace.monitor_ms)
      step_trace.monitor_status = 2;
  }
  if (step_trace.status != 2 || phase++ % stride) return;
  // 首个样本就是控制器接受新目标的周期，含同周期实际下发电流。
  StepTraceSample& sample = step_trace.samples[step_trace.count];
  sample.tick_ms = now;
  Motor* joints[2] = {&J0, &J1};
  for (int j = 0; j < 2; ++j) {
    Motor& motor = *joints[j];
    sample.target[j] = motor.targetAngle();
    sample.angle[j] = motor.realAngle();
    // 原始编码器换到标定后的同一坐标，防止用滤波曲线掩盖实际振动。
    sample.raw_angle[j] = motor.realAngle() + motor.motor_data_.angle -
                          *motor.control_data_.fdb_angle_src;
    sample.speed[j] = motor.realSpeed();
    sample.current[j] = motor.intensity_ * math::sign(motor.ratio_) / manual::current_to_raw;
  }
  if (++step_trace.count == 256) step_trace.status = 3;
}

void setJointStepProfile(bool enabled) {
  static PID saved_position[2], saved_speed[2];
  static float* saved_speed_source[2]{};
  if (step_profile_active == enabled) return;
  Motor* joints[2] = {&J0, &J1};
  for (int j = 0; j < 2; ++j) {
    Motor& motor = *joints[j];
    if (enabled) {
      saved_position[j] = motor.ppid_;
      saved_speed[j] = motor.spid_;
      saved_speed_source[j] = motor.control_data_.fdb_speed_src;
      motor.control_data_.fdb_speed_src = &motor.kf_data_.x[1];
      const auto& p = step::position[j];
      const auto& s = step::speed[j];
      motor.ppid_ = PID(p.kp, p.ki, p.kd, p.i_max, saved_position[j].out_max_,
                        step::position_d_filter[j], step::position_deadband[j]);
      motor.ppid_.derivative_on_measurement_ = step::position_d_on_measurement[j];
      motor.ppid_.reference_alpha_ = step::position_reference_alpha[j];
      motor.spid_ = PID(s.kp, s.ki, s.kd, s.i_max, saved_speed[j].out_max_, step::d_filter);
      const Motor::KFParam_t filter(5.f, step::kf_q_speed[j], 1.f);
      motor.kf_.setQR(filter.Q, filter.R);
    } else {
      motor.ppid_ = saved_position[j];
      motor.spid_ = saved_speed[j];
      motor.control_data_.fdb_speed_src = saved_speed_source[j];
      motor.kf_.setQR(joint_kf_param.Q, joint_kf_param.R);
    }
    motor.ppid_.reset();
    motor.spid_.reset();
    step_bias[j].reset(motor.realAngle());
    step_feedforward.positive_a[j] = enabled ? step::positive_current_a[j] : 0.f;
    step_feedforward.negative_a[j] = enabled ? step::negative_current_a[j] : 0.f;
    step_feedforward.applied_a[j] = 0.f;
  }
  step_profile_active = enabled;
}

static void updateDebug(Motor& motor, MotorDebugData& debug) {
  debug.target_deg = motor.targetAngle();
  debug.angle_deg = motor.realAngle();
  debug.error_deg = debug.target_deg - debug.angle_deg;
  debug.target_dps = motor.targetSpeed();
  debug.speed_dps = motor.realSpeed();
  debug.raw_speed_dps = motor.motor_data_.rotate_speed;
  // 转成关节正方向的指令，便于与反馈速度对比。
  debug.command_a = motor.intensity_ * math::sign(motor.ratio_) / manual::current_to_raw;
  debug.feedback_current_raw = motor.motor_data_.current;
  debug.temperature = motor.motor_data_.temp;
}

// DJI Motor id config, M3508/M2006: 1~8, GM6020: 5~11
// DJI电机ID配置，M3508，M2006可配置范围为1~8，GM6020可配置范围为5~11
Motor* can1_dji_motor[11] = {
    &J1,   // id:1
    &J0,   // id:2
    nullptr,  // id:3
    nullptr,  // id:4
    nullptr,    // id:5
    nullptr,     // id:6
    nullptr,  // id:7
    nullptr,      // id:8
    nullptr,  // id:9
    nullptr,  // id:10
    nullptr   // id:11
};
Motor* can2_dji_motor[11] = {
    nullptr,    // id:1
    nullptr,    // id:2
    nullptr,    // id:3
    nullptr,    // id:4
    nullptr,     // id:5
    nullptr,  // id:6
    nullptr,  // id:7
    nullptr,  // id:8
    nullptr,  // id:9
    nullptr,  // id:10
    nullptr   // id:11
};

DJIMotorDriver dji_motor_driver(can1_dji_motor, can2_dji_motor);

MITMotorDriver mit_motor_driver[] = {}; // slave_id必须小于0x0F

// 电机停转速度上限(dps)
const float motor_stop_rotate_speed_thres = 1200;

// Stop and shut off all motors
// 所有电机先停转再断电
void allMotorsStopShutoff(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr &&
        can1_dji_motor[i]->mode_ != Motor::INIT) {
      if (fabs(can1_dji_motor[i]->motor_data_.rotate_speed) >
          motor_stop_rotate_speed_thres) {
        can1_dji_motor[i]->mode_ = Motor::STOP;
      } else {
        can1_dji_motor[i]->mode_ = Motor::POWEROFF;
      }
    }
    if (can2_dji_motor[i] != nullptr &&
        can2_dji_motor[i]->mode_ != Motor::INIT) {
      if (fabs(can2_dji_motor[i]->motor_data_.rotate_speed) >
          motor_stop_rotate_speed_thres) {
        can2_dji_motor[i]->mode_ = Motor::STOP;
      } else {
        can2_dji_motor[i]->mode_ = Motor::POWEROFF;
      }
    }
  }
  for (auto & i : mit_motor_driver) {
    i.motor_->mode_ = Motor::POWEROFF;
  }
}

// Shut off all motors
// 所有电机断电
void allMotorsShutOff(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr &&
        can1_dji_motor[i]->mode_ != Motor::INIT) {
      can1_dji_motor[i]->mode_ = Motor::POWEROFF;
    }
    if (can2_dji_motor[i] != nullptr &&
        can2_dji_motor[i]->mode_ != Motor::INIT) {
      can2_dji_motor[i]->mode_ = Motor::POWEROFF;
    }
  }
  for (auto & i : mit_motor_driver) {
    i.motor_->mode_ = Motor::POWEROFF;
  }
}

// Stop all motors
// 所有电机停转
void allMotorsStop(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr &&
        can1_dji_motor[i]->mode_ != Motor::INIT) {
      can1_dji_motor[i]->mode_ = Motor::STOP;
    }
    if (can2_dji_motor[i] != nullptr &&
        can2_dji_motor[i]->mode_ != Motor::INIT) {
      can2_dji_motor[i]->mode_ = Motor::STOP;
    }
  }
  for (auto & i : mit_motor_driver) {
    i.motor_->mode_ = Motor::STOP;
  }
}

// Startup all motors
// 所有电机启动
void allMotorsOn(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr &&
        can1_dji_motor[i]->mode_ != Motor::INIT) {
      can1_dji_motor[i]->mode_ = Motor::WORKING;
    }
    if (can2_dji_motor[i] != nullptr &&
        can2_dji_motor[i]->mode_ != Motor::INIT) {
      can2_dji_motor[i]->mode_ = Motor::WORKING;
    }
  }
  for (auto & i : mit_motor_driver) {
    i.motor_->mode_ = Motor::WORKING;
  }
}

// Reset all motors.
// 电机统一初始化
void allMotorsInit(void) {
  dji_motor_driver.idConfig();
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr) {
      can1_dji_motor[i]->reset();
    }
    if (can2_dji_motor[i] != nullptr) {
      can2_dji_motor[i]->reset();
    }
  }
  for (auto & i : mit_motor_driver) {
    i.motor_->reset();
    i.setCmd(mitmotor::MOTOR_MODE);
  }
  // KF 输出仍使用 deg、deg/s；位置环和速度环都使用滤波反馈
  J0.setFdbSrc(&J0.kfAngle(), &J0.kfSpeed());
  J1.setFdbSrc(&J1.kfAngle(), &J1.kfSpeed());
  J0.info_.max_intensity = manual::j0_current_limit * manual::current_to_raw;
  J1.info_.max_intensity = manual::j1_current_limit * manual::current_to_raw;
}

// Handle all motors. Called in motorTask
// 电机统一处理，在motorTask中调用
void allMotorsHandle(void) {
  if (step_profile_active && arm.mode() == Arm::Mode::Joint) {
    Motor* joints[2] = {&J0, &J1};
    for (int j=0;j<2;++j) {
      step_feedforward.applied_a[j] = step_bias[j].update(joints[j]->targetAngle(),
          step_feedforward.positive_a[j], step_feedforward.negative_a[j]);
      // Arm 每周期先写入基础前馈；这里仅叠加一次，最终沿用电机原有电流上限。
      joints[j]->control_data_.feedforward_intensity +=
          step_feedforward.applied_a[j] * manual::current_to_raw;
    }
  }
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr) {
      can1_dji_motor[i]->handle();
    }
    if (can2_dji_motor[i] != nullptr) {
      can2_dji_motor[i]->handle();
    }
  }
  for (auto & i : mit_motor_driver) {
    // call motor::handle() through MITMotorDriver::handle()
    i.handle();
  }
  updateDebug(J0, j0_debug);
  updateDebug(J1, j1_debug);
  manual_mode = static_cast<uint32_t>(arm.mode());
  calibration_ok = arm.calibrated() ? 1u : 0u;
  updateStepTrace();
}

// Check CAN channel and id of received CAN message
// 校验接收信息的CAN通道和ID，调用对应回调函数
void motorsCanRxMsgHandle(CAN_HandleTypeDef* hcan,
                          CAN_RxHeaderTypeDef rx_header, uint8_t* rx_data) {
  if (dji_motor_driver.canRxMsgCheck(hcan, rx_header)) {
    dji_motor_driver.canRxMsgCallback(hcan, rx_header, rx_data);
  }
  for (auto & i : mit_motor_driver) {
    if (i.canRxMsgCheck(hcan, rx_header)) {
      i.canRxMsgCallback(hcan, rx_header, rx_data);
    }
  }
}
