/**
 ******************************************************************************
 * @file    motor.cpp/h
 * @brief   Motor control. 电机控制
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "base/motor/motor.h"
#include "base/common/math.h"

namespace motor {

// max time to wait for connect(ms)
const uint32_t connect_timeout = 50;

// Kalman filter parameters
const float kf_F[4] = {1, 1e-3f, 0, 1};
const float kf_B[2] = {0, 1};
const float kf_H[2] = {1, 0};
const float kf_x0[2] = {0, 0};

}  // namespace motor

// Motor构造函数:
Motor::Motor(const Type_e& type, const float& ratio,
             const ControlMethod_e& method, const PID& ppid, const PID& spid,
             bool use_kf, const KFParam_t& kf_param,
             float (*model)(const float&, const float&))
    : connect_(motor::connect_timeout),
      ratio_(ratio),
      method_(method),
      ppid_(ppid),
      spid_(spid),
      model_(model),
      use_kf_(use_kf),
      kf_(2, 1, 1, motor::kf_F, motor::kf_B, motor::kf_H, kf_param.Q,
          kf_param.R, motor::kf_x0) {
  info_.type = type;
  info_.can_channel = 0;
  info_.id = 0;
  // 设定控制信号最大范围
  if (info_.type == Motor::M3508) {
    info_.max_intensity = 16384;
  } else if (info_.type == Motor::M2006) {
    info_.max_intensity = 10000;
  } else if (info_.type == Motor::GM6020) {
    info_.max_intensity = 30000;
  } else if (info_.type == Motor::MIT) {
    info_.max_intensity = 40;
  }
  mode_ = INIT;
  setFdbSrc(nullptr, nullptr);
  reset();
}

// Reset motor
// 初始化/重置电机
void Motor::reset(void) {
  motor_data_.angle = 0;
  motor_data_.rotate_speed = 0;

  control_data_.target_angle = 0;
  control_data_.fdb_angle = 0;
  control_data_.target_speed = 0;
  control_data_.fdb_speed = 0;
  control_data_.feedforward_speed = 0;
  control_data_.target_torque = 0;
  control_data_.feedforward_intensity = 0;

  ppid_.reset();
  spid_.reset();
  kf_.reset(motor::kf_x0);
  memcpy(kf_data_.x, motor::kf_x0, sizeof(kf_data_.x));
}

// Receive data callback, refresh connect status
// 反馈信息接受回调，刷新连接状态
void Motor::rxCallback(void) {
  if (connect_.refresh() == CONNECT) {
    // Receive data when in unconnected state -> reset motor
    // 断连状态下收到反馈重置电机
    mode_ = INIT;
    reset();
  }
}

// Control handle(1ms)
// 电机控制处理(1ms)
void Motor::handle(void) {
  // Check connection. 检查电机连接状态
  if (!connect_.check()) {
    motor_data_.rotate_speed = 0;
    intensity_ = 0;
    return;
  }

  // Initialization 初始化编码器角度记录
  if (mode_ == INIT) {
    motor_data_.last_ecd_angle = motor_data_.ecd_angle;
    control_data_.last_fdb_angle = *control_data_.fdb_angle_src;
    mode_ = POWEROFF;
    return;
  }

  // Kalman filter update
  // 卡尔曼滤波器更新
  if (use_kf_) {
    kf_data_.u[0] = 0;  // torq/Im
    kf_data_.z[0] = motor_data_.angle;
    // 每周期根据 Q、R 递推协方差 P 和增益 K
    kf_.update(kf_data_.x, kf_data_.u, kf_data_.z, 2, 1, 1);
  }

  // Update control feedback (fdb_src->fdb)
  // 更新控制反馈 (fdb_src->fdb)
  // 提取fdb_angle(两次指针取值间fdb_angle_src的值可能被改变)
  float fdb_angle_src = *control_data_.fdb_angle_src;
  control_data_.fdb_angle +=
      math::degNormalize180(fdb_angle_src - control_data_.last_fdb_angle);
  control_data_.fdb_speed = *control_data_.fdb_speed_src;
  control_data_.last_fdb_angle = fdb_angle_src;

  // Calculate intensity(control value)
  // 计算控制量
  if (mode_ == WORKING) {
    if (method_ == Motor::POSITION_SPEED) {  // 位置速度双环
      control_data_.target_speed =
          ppid_.calc(control_data_.target_angle, control_data_.fdb_angle) +
          control_data_.feedforward_speed;
      intensity_float_ =
          spid_.calc(control_data_.target_speed, control_data_.fdb_speed) +
          control_data_.feedforward_intensity;
    } else if (method_ == Motor::POSITION) {  // 单位置环
      intensity_float_ =
          ppid_.calc(control_data_.target_angle, control_data_.fdb_angle) +
          control_data_.feedforward_intensity;
    } else if (method_ == Motor::SPEED) {  // 单速度环
      control_data_.target_angle = control_data_.fdb_angle;
      intensity_float_ =
          spid_.calc(control_data_.target_speed, control_data_.fdb_speed) +
          control_data_.feedforward_intensity;
    } else if (method_ == Motor::TORQUE) {
      if (model_ != nullptr) {
        // 有电机输出模型-力矩控制
        intensity_float_ =
            model_(control_data_.target_torque, control_data_.fdb_speed);
      } else {
        // 无电机输出模型-直接设置控制量
        intensity_float_ = control_data_.target_torque;
      }
    }
  } else if (mode_ == STOP) {
    if (method_ == Motor::POSITION_SPEED || method_ == Motor::SPEED) {
      control_data_.target_speed = 0;
      control_data_.feedforward_speed = 0;
      control_data_.feedforward_intensity = 0;
      intensity_float_ =
          spid_.calc(control_data_.target_speed, control_data_.fdb_speed);
    } else if (method_ == Motor::POSITION || method_ == Motor::TORQUE) {
      intensity_float_ = 0;
    }
  } else if (mode_ == POWEROFF || mode_ == INIT) {
    intensity_float_ = 0;
  }
  // 根据减速比确定输出方向
  intensity_float_ = math::limit(intensity_float_ * math::sign(ratio_),
                                 -info_.max_intensity, info_.max_intensity);
  // 输出限幅
  intensity_ = intensity_float_;
}

// Configure CAN id
// 配置CAN通道和id
void Motor::CANIdConfig(const uint8_t& can_channel, const uint8_t& id) {
  info_.can_channel = can_channel;
  info_.id = id;
}

// Set target angle, feedforward speed/intensity
// 设置目标角度，角速度/输出前馈
void Motor::setAngleSpeed(const float& target_angle, const float& ff_speed,
                          const float& ff_intensity) {
  control_data_.target_angle = target_angle;
  control_data_.feedforward_speed = ff_speed;
  if (model_ != nullptr) {
    control_data_.feedforward_intensity =
        model_(ff_intensity, control_data_.fdb_speed);
  } else {
    control_data_.feedforward_intensity = ff_intensity;
  }
}

// Set target angle, feedforward intensity
// 设置目标角度，输出前馈
void Motor::setAngle(const float& target_angle, const float& ff_intensity) {
  control_data_.target_angle = target_angle;
  if (model_ != nullptr) {
    control_data_.feedforward_intensity =
        model_(ff_intensity, control_data_.fdb_speed);
  } else {
    control_data_.feedforward_intensity = ff_intensity;
  }
}

// Set target speed, feedforward intensity
// 设置目标角速度，输出前馈
void Motor::setSpeed(const float& target_speed, const float& ff_intensity) {
  control_data_.target_speed = target_speed;
  if (model_ != nullptr) {
    control_data_.feedforward_intensity =
        model_(ff_intensity, control_data_.fdb_speed);
  } else {
    control_data_.feedforward_intensity = ff_intensity;
  }
}

// Set control feedback data source (important! unit: deg, dps)
// 设置控制反馈数据源 (重要! 单位: deg, dps)
// nullptr: use motor_data_ as feedback source
// 空指针: 使用motor_data_作为反馈数据源
void Motor::setFdbSrc(float* fdb_angle_src, float* fdb_speed_src) {
  if (fdb_angle_src != nullptr) {
    if (control_data_.fdb_angle_src != fdb_angle_src) {
      control_data_.fdb_angle_src = fdb_angle_src;
      control_data_.last_fdb_angle = *fdb_angle_src;
    }
  } else {
    // fdb_angle_src == nullptr
    if (control_data_.fdb_angle_src != &motor_data_.angle) {
      control_data_.fdb_angle_src = &motor_data_.angle;
      control_data_.last_fdb_angle = motor_data_.angle;
    }
  }
  if (fdb_speed_src != nullptr) {
    control_data_.fdb_speed_src = fdb_speed_src;
  } else {
    control_data_.fdb_speed_src = &motor_data_.rotate_speed;
  }
}

// Reset control feedback angle to specific value
// 将控制反馈角度(control_data_.angle)重置为特定角度
void Motor::resetFeedbackAngle(const float& angle) {
  control_data_.fdb_angle = angle;
}
