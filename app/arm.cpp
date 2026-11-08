//
// Created by NoSignal on 2026/10/16.
//

#include "arm.h"
#include <cmath>
#include "base/common/math.h"
#include "main.h"

Arm::Arm(Motor& joint0, Motor& joint1, const Limits& limits,
         const Feedforward& feedforward)
    : joint0_(joint0), joint1_(joint1), limits_(limits),
      feedforward_(feedforward) {}

bool Arm::feedbackReady() {
  if (!joint0_.connect_.check() || !joint1_.connect_.check() ||
      joint0_.mode_ == Motor::INIT || joint1_.mode_ == Motor::INIT) {
    calibrated_ = false;
    return false;
  }
  const bool valid = std::isfinite(joint0_.realAngle()) &&
                     std::isfinite(joint1_.realAngle()) &&
                     std::isfinite(joint0_.realSpeed()) &&
                     std::isfinite(joint1_.realSpeed());
  if (!valid) calibrated_ = false;
  return valid;
}

bool Arm::withinLimits(const float q[2]) const {
  for (int i = 0; i < 2; ++i) {
    if (!std::isfinite(q[i]) || q[i] < limits_.q_min[i] ||
        q[i] > limits_.q_max[i]) return false;
  }
  return true;
}

bool Arm::calibrate(float q0, float q1) {
  const float q[2] = {q0, q1};
  if (mode_ != Mode::Disabled || !feedbackReady() || !withinLimits(q))
    return false;
  joint0_.resetFeedbackAngle(math::rad2deg(q0));
  joint1_.resetFeedbackAngle(math::rad2deg(q1));
  updateFeedback();
  reference_ = Reference{};
  reference_.q[0] = q0;
  reference_.q[1] = q1;
  calibrated_ = true;
  return true;
}

bool Arm::enable(Mode mode) {
  if (mode == Mode::Disabled || !feedbackReady() || !calibrated_)
    return false;
  updateFeedback();
  if (!withinLimits(state_.q)) return false;
  if (mode_ == mode) return true;
  trajAbort();
  reference_ = Reference{};
  for (int i = 0; i < 2; ++i) {
    reference_.q[i] = state_.q[i];
    speed_reference_[i] = current_reference_[i] = 0.f;
  }
  joint0_.ppid_.reset();
  joint0_.spid_.reset();
  joint1_.ppid_.reset();
  joint1_.spid_.reset();
  mode_ = mode;
  joint0_.mode_ = joint1_.mode_ = Motor::WORKING;
  writeMotorCommands();
  return true;
}

void Arm::disable() {
  trajAbort();
  mode_ = Mode::Disabled;
  speed_reference_[0] = speed_reference_[1] = 0.f;
  current_reference_[0] = current_reference_[1] = 0.f;
  if (joint0_.mode_ != Motor::INIT) joint0_.mode_ = Motor::POWEROFF;
  if (joint1_.mode_ != Motor::INIT) joint1_.mode_ = Motor::POWEROFF;
  joint0_.intensity_ = joint1_.intensity_ = 0;
  joint0_.intensity_float_ = joint1_.intensity_float_ = 0.f;
  joint0_.setAngleSpeed(joint0_.realAngle());
  joint1_.setAngleSpeed(joint1_.realAngle());
}

bool Arm::setReference(const Reference& reference) {
  if (mode_ != Mode::Joint || trajRunning()) return false;
  return applyReference(reference);
}

bool Arm::applyReference(const Reference& reference) {
  if (!withinLimits(reference.q)) return false;
  for (int i = 0; i < 2; ++i) {
    if (!std::isfinite(reference.dq[i]) || !std::isfinite(reference.ddq[i]))
      return false;
  }
  reference_ = reference;
  return true;
}

bool Arm::addJointReference(float speed0, float speed1, float dt) {
  if (mode_ != Mode::Joint || trajRunning() || !std::isfinite(dt) || dt <= 0.f ||
      !std::isfinite(speed0) || !std::isfinite(speed1)) return false;
  const float speed[2] = {speed0, speed1};
  Reference next{};
  for (int i = 0; i < 2; ++i) {
    next.q[i] = math::limit(reference_.q[i] + speed[i] * dt,
                            limits_.q_min[i], limits_.q_max[i]);
  }
  // 调位置环时先不用速度前馈，摇杆回中保持积分后的目标。
  return applyReference(next);
}

bool Arm::setJointSpeed(float speed0, float speed1) {
  if (mode_ != Mode::Speed || !std::isfinite(speed0) || !std::isfinite(speed1))
    return false;
  speed_reference_[0] = speed0;
  speed_reference_[1] = speed1;
  return true;
}

bool Arm::setJointCurrent(float current0, float current1) {
  if (mode_ != Mode::Current || !std::isfinite(current0) || !std::isfinite(current1))
    return false;
  current_reference_[0] = current0;
  current_reference_[1] = current1;
  return true;
}

bool Arm::trajectoryReady() {
  if (mode_ != Mode::Joint || !calibrated_ || trajRunning() || !feedbackReady())
    return false;
  updateFeedback();
  return withinLimits(state_.q) && withinLimits(reference_.q);
}

bool Arm::trajStart(float x, float y, float duration_s) {
  if (!std::isfinite(duration_s) || duration_s <= 0.f || duration_s > 2147483.f ||
      !trajectoryReady()) return false;
  const CartesianReference start = ArmKinematics::forward(reference_.q);
  CartesianReference end{};
  end.x = x; end.y = y;
  if (!ArmKinematics::lineReachable(start, end)) return false;

  // 整段固定肘支；伸直/折叠的临界姿态沿用上一次的选择。
  int elbow = trajectory_elbow_;
  const float sin_q1 = sinf(reference_.q[1]);
  if (fabsf(sin_q1) > 1e-5f) elbow = sin_q1 > 0.f ? 1 : -1;
  float previous_q[2] = {reference_.q[0], reference_.q[1]};
  // 几何可达性已解析判断；这里另检查沿途的累计关节角限位。
  for (int i = 0; i <= 128; ++i) {
    const float u = i / 128.f;
    CartesianReference point{};
    point.x = start.x + (end.x - start.x) * u;
    point.y = start.y + (end.y - start.y) * u;
    float q[2];
    if (!ArmKinematics::inverse(point, previous_q, elbow, q) || !withinLimits(q))
      return false;
    previous_q[0] = q[0]; previous_q[1] = q[1];
  }
  if (!trajectory_.start(start, end, duration_s)) return false;
  trajectory_elbow_ = elbow;
  trajectory_start_tick_ = HAL_GetTick();
  trajectory_type_ = TrajectoryType::Cartesian;
  trajectory_finished_ = false;
  reference_.dq[0] = reference_.dq[1] = 0.f;
  reference_.ddq[0] = reference_.ddq[1] = 0.f;
  return true;
}

bool Arm::trajStartJoint(float q0, float q1, float duration_s) {
  const float end[2] = {q0, q1};
  if (!std::isfinite(duration_s) || duration_s <= 0.f || duration_s > 2147483.f ||
      !withinLimits(end) || !trajectoryReady()) return false;
  // 两轴使用相同的归一化时间，从实际反馈同步起步。
  for (int i = 0; i < 2; ++i) {
    joint_trajectory_start_[i] = state_.q[i];
    joint_trajectory_end_[i] = end[i];
    reference_.q[i] = state_.q[i];
    reference_.dq[i] = reference_.ddq[i] = 0.f;
  }
  joint_trajectory_duration_ = duration_s;
  trajectory_start_tick_ = HAL_GetTick();
  trajectory_type_ = TrajectoryType::Joint;
  trajectory_finished_ = false;
  return true;
}

void Arm::trajAbort() {
  trajectory_.abort();
  trajectory_type_ = TrajectoryType::None;
  trajectory_finished_ = false;
  for (int i = 0; i < 2; ++i) {
    reference_.dq[i] = reference_.ddq[i] = 0.f;
  }
}

bool Arm::setTrajectorySpeedGain(float gain0, float gain1) {
  if (trajRunning() || !std::isfinite(gain0) || !std::isfinite(gain1) ||
      gain0 < 0.f || gain0 > 1.5f || gain1 < 0.f || gain1 > 1.5f) return false;
  trajectory_speed_gain_[0] = gain0;
  trajectory_speed_gain_[1] = gain1;
  return true;
}

void Arm::trajectoryPlanner() {
  if (!trajRunning()) return;
  if (mode_ != Mode::Joint) {
    trajAbort();
    return;
  }
  // 无符号相减允许系统毫秒计数跨过一次溢出。
  const float elapsed_s = (HAL_GetTick() - trajectory_start_tick_) * 0.001f;
  Reference next{};
  bool finished = false;
  if (trajectory_type_ == TrajectoryType::Cartesian) {
    const CartesianReference point = trajectory_.sample(elapsed_s);
    if (!ArmKinematics::inverse(point, reference_.q, trajectory_elbow_, next.q)) {
      trajAbort();
      return;
    }
    if (trajectory_speed_gain_[0] != 0.f || trajectory_speed_gain_[1] != 0.f) {
      float velocity[2], acceleration[2];
      if (!ArmKinematics::derivatives(point, next.q, velocity, acceleration)) {
        trajAbort();
        return;
      }
      for (int i = 0; i < 2; ++i) next.dq[i] = trajectory_speed_gain_[i] * velocity[i];
    }
    finished = !trajectory_.running();
  } else {
    for (int i = 0; i < 2; ++i) {
      float velocity, acceleration;
      ArmTrajectory::interpolate(joint_trajectory_start_[i], joint_trajectory_end_[i],
                                 elapsed_s, joint_trajectory_duration_, next.q[i],
                                 velocity, acceleration);
      next.dq[i] = trajectory_speed_gain_[i] * velocity;
    }
    finished = elapsed_s >= joint_trajectory_duration_;
  }
  // 位置纠偏与轨迹速度前馈相加；加速度前馈仍为零，单独比较速度前馈效果。
  if (!applyReference(next)) {
    trajAbort();
    return;
  }
  if (finished) {
    trajectory_type_ = TrajectoryType::None;
    trajectory_finished_ = true;
  }
}

void Arm::updateFeedback() {
  state_.q[0] = math::deg2rad(joint0_.realAngle());
  state_.q[1] = math::deg2rad(joint1_.realAngle());
  state_.dq[0] = math::deg2rad(joint0_.realSpeed());
  state_.dq[1] = math::deg2rad(joint1_.realSpeed());
}

void Arm::handle() {
  if (!feedbackReady()) {
    disable();
    return;
  }
  updateFeedback();
  if (mode_ != Mode::Disabled && !withinLimits(state_.q)) disable();
  trajectoryPlanner();
  writeMotorCommands();
}

void Arm::writeMotorCommands() {
  Motor* joints[2] = {&joint0_, &joint1_};
  for (int i = 0; i < 2; ++i) {
    Motor& motor = *joints[i];
    if (mode_ == Mode::Disabled) {
      // INIT 必须交给 Motor::handle 完成，不能在这里覆盖。
      if (motor.mode_ != Motor::INIT) motor.mode_ = Motor::POWEROFF;
      continue;
    }
    motor.mode_ = Motor::WORKING;
    if (mode_ == Mode::Joint) {
      motor.method_ = Motor::POSITION_SPEED;
      motor.setAngleSpeed(math::rad2deg(reference_.q[i]),
                          math::rad2deg(reference_.dq[i]),
                          feedforward_.k_acc[i] * reference_.ddq[i]);
    } else {
      float command = mode_ == Mode::Speed ? speed_reference_[i] : current_reference_[i];
      if ((state_.q[i] <= limits_.q_min[i] && command < 0.f) ||
          (state_.q[i] >= limits_.q_max[i] && command > 0.f)) command = 0.f;
      if (mode_ == Mode::Speed) {
        motor.method_ = Motor::SPEED;
        motor.setSpeed(math::rad2deg(command));
      } else {
        motor.method_ = Motor::TORQUE;
        motor.targetTorque() = command;
      }
    }
  }
}
