//
// Created by NoSignal on 2026/10/16.
//

#include "arm.h"
#include <cmath>
#include "base/common/math.h"

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
  return false;
}

bool Arm::trajStart(float, float, float) {
  return false;
}

bool Arm::trajStartJoint(float, float, float) {
  return false;
}

void Arm::trajAbort() {
  trajectory_.abort();
  trajectory_type_ = TrajectoryType::None;
  trajectory_finished_ = false;
  for (int i = 0; i < 2; ++i) {
    reference_.dq[i] = reference_.ddq[i] = 0.f;
  }
}

void Arm::trajectoryPlanner() {
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
