//
// Created by NoSignal on 2026/10/16.
//

#include "arm.h"
#include <cmath>

#include "base/common/math.h"
#include "stm32f4xx_hal.h"

Arm::Arm(Motor& joint0, Motor& joint1, const Limits& limits,
         const Feedforward& feedforward)
    : joint0_(joint0), joint1_(joint1), limits_(limits),
      feedforward_(feedforward) {}

bool Arm::feedbackReady() {
  return joint0_.connect_.check() && joint1_.connect_.check() &&
         joint0_.mode_ != Motor::INIT && joint1_.mode_ != Motor::INIT;
}

bool Arm::withinLimits(const float q[2]) const {
  return q[0] >= limits_.q_min[0] && q[0] <= limits_.q_max[0] &&
       q[1] >= limits_.q_min[1] && q[1] <= limits_.q_max[1];
}

bool Arm::calibrate(float q0_rad, float q1_rad) {
  const float q[2] = {q0_rad, q1_rad};
  if (mode_ != Mode::Disabled || !withinLimits(q)) {
    return false;
  }
  joint0_.resetFeedbackAngle(math::rad2deg(q0_rad));
  joint1_.resetFeedbackAngle(math::rad2deg(q1_rad));
  calibrated_ = true;

  trajAbort();
  updateFeedback();
  reference_ =Reference{};
  reference_.q[0] = state_.q[0];
  reference_.q[1] = state_.q[1];
  return true;
}

bool Arm::enable() {
  if (!calibrated_||!feedbackReady()) return false;
  if (mode_ == Mode::Joint) return true;

  updateFeedback();
  reference_ = Reference{};
  reference_.q[0] = state_.q[0];
  reference_.q[1] = state_.q[1];
  mode_ = Mode::Joint;
  return true;
}

void Arm::disable() {
  trajAbort();
  mode_ = Mode::Disabled;
  if (joint0_.mode_ != Motor::INIT) {
    joint0_.mode_ = Motor::POWEROFF;
  }
  if (joint1_.mode_ != Motor::INIT) {
    joint1_.mode_ = Motor::POWEROFF;
  }

  joint0_.intensity_ = 0.0f;
  joint0_.intensity_float_ = 0.0f;
  joint1_.intensity_ = 0.0f;
  joint1_.intensity_float_ = 0.0f;
}

bool Arm::setReference(const Reference& reference) {
  if (trajRunning()) return false;
  return applyReference(reference);
}

bool Arm::applyReference(const Reference& reference) {
  if (!withinLimits(reference.q)) {
    return false;
  }
  reference_ = reference;
  return true;
}

bool Arm::addJointReference(float speed0, float speed1, float dt_s) {
  if (trajRunning()||mode_!=Mode::Joint) return false;

  Reference next{};
  next.q[0] = reference_.q[0] + speed0 * dt_s;
  next.q[1] = reference_.q[1] + speed1 * dt_s;

  const float speed[2] ={speed0,speed1};
  for (int i=0;i<2;i++) {
    next.q[i]=math::limit(next.q[i],limits_.q_min[i],limits_.q_max[i]);
    next.dq[i]=speed[i];
    if ((speed[i] > 0.f && next.q[i] >= limits_.q_max[i]) ||
        (speed[i] < 0.f && next.q[i] <= limits_.q_min[i])) {
      next.dq[i] = 0.f;
        }
    next.ddq[i]= 0.0f;
  }

  if (!applyReference(next)) {
    return false;
  }
  return true;
}

bool Arm::trajectoryReady() {
  return mode_ ==Mode::Joint && calibrated_ && feedbackReady() && !trajRunning();
}

bool Arm::trajStart(float x_m, float y_m, float duration_s) {
  if (!trajectoryReady() || duration_s <= 0.0f ||!withinLimits(reference_.q)) {
    return false;
  }

  CartesianReference start = ArmKinematics::forward(reference_.q);
  CartesianReference end{};
  end.x = x_m;
  end.y = y_m;

  const float sin_q1 = sin(reference_.q[1]);
  if (fabs(sin_q1)< ArmKinematics::min_sin_q1) {
    return false;
  }
  const int elbow = sin_q1 > 0.0f? 1:-1;

  if (!trajectory_.start(start,end,duration_s)) {
    return false;
  }

  joint_trajectory_duration_ = duration_s;
  trajectory_start_tick_ = HAL_GetTick();
  trajectory_type_ = TrajectoryType::Cartesian;
  trajectory_finished_ = false;
  return true;
}

bool Arm::trajStartJoint(float q0_rad, float q1_rad, float duration_s) {
  const float end[2] ={q0_rad, q1_rad};
  if (!trajectoryReady()||!withinLimits(end)||duration_s<0.0f) {
    return false;
  }

  updateFeedback();
  if (!withinLimits(state_.q)) {
    return false;
  }
  joint_trajectory_start_[0] = state_.q[0];
  joint_trajectory_start_[1] = state_.q[1];

  joint_trajectory_end_[0] = q0_rad;
  joint_trajectory_end_[1] = q1_rad;

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

  reference_.dq[0] = 0.0f;
  reference_.dq[1] = 0.0f;
  reference_.ddq[0] = 0.0f;
  reference_.ddq[1] = 0.0f;
}

void Arm::trajectoryPlanner() {
  const float elapsed_time = (HAL_GetTick()-trajectory_start_tick_)*0.001f;
  if (trajectory_type_ == TrajectoryType::None) {
    return;
  }else if (trajectory_type_ == TrajectoryType::Joint) {
    Reference next{};
    ArmTrajectory::interpolate(joint_trajectory_start_[0],joint_trajectory_end_[0],elapsed_time,
      joint_trajectory_end_[0],next.q[0],next.dq[0],next.ddq[0]);
    ArmTrajectory::interpolate(joint_trajectory_start_[1],joint_trajectory_end_[1],elapsed_time,
      joint_trajectory_end_[1],next.q[1],next.dq[1],next.ddq[1]);
    if (!applyReference(next)) {
      trajAbort();
    }
    return;
  }else if (trajectory_type_ == TrajectoryType::Cartesian) {
    CartesianReference sample_point = trajectory_.sample(elapsed_time);
    Reference next{};
    if (!ArmKinematics::inverse(sample_point,reference_.q,trajectory_elbow_,next.q)) {
      trajAbort();
      return;
    }

    if (!ArmKinematics::derivatives(sample_point,next.q,next.dq,next.ddq)) {
      trajAbort();
      return;
    }

    if (!applyReference(next)) {
      trajAbort();
      return;
    }

    if (!trajectory_.running()) {
      trajectory_type_=TrajectoryType::None;
      trajectory_finished_ = true;
    }
  }
}

void Arm::updateFeedback() {
  float q[2];
  float dq[2];

  q[0] = joint0_.realAngle();
  q[1] = joint1_.realAngle();
  dq[0] = joint0_.realSpeed();
  dq[1] = joint1_.realSpeed();

  state_.q[0] = math::deg2rad(q[0]);
  state_.q[1] = math::deg2rad(q[1]);
  state_.dq[0] = math::deg2rad(dq[0]);
  state_.dq[1] = math::deg2rad(dq[1]);
}

void Arm::handle() {
  if (!feedbackReady()) {
    disable();
    return;
  }

  updateFeedback();
  if (mode_ ==Mode::Disabled) {
    return;
  }

  if (!calibrated_) {
    disable();
    return;
  }

  trajectoryPlanner();
  writeMotorCommands();
}

void Arm::writeMotorCommands() {
  if (mode_!=Mode::Joint) {
    return;
  }

  joint0_.method_ =Motor::POSITION_SPEED;
  joint1_.method_ =Motor::POSITION_SPEED;
  joint0_.setAngleSpeed( math::rad2deg(reference_.q[0]),math::rad2deg(reference_.dq[0]),
    feedforward_.k_acc[0]*reference_.ddq[0]);
  joint1_.setAngleSpeed(math::rad2deg(reference_.q[1]),math::rad2deg(reference_.dq[1]),
    feedforward_.k_acc[1]*reference_.ddq[1]);

  joint0_.mode_ =Motor::WORKING;
  joint1_.mode_ =Motor::WORKING;
}
