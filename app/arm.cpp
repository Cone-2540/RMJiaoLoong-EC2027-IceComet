//
// Created by NoSignal on 2026/10/16.
//

#include "arm.h"

Arm::Arm(Motor& joint0, Motor& joint1, const Limits& limits,
         const Feedforward& feedforward)
    : joint0_(joint0), joint1_(joint1), limits_(limits),
      feedforward_(feedforward) {}

bool Arm::feedbackReady() {
  return false;
}

bool Arm::withinLimits(const float[2]) const {
  return false;
}

bool Arm::calibrate(float, float) {
  return false;
}

bool Arm::enable() {
  return false;
}

void Arm::disable() {
}

bool Arm::setReference(const Reference&) {
  return false;
}

bool Arm::applyReference(const Reference&) {
  return false;
}

bool Arm::addJointReference(float, float, float) {
  return false;
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
}

void Arm::trajectoryPlanner() {
}

void Arm::updateFeedback() {
}

void Arm::handle() {
}

void Arm::writeMotorCommands() {
}
