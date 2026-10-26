#include "arm_trajectory.h"

bool ArmTrajectory::start(const CartesianReference&,
                         const CartesianReference&, float) {
  return false;
}

CartesianReference ArmTrajectory::sample(float) {
  return CartesianReference{};
}

void ArmTrajectory::interpolate(float, float, float, float,
                               float& position, float& velocity,
                               float& acceleration) {
  position = velocity = acceleration = 0.f;
}

void ArmTrajectory::abort() {
}
