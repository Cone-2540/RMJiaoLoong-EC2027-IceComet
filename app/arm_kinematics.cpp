#include "arm_kinematics.h"

CartesianReference ArmKinematics::forward(const float[2]) {
  return CartesianReference{};
}

bool ArmKinematics::inverse(const CartesianReference&, const float[2],
                           int, float[2]) {
  return false;
}

bool ArmKinematics::lineReachable(const CartesianReference&,
                                const CartesianReference&) {
  return false;
}

bool ArmKinematics::derivatives(const CartesianReference&, const float[2],
                               float[2], float[2]) {
  return false;
}
