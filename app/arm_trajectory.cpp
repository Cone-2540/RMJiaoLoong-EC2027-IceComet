#include "arm_trajectory.h"
#include <cmath>

bool ArmTrajectory::start(const CartesianReference& start,
                         const CartesianReference& end, float duration_s) {
  if (!std::isfinite(duration_s) || duration_s <= 0.f ||
      !ArmKinematics::lineReachable(start, end)) return false;
  start_ = start;
  end_ = end;
  duration_s_ = duration_s;
  last_ = CartesianReference{};
  last_.x = start.x; last_.y = start.y;
  running_ = true;
  return true;
}

CartesianReference ArmTrajectory::sample(float elapsed_s) {
  if (!running_ || !std::isfinite(elapsed_s)) return last_;
  interpolate(start_.x, end_.x, elapsed_s, duration_s_, last_.x, last_.dx, last_.ddx);
  interpolate(start_.y, end_.y, elapsed_s, duration_s_, last_.y, last_.dy, last_.ddy);
  if (elapsed_s >= duration_s_) running_ = false;
  return last_;
}

void ArmTrajectory::interpolate(float start, float end, float elapsed_s, float duration_s,
                               float& position, float& velocity,
                               float& acceleration) {
  position = start;
  velocity = acceleration = 0.f;
  if (!std::isfinite(start) || !std::isfinite(end) || !std::isfinite(elapsed_s) ||
      !std::isfinite(duration_s) || duration_s <= 0.f || elapsed_s <= 0.f) return;
  if (elapsed_s >= duration_s) {
    position = end;
    return;
  }
  const float u = elapsed_s / duration_s;
  const float u2 = u * u, u3 = u2 * u;
  // s = 10u³ - 15u⁴ + 6u⁵，两端的一阶、二阶导数均为零。
  const float s = u3 * (10.f + u * (-15.f + 6.f * u));
  const float ds = 30.f * u2 * (1.f - u) * (1.f - u) / duration_s;
  const float dds = 60.f * u * (1.f - u) * (1.f - 2.f * u) / (duration_s * duration_s);
  position = start + (end - start) * s;
  velocity = (end - start) * ds;
  acceleration = (end - start) * dds;
}

void ArmTrajectory::abort() {
  running_ = false;
  last_.dx = last_.dy = last_.ddx = last_.ddy = 0.f;
}
