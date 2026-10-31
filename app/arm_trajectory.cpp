#include "arm_trajectory.h"

#include <bits/ios_base.h>

#include <valarray>

#include "arm_kinematics.h"

bool ArmTrajectory::start(const CartesianReference& start,
                         const CartesianReference& end, float duration_s) {
  if (running_ || duration_s <= 0.0f) return false;
  if (!ArmKinematics::lineReachable(start,end)) return false;

  start_ ={};
  start_.x =start.x;
  start_.y =start.y;

  end_={};
  end_.x =end.x;
  end_.y =end.y;

  duration_s_ = duration_s;
  last_ =start_;
  running_ = true;
  return true;
}

CartesianReference ArmTrajectory::sample(float elapsed_s) {
  if (!running_) {
    return last_;
  }

  if (elapsed_s >= duration_s_) {
    last_ = end_;
    running_ = false;
    return last_;
  }

  float s, ds, dds;
  interpolate(0.0f,1.0f,elapsed_s,duration_s_,s, ds, dds);

  float delta_x = end_.x - start_.x;
  float delta_y = end_.y - start_.y;

  last_.x = start_.x + delta_x * s;
  last_.y = start_.y + delta_y * s;
  last_.dx = delta_x * ds;
  last_.dy = delta_y * ds;
  last_.ddx = delta_x * dds;
  last_.ddy = delta_y * dds;

  return last_;
}

void ArmTrajectory::interpolate(float start, float end, float elapsed_s, float duration_s,
                               float& position, float& velocity,
                               float& acceleration) {
  position =start;
  velocity = acceleration = 0.0f;

  if (elapsed_s >= duration_s) {
    position = end;
    return;
  }

  const float inv = 1.0f / duration_s;
  const float u = elapsed_s * inv;
  const float u2 = u * u;
  const float u3 = u2 * u;
  const float delta = end - start;

  const float s = u3*(10.0f + u*(-15.0f + 6.0f*u));
  const float ds = 30.0f *u2 *(1 - u) *(1 - u) *inv;
  const float dds = 60.0f * (1 - u) * u * (1.0f - 2.0f * u) * inv *inv;

  position = start + delta * s;
  velocity = delta * ds;
  acceleration = delta * dds;
}

void ArmTrajectory::abort() {
  running_ = false;

  last_.dx = 0.0f;
  last_.dy = 0.0f;
  last_.ddx = 0.0f;
  last_.ddy = 0.0f;
}
