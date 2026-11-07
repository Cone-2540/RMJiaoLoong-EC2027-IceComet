#include "arm_kinematics.h"
#include <cmath>

namespace {
constexpr float two_pi = 6.28318530718f;
constexpr float radius_tolerance = 1e-7f;  // m，容纳浮点舍入误差

bool finitePosition(const CartesianReference& point) {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

bool radiusReachable(float radius_squared) {
  const float inner = fabsf(ArmKinematics::l1 - ArmKinematics::l0) - radius_tolerance;
  const float outer = ArmKinematics::l0 + ArmKinematics::l1 + radius_tolerance;
  return radius_squared >= inner * inner && radius_squared <= outer * outer;
}

float unwrap(float angle, float previous) {
  return angle + two_pi * roundf((previous - angle) / two_pi);
}
}  // namespace

CartesianReference ArmKinematics::forward(const float q[2]) {
  CartesianReference point{};
  point.x = l0 * cosf(q[0]) + l1 * cosf(q[0] + q[1]);
  point.y = l0 * sinf(q[0]) + l1 * sinf(q[0] + q[1]);
  return point;
}

bool ArmKinematics::inverse(const CartesianReference& point,
                           const float previous_q[2], int elbow, float q[2]) {
  if (!finitePosition(point) || !std::isfinite(previous_q[0]) ||
      !std::isfinite(previous_q[1]) || (elbow != 1 && elbow != -1)) return false;
  const float r2 = point.x * point.x + point.y * point.y;
  if (!radiusReachable(r2)) return false;

  float cos_q1 = (r2 - l0 * l0 - l1 * l1) / (2.f * l0 * l1);
  if (cos_q1 > 1.f) cos_q1 = 1.f;
  if (cos_q1 < -1.f) cos_q1 = -1.f;
  const float q1 = elbow * acosf(cos_q1);
  const float q0 = atan2f(point.y, point.x) -
                   atan2f(l1 * sinf(q1), l0 + l1 * cosf(q1));
  // atan2 只给出一圈内的角度，补回最接近上一参考的累计圈数。
  q[0] = unwrap(q0, previous_q[0]);
  q[1] = unwrap(q1, previous_q[1]);
  return true;
}

bool ArmKinematics::lineReachable(const CartesianReference& start,
                                const CartesianReference& end) {
  if (!finitePosition(start) || !finitePosition(end)) return false;
  const float start_r2 = start.x * start.x + start.y * start.y;
  const float end_r2 = end.x * end.x + end.y * end.y;
  if (!radiusReachable(start_r2) || !radiusReachable(end_r2)) return false;

  const float dx = end.x - start.x, dy = end.y - start.y;
  const float length2 = dx * dx + dy * dy;
  if (length2 == 0.f) return true;
  // 线段到原点的最近点：两端可达，还要排除中间穿过内侧盲区。
  float u = -(start.x * dx + start.y * dy) / length2;
  if (u < 0.f) u = 0.f;
  if (u > 1.f) u = 1.f;
  const float x = start.x + u * dx, y = start.y + u * dy;
  return radiusReachable(x * x + y * y);
}

bool ArmKinematics::derivatives(const CartesianReference& point, const float q[2],
                               float dq[2], float ddq[2]) {
  if (!finitePosition(point) || !std::isfinite(point.dx) || !std::isfinite(point.dy) ||
      !std::isfinite(point.ddx) || !std::isfinite(point.ddy) ||
      !std::isfinite(q[0]) || !std::isfinite(q[1]) || fabsf(sinf(q[1])) < min_sin_q1)
    return false;
  const float s0 = sinf(q[0]), c0 = cosf(q[0]);
  const float s01 = sinf(q[0] + q[1]), c01 = cosf(q[0] + q[1]);
  const float j00 = -l0 * s0 - l1 * s01, j01 = -l1 * s01;
  const float j10 = l0 * c0 + l1 * c01, j11 = l1 * c01;
  const float det = l0 * l1 * sinf(q[1]);
  const float v0 = (j11 * point.dx - j01 * point.dy) / det;
  const float v1 = (-j10 * point.dx + j00 * point.dy) / det;
  // J * ddq = ddp - J_dot * dq。
  const float v01 = v0 + v1;
  const float ax = point.ddx + l0 * c0 * v0 * v0 + l1 * c01 * v01 * v01;
  const float ay = point.ddy + l0 * s0 * v0 * v0 + l1 * s01 * v01 * v01;
  const float a0 = (j11 * ax - j01 * ay) / det;
  const float a1 = (-j10 * ax + j00 * ay) / det;
  if (!std::isfinite(v0) || !std::isfinite(v1) || !std::isfinite(a0) || !std::isfinite(a1))
    return false;
  dq[0] = v0; dq[1] = v1;
  ddq[0] = a0; ddq[1] = a1;
  return true;
}
