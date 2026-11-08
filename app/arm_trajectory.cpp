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
  circle_ = false;
  path_mode_ = false;
  rounded_mode_ = false;
  return true;
}

bool ArmTrajectory::startCircle(const CartesianReference& start, float duration_s) {
  // 以原点为圆心时整圈半径不变，起点可达即整圈几何可达。
  if (!this->start(start, start, duration_s)) return false;
  circle_ = true;
  return true;
}

CartesianReference ArmTrajectory::sample(float elapsed_s) {
  if (!running_ || !std::isfinite(elapsed_s)) return last_;
  if (rounded_mode_) {
    last_ = roundedSquareSample(elapsed_s);
  } else if (path_mode_) {
    last_ = pathSample(elapsed_s);
    if (elapsed_s >= path_duration_) running_ = false;
    return last_;
  } else if (circle_) {
    float angle, speed, acceleration;
    interpolate(0.f, -6.28318530718f, elapsed_s, duration_s_, angle, speed, acceleration);
    const float c = cosf(angle), s = sinf(angle);
    last_.x = start_.x * c - start_.y * s;
    last_.y = start_.x * s + start_.y * c;
    last_.dx = -last_.y * speed; last_.dy = last_.x * speed;
    last_.ddx = -last_.x * speed * speed - last_.y * acceleration;
    last_.ddy = -last_.y * speed * speed + last_.x * acceleration;
    if (elapsed_s >= duration_s_) {
      last_.x = start_.x; last_.y = start_.y;
    }
  } else {
    interpolate(start_.x, end_.x, elapsed_s, duration_s_, last_.x, last_.dx, last_.ddx);
    interpolate(start_.y, end_.y, elapsed_s, duration_s_, last_.y, last_.dy, last_.ddy);
  }
  if (elapsed_s >= duration_s_) running_ = false;
  return last_;
}

bool ArmTrajectory::configurePath(const Point* points, uint32_t count, float speed) {
  if (running_) return false;
  path_count_ = 0;
  if (!points || count < 2 || count > max_path_points ||
      !std::isfinite(speed) || speed <= 0.f) return false;
  path_time_[0] = 0.f;
  for (uint32_t i = 0; i < count; ++i) {
    if (!std::isfinite(points[i].x) || !std::isfinite(points[i].y)) return false;
    path_[i] = points[i]; tangent_[i] = {0.f, 0.f};
    if (i) {
      const float length = hypotf(points[i].x - points[i-1].x, points[i].y - points[i-1].y);
      const float duration = length / speed;
      if (length < 1e-6f || !std::isfinite(duration) || duration < .001f) return false;
      path_time_[i] = path_time_[i-1] + duration;
      if (!std::isfinite(path_time_[i]) || path_time_[i] > 2147483.f ||
          path_time_[i] <= path_time_[i-1]) return false;
    }
  }
  for (uint32_t i = 1; i + 1 < count; ++i) {
    const float a = hypotf(path_[i].x-path_[i-1].x, path_[i].y-path_[i-1].y);
    const float b = hypotf(path_[i+1].x-path_[i].x, path_[i+1].y-path_[i].y);
    // 相邻单位方向取平均，直线不停顿，急转弯自动降低节点速度。
    tangent_[i].x = .5f * speed * ((path_[i].x-path_[i-1].x)/a + (path_[i+1].x-path_[i].x)/b);
    tangent_[i].y = .5f * speed * ((path_[i].y-path_[i-1].y)/a + (path_[i+1].y-path_[i].y)/b);
  }
  path_count_ = count;
  path_duration_ = path_time_[count-1];
  return true;
}

bool ArmTrajectory::startPath() {
  if (running_ || path_count_ < 2) return false;
  path_mode_ = true; circle_ = false; rounded_mode_ = false; running_ = true;
  duration_s_ = path_duration_;
  last_ = pathSample(0.f);
  return true;
}

bool ArmTrajectory::configureRoundedSquare(float side, float radius, float duration) {
  if (running_) return false;
  rounded_duration_ = 0.f;
  if (!std::isfinite(side) || side <= 0.f || !std::isfinite(radius) ||
      radius <= 0.f || radius > side * .5f || !std::isfinite(duration) ||
      duration <= 0.f || duration > 2147483.f) return false;
  // 整圈离原点最近为半边长，最远为圆角中点；解析排除工作空间外的路径。
  const float half = side * .5f, far = 1.41421356237f * (half-radius) + radius;
  CartesianReference inner{}, outer{};
  inner.x = half; outer.x = far;
  if (!ArmKinematics::lineReachable(inner, outer)) return false;
  rounded_half_ = half; rounded_radius_ = radius;
  rounded_length_ = 4.f * (side-2.f*radius) + 6.28318530718f*radius;
  rounded_duration_ = duration;
  return true;
}

bool ArmTrajectory::startRoundedSquare() {
  if (running_ || rounded_duration_ <= 0.f) return false;
  rounded_mode_ = true; path_mode_ = circle_ = false;
  duration_s_ = rounded_duration_;
  last_ = roundedSquareSample(0.f);
  running_ = true;
  return true;
}

CartesianReference ArmTrajectory::roundedSquareSample(float t) const {
  CartesianReference result{};
  if (rounded_duration_ <= 0.f || !std::isfinite(t)) return result;
  const float h = rounded_half_, r = rounded_radius_, c = h-r;
  result.x = h; result.y = c;
  if (t <= 0.f || t >= rounded_duration_) return result;
  float s, speed, acceleration;
  // 整圈使用同一五次时间律，内部切点不停顿；起终点速度、加速度为零。
  interpolate(0.f, rounded_length_, t, rounded_duration_, s, speed, acceleration);
  const float straight = 2.f*c, quarter = 1.57079632679f*r;
  const float sector_length = straight+quarter;
  int sector = static_cast<int>(s/sector_length);
  if (sector > 3) sector = 3;
  const float local = s-sector*sector_length;
  float x,y,tx,ty,nx=0.f,ny=0.f;
  if (local < straight) {
    x=h; y=c-local; tx=0.f; ty=-1.f;
  } else {
    const float angle = (local-straight)/r;
    const float co = cosf(angle), si = sinf(angle);
    x=c+r*co; y=-c-r*si; tx=-si; ty=-co;
    // 圆弧加速度 = 切向加速度 + 指向圆心的 v²/r。
    nx=-co*speed*speed/r; ny=si*speed*speed/r;
  }
  result.x=x; result.y=y;
  result.dx=tx*speed; result.dy=ty*speed;
  result.ddx=tx*acceleration+nx; result.ddy=ty*acceleration+ny;
  // 其余三组“直线+1/4 圆”由第一组顺时针旋转得到。
  for (int i=0; i<sector; ++i) {
    const float px=result.x, vx=result.dx, ax=result.ddx;
    result.x=result.y; result.y=-px;
    result.dx=result.dy; result.dy=-vx;
    result.ddx=result.ddy; result.ddy=-ax;
  }
  return result;
}

CartesianReference ArmTrajectory::pathSample(float t) const {
  CartesianReference result{};
  if (path_count_ < 2 || !std::isfinite(t)) return result;
  if (t <= 0.f) { result.x=path_[0].x; result.y=path_[0].y; return result; }
  if (t >= path_duration_) {
    result.x=path_[path_count_-1].x; result.y=path_[path_count_-1].y; return result;
  }
  uint32_t i = 0;
  while (i+2 < path_count_ && t > path_time_[i+1]) ++i;
  const float duration = path_time_[i+1] - path_time_[i];
  const float u = (t-path_time_[i])/duration;
  for (int axis = 0; axis < 2; ++axis) {
    const float p0 = axis ? path_[i].y : path_[i].x;
    const float p1 = axis ? path_[i+1].y : path_[i+1].x;
    const float v0 = duration * (axis ? tangent_[i].y : tangent_[i].x);
    const float v1 = duration * (axis ? tangent_[i+1].y : tangent_[i+1].x);
    const float d = p1-p0;
    // 五次 Hermite：节点位置、速度连续，节点两侧加速度均为零。
    const float a3=10*d-6*v0-4*v1, a4=-15*d+8*v0+7*v1, a5=6*d-3*v0-3*v1;
    const float p=p0+u*(v0+u*u*(a3+u*(a4+u*a5)));
    const float v=(v0+u*u*(3*a3+u*(4*a4+5*u*a5)))/duration;
    const float a=u*(6*a3+u*(12*a4+20*u*a5))/(duration*duration);
    if (axis) { result.y=p; result.dy=v; result.ddy=a; }
    else { result.x=p; result.dx=v; result.ddx=a; }
  }
  return result;
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
