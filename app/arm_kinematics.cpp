#include "arm_kinematics.h"
#include <cmath>
namespace {
constexpr float PI = 3.14159265358979323846;
constexpr float kTolerance = 0.001;

  float elb_cos(float radius) {
    float c = (radius * radius -ArmKinematics::l0*ArmKinematics::l0 -ArmKinematics::l1*
      ArmKinematics::l1)/(2*ArmKinematics::l0*ArmKinematics::l1);
    return c;
  }

  bool reverseKinematics(float radius) {
    const float c = elb_cos(radius);
    if (c > 1.0f) return false;
    if ((1.0f - c*c)< ArmKinematics::min_sin_q1*ArmKinematics::min_sin_q1) return false;;
  }

  float unWrapNear(const float angle,const float last_angle) {
    return last_angle + std::remainder(angle-last_angle,2*PI);
  }
}

CartesianReference ArmKinematics::forward(const float q[2]) {
  CartesianReference result;
  result.x = l0 * cos(q[0]) + l1 * cos(q[1]+q[0]);
  result.y = l0 * sin(q[0]) + l1 * sin(q[1]+q[0]);
  return result;
}

bool ArmKinematics::inverse(const CartesianReference& point,
                      const float previous_q[2], int elbow, float q[2]) {
  float c1 = (point.x*point.x + point.y*point.y -l0*l0
    - l1*l1)/(2*l0*l1);
  if (c1> 1.0f + kTolerance || c1< -1.0f - kTolerance) return false;
  if (c1 > 1.0f) c1=1.0f;
  if (c1< -1.0f) c1=-1.0f;

  float s1 =static_cast<float>(elbow)*sqrt(1 - c1*c1);
  const float raw_q1 =std::atan2(s1,c1);
  const float raw_q0 =std::atan2(point.x,point.y) -
    std::atan2(l1*s1, l0+l1*c1);

  const float next_q0 =unWrapNear(raw_q0,previous_q[0]);
  const float next_q1 =unWrapNear(raw_q1,previous_q[1]);

  if (!std::isfinite(next_q0)||!std::isfinite(next_q1)) return false;
  q[0]=next_q0;
  q[1]=next_q1;

  return true;
}

bool ArmKinematics::lineReachable(const CartesianReference& start,
                                const CartesianReference& end) {
  const float dx = end.x - start.x;
  const float dy = end.y - start.y;
  const float length_sq= dx*dx + dy*dy;

  const float a =l0;
  const float b =l1;
  const float min_s =min_sin_q1;
  const float max_c =sqrt(1.0f -min_s*min_s);

  const float r_min = a*a + b*b -2.0f * a*b* max_c;
  const float r_max = a*a + b*b +2.0f * a*b* max_c;

  const float start_r = sqrt(start.x*start.x + start.y*start.y);
  const float end_r = sqrt(end.x*end.x + end.y*end.y);

  if (start_r > r_max || end_r < r_min) return false;
  float t = (start.x*end.x + start.y*end.y) /length_sq;
  if (t <0.0f) t=0.0f;
  if (t >1.0f) t=1.0f;

  const float nx =start.x + t*dx;
  const float ny =start.y + t*dy;
  const float nearest_r_sq = nx*nx + ny*ny;
  if (nearest_r_sq < r_min * r_min) return false;
}

bool ArmKinematics::derivatives(const CartesianReference& point,
                                const float q[2], float dq[2], float ddq[2]) {
  float s1 = sin(q[1]);
  if (fabs(s1)<min_sin_q1) return false;

  float s01 = sin(q[1]+q[0]);
  float c01 = cos(q[1]+q[0]);
  float s0 = sin(q[0]);
  float c0 = cos(q[0]);
  const float j00 = -l0*s0 -l1*s01;
  const float j01 = -l1*s01;
  const float j10 = l0*c0 + l1*c01;
  const float j11 = l1*c01;
  const float det = l0*l1*s1;

  dq[0]=(j11*point.dx - j01*point.dy)/det;
  dq[1]=(-j00*point.dy + j01*point.dx)/det;
  const float w=dq[0] + dq[1];
  const float ax =point.ddx + l0* c0* dq[0]*dq[0] +
    l1 *c01 *w *w;
  const float ay =point.ddy + l0* s0* dq[0]*dq[0] +
    l1 *s01 *w *w;

  const float ddq0 = (j11 * ax - j01 * ay) / det;
  const float ddq1 = (-j10 * ax + j00 * ay) / det;
  ddq[0]=ddq0;
  ddq[1]=ddq1;

  return true;
}
