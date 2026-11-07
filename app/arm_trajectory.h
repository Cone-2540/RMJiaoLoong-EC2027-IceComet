#ifndef DOUBLE_JOINTED_ROBOTIC_ARM_TRAJECTORY_H
#define DOUBLE_JOINTED_ROBOTIC_ARM_TRAJECTORY_H

#include "arm_kinematics.h"

// 单段直线、五次时间插值，两端速度和加速度为零
// 同时计算解析速度、加速度，供前馈和观察使用
class ArmTrajectory {
 public:
  bool start(const CartesianReference& start, const CartesianReference& end,
             float duration_s);
  CartesianReference sample(float elapsed_s);
  // 两端静止的单轴插值，位置或关节角通用
  static void interpolate(float start, float end, float elapsed_s,
                          float duration_s, float& position, float& velocity,
                          float& acceleration);
  void abort();
  bool running() const { return running_; }

 private:
  CartesianReference start_{}, end_{}, last_{};
  float duration_s_{};
  bool running_{false};
};

#endif  // DOUBLE_JOINTED_ROBOTIC_ARM_TRAJECTORY_H
