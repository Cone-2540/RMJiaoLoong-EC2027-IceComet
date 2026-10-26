#ifndef DOUBLE_JOINTED_ROBOTIC_ARM_TRAJECTORY_H
#define DOUBLE_JOINTED_ROBOTIC_ARM_TRAJECTORY_H

#include "arm_kinematics.h"

// 单段直线、五次时间插值，两端速度和加速度为零
// 必做完成位置输出，解析速度与加速度列为进阶任务
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
  CartesianReference start_{}, end_{};
  float duration_s_{};
  bool running_{false};
};

#endif  // DOUBLE_JOINTED_ROBOTIC_ARM_TRAJECTORY_H
