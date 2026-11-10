#ifndef DOUBLE_JOINTED_ROBOTIC_ARM_TRAJECTORY_H
#define DOUBLE_JOINTED_ROBOTIC_ARM_TRAJECTORY_H

#include "arm_kinematics.h"
#include <cstdint>

// 单段直线、整圆、圆角方形或多点平滑曲线；起终点速度和加速度为零
// 同时计算解析速度、加速度，供前馈和观察使用
class ArmTrajectory {
 public:
  static constexpr uint32_t max_path_points = 128;
  struct Point { float x, y; }; // m
  bool start(const CartesianReference& start, const CartesianReference& end,
             float duration_s);
  // 绕基座原点顺时针一圈，半径为起点到原点的距离。
  bool startCircle(const CartesianReference& start, float duration_s);
  // 向内倒圆角，外形仍为 side × side；从右上角的右侧切点顺时针闭合一圈。
  bool configureRoundedSquare(float side, float radius, float duration_s);
  bool startRoundedSquare();
  CartesianReference roundedSquareSample(float elapsed_s) const;
  float roundedSquareDuration() const { return rounded_duration_; }
  float roundedSquareLength() const { return rounded_length_; }
  // 多点连续轨迹：段用时=弦长/平均线速度，内部节点共享切向速度。
  bool configurePath(const Point* points, uint32_t count, float speed_mps);
  bool startPath();
  CartesianReference pathSample(float elapsed_s) const;
  float pathDuration() const { return path_duration_; }
  const Point& pathStart() const { return path_[0]; }
  uint32_t pathCount() const { return path_count_; }
  const Point& pathPoint(uint32_t i) const { return path_[i]; }
  float pathTime(uint32_t i) const { return path_time_[i]; }
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
  bool circle_{false};
  bool path_mode_{false};
  bool rounded_mode_{false};
  float rounded_half_{}, rounded_radius_{}, rounded_duration_{}, rounded_length_{};
  Point path_[max_path_points]{}, tangent_[max_path_points]{};
  float path_time_[max_path_points]{};
  uint32_t path_count_{};
  float path_duration_{};
};

#endif  // DOUBLE_JOINTED_ROBOTIC_ARM_TRAJECTORY_H
