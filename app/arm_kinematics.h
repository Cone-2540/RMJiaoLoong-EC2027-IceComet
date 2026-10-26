#ifndef DOUBLE_JOINTED_ROBOTIC_ARM_KINEMATICS_H
#define DOUBLE_JOINTED_ROBOTIC_ARM_KINEMATICS_H

struct CartesianReference {
  float x{}, y{};      // m
  float dx{}, dy{};    // m/s
  float ddx{}, ddy{};  // m/s²
};

// 同向伸直为零位，q1 相对第一杆
class ArmKinematics {
 public:
  static constexpr float l0 = 0.045f;
  static constexpr float l1 = 0.115f;
  static constexpr float min_sin_q1 = 0.01f;  // 雅可比奇异阈值

  static CartesianReference forward(const float q[2]);
  static bool lineReachable(const CartesianReference& start,
                            const CartesianReference& end);
  // elbow 为 +1/-1；q 展开到 previous_q 附近的圈数
  static bool inverse(const CartesianReference& point,
                      const float previous_q[2], int elbow, float q[2]);
  static bool derivatives(const CartesianReference& point, const float q[2],
                          float dq[2], float ddq[2]);
};

#endif  // DOUBLE_JOINTED_ROBOTIC_ARM_KINEMATICS_H
