/******************************************************************************
 * WholeBodyTrajectoryCost.h
 *
 * 全身轨迹跟踪 StateCost:
 *
 *   L(x, t) = 0.5 * (x - x_d(t))^T Q (x - x_d(t))
 *
 *   x_d(t) 直接来自 OCS2 传进来的 TargetTrajectories (即 ReferenceManager 里
 *   由 ROS topic <robot_name>_mpc_target 更新的那一份)。
 *
 * 关键设计:
 *   1. 不持有 ReferenceManager 指针。StateCost 接口本身就把 TargetTrajectories
 *      作为形参传进来, 由 solver 在 preSolverRun() 之后统一切换, 天然线程安全。
 *   2. stateTrajectory 的每个元素必须是 stateDim 维 (例如 9 = [x, y, yaw, q1..q6])。
 *      若维度不匹配 (例如仍然是旧的 7 维 EE pose), 本 cost 自动退化为 0,
 *      而不是抛异常把 MPC 线程打死。
 *   3. yaw 分量: 插值走最短角路径, 误差做 wrap 到 [-pi, pi]。
 *****************************************************************************/

#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/cost/StateCost.h>
#include <ocs2_core/reference/TargetTrajectories.h>

namespace ocs2 {
namespace mobile_manipulator {

class WholeBodyTrajectoryCost final : public StateCost {
 public:
  /**
   * @param Q             stateDim x stateDim 的权重矩阵
   * @param yawIndex      底盘 yaw 在 state 中的下标; 无 yaw 时传 -1
   * @param fallbackState 当 TargetTrajectories 为空时使用的参考 (一般是 initialState)
   */
  WholeBodyTrajectoryCost(matrix_t Q, int yawIndex, vector_t fallbackState);

  ~WholeBodyTrajectoryCost() override = default;

  WholeBodyTrajectoryCost* clone() const override;

  scalar_t getValue(scalar_t time, const vector_t& state,
                    const TargetTrajectories& targetTrajectories,
                    const PreComputation& preComputation) const override;

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(
      scalar_t time, const vector_t& state,
      const TargetTrajectories& targetTrajectories,
      const PreComputation& preComputation) const override;

 private:
  WholeBodyTrajectoryCost(const WholeBodyTrajectoryCost& other) = default;

  /** 把角度 wrap 到 [-pi, pi] */
  static scalar_t wrapToPi(scalar_t angle);

  /**
   * 在 targetTrajectories 中按时间插值出期望全身状态。
   * 越界时钳位到首 / 末航点。
   * 若 target 维度非法, 返回一个 size()==0 的空向量, 调用方据此关闭 cost。
   */
  vector_t getDesiredState(scalar_t time,
                           const TargetTrajectories& targetTrajectories) const;

  matrix_t Q_;
  int      yawIndex_{-1};
  vector_t fallbackState_;
};

}  // namespace mobile_manipulator
}  // namespace ocs2
