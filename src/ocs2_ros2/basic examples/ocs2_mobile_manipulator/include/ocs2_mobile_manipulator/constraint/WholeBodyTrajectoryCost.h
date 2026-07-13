/******************************************************************************
 * WholeBodyTrajectoryCost.h
 *
 * 全身轨迹跟踪软约束 (以二次型 StateCost 实现)。
 *
 * 内部持有一条时间参数化的全身参考轨迹 x_d(t):
 *   x_d(t) = [x, y, yaw, q_1 ... q_n]^T   (WheelBased: 3 + armDim 维)
 *
 * 代价:
 *   L(x, t) = 0.5 * e^T Q e,   e = x - x_d(t)  (yaw 分量做角度 wrap)
 *
 * 特点:
 *  - 解析梯度 / Hessian (dL/dx = Q e, d2L/dx2 = Q), 无需 CppAd, 无需重编译库
 *  - 轨迹用 ocs2::LinearInterpolation 插值, 查询时间超出范围时钳位到端点
 *    (即 t > t_end 后 MPC 自动 "保持在终点状态")
 *  - 作为软约束加入 OptimalControlProblem::stateCostPtr, 不影响硬约束结构
 *****************************************************************************/

#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/cost/StateCost.h>

namespace ocs2 {
namespace mobile_manipulator {

class WholeBodyTrajectoryCost final : public StateCost {
 public:
  /**
   * @param Q               状态偏差权重矩阵 (stateDim x stateDim, 通常取对角)
   * @param timeTrajectory  航点时间序列 (严格递增, 单位 s, MPC 时钟)
   * @param stateTrajectory 航点全身状态序列 (每个 stateDim 维)
   * @param yawIndex        状态向量中航向角的下标 (WheelBased 模型为 2);
   *                        传入 <0 表示状态中没有需要 wrap 的角度分量
   */
  WholeBodyTrajectoryCost(matrix_t Q, scalar_array_t timeTrajectory,
                          vector_array_t stateTrajectory, int yawIndex = 2);

  ~WholeBodyTrajectoryCost() override = default;

  WholeBodyTrajectoryCost* clone() const override {
    return new WholeBodyTrajectoryCost(*this);
  }

  scalar_t getValue(scalar_t time, const vector_t& state,
                    const TargetTrajectories& targetTrajectories,
                    const PreComputation& preComputation) const override;

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(
      scalar_t time, const vector_t& state,
      const TargetTrajectories& targetTrajectories,
      const PreComputation& preComputation) const override;

  /** 查询 t 时刻的期望全身状态 (线性插值, 端点钳位) */
  vector_t getDesiredState(scalar_t time) const;

 private:
  WholeBodyTrajectoryCost(const WholeBodyTrajectoryCost& rhs) = default;

  /** e = x - x_d(t), 其中 yaw 分量 wrap 到 (-pi, pi] */
  vector_t computeError(scalar_t time, const vector_t& state) const;

  matrix_t Q_;
  scalar_array_t timeTrajectory_;
  vector_array_t stateTrajectory_;
  int yawIndex_;
};

}  // namespace mobile_manipulator
}  // namespace ocs2
