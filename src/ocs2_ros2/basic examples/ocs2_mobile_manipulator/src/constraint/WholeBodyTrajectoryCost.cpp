/******************************************************************************
 * WholeBodyTrajectoryCost.cpp
 *****************************************************************************/

#include "ocs2_mobile_manipulator/constraint/WholeBodyTrajectoryCost.h"

#include <cmath>
#include <stdexcept>

#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2 {
namespace mobile_manipulator {

WholeBodyTrajectoryCost::WholeBodyTrajectoryCost(matrix_t Q,
                                                 scalar_array_t timeTrajectory,
                                                 vector_array_t stateTrajectory,
                                                 int yawIndex)
    : Q_(std::move(Q)),
      timeTrajectory_(std::move(timeTrajectory)),
      stateTrajectory_(std::move(stateTrajectory)),
      yawIndex_(yawIndex) {
  if (timeTrajectory_.empty() || stateTrajectory_.empty()) {
    throw std::runtime_error(
        "[WholeBodyTrajectoryCost] reference trajectory is empty!");
  }
  if (timeTrajectory_.size() != stateTrajectory_.size()) {
    throw std::runtime_error(
        "[WholeBodyTrajectoryCost] time and state trajectory size mismatch!");
  }
  for (size_t i = 0; i + 1 < timeTrajectory_.size(); ++i) {
    if (timeTrajectory_[i + 1] <= timeTrajectory_[i]) {
      throw std::runtime_error(
          "[WholeBodyTrajectoryCost] time trajectory must be strictly increasing!");
    }
  }
  for (const auto& s : stateTrajectory_) {
    if (s.size() != Q_.rows()) {
      throw std::runtime_error(
          "[WholeBodyTrajectoryCost] waypoint state dim != Q dim!");
    }
  }
}

vector_t WholeBodyTrajectoryCost::getDesiredState(scalar_t time) const {
  // LinearInterpolation 在 time 越界时自动钳位到首 / 末航点
  return LinearInterpolation::interpolate(time, timeTrajectory_, stateTrajectory_);
}

vector_t WholeBodyTrajectoryCost::computeError(scalar_t time,
                                               const vector_t& state) const {
  vector_t e = state - getDesiredState(time);
  if (yawIndex_ >= 0 && yawIndex_ < e.size()) {
    e(yawIndex_) = std::atan2(std::sin(e(yawIndex_)), std::cos(e(yawIndex_)));
  }
  return e;
}

scalar_t WholeBodyTrajectoryCost::getValue(
    scalar_t time, const vector_t& state,
    const TargetTrajectories& /*targetTrajectories*/,
    const PreComputation& /*preComputation*/) const {
  const vector_t e = computeError(time, state);
  return 0.5 * e.dot(Q_ * e);
}

ScalarFunctionQuadraticApproximation
WholeBodyTrajectoryCost::getQuadraticApproximation(
    scalar_t time, const vector_t& state,
    const TargetTrajectories& /*targetTrajectories*/,
    const PreComputation& /*preComputation*/) const {
  const vector_t e = computeError(time, state);

  ScalarFunctionQuadraticApproximation L;
  L.f = 0.5 * e.dot(Q_ * e);
  L.dfdx = Q_ * e;    // wrap 后的角度误差对 x 的导数仍为 1, 解析式不变
  L.dfdxx = Q_;
  return L;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
