/******************************************************************************
 * WholeBodyTrajectoryCost.cpp
 *****************************************************************************/

#include "ocs2_mobile_manipulator/cost/WholeBodyTrajectoryCost.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace ocs2
{
  namespace mobile_manipulator
  {

    WholeBodyTrajectoryCost::WholeBodyTrajectoryCost(matrix_t Q, int yawIndex,
                                                     vector_t fallbackState)
        : Q_(std::move(Q)),
          yawIndex_(yawIndex),
          fallbackState_(std::move(fallbackState))
    {
      if (Q_.rows() != Q_.cols())
      {
        throw std::runtime_error("[WholeBodyTrajectoryCost] Q must be square!");
      }
      if (fallbackState_.size() != Q_.rows())
      {
        throw std::runtime_error(
            "[WholeBodyTrajectoryCost] fallbackState dim != Q dim!");
      }
      if (yawIndex_ >= static_cast<int>(Q_.rows()))
      {
        throw std::runtime_error("[WholeBodyTrajectoryCost] yawIndex out of range!");
      }
    }

    WholeBodyTrajectoryCost *WholeBodyTrajectoryCost::clone() const
    {
      return new WholeBodyTrajectoryCost(*this);
    }

    scalar_t WholeBodyTrajectoryCost::wrapToPi(scalar_t angle)
    {
      return std::atan2(std::sin(angle), std::cos(angle));
    }

    vector_t WholeBodyTrajectoryCost::getDesiredState(
        scalar_t time, const TargetTrajectories &targetTrajectories) const
    {
      const auto &ts = targetTrajectories.timeTrajectory;
      const auto &xs = targetTrajectories.stateTrajectory;

      const auto stateDim = Q_.rows();

      // 1) 空 reference -> 用 fallback (通常是 initialState), 相当于"停在原地"
      if (ts.empty() || xs.empty())
      {
        return fallbackState_;
      }

      // 2) 维度检查。若还是旧的 7 维 EE pose, 直接关闭本 cost。
      if (xs.front().size() != stateDim)
      {
        static bool warned = false;
        if (!warned)
        {
          warned = true;
          std::cerr << "[WholeBodyTrajectoryCost] WARNING: target state dim = "
                    << xs.front().size() << ", expected " << stateDim
                    << ". Whole-body tracking cost is DISABLED until a valid "
                       "whole-body TargetTrajectories arrives.\n";
        }
        return vector_t(); // size == 0 -> 调用方返回零 cost
      }

      // 3) 单点 reference
      if (xs.size() == 1)
      {
        return xs.front();
      }

      // 4) 越界钳位
      if (time <= ts.front())
      {
        return xs.front();
      }
      if (time >= ts.back())
      {
        return xs.back();
      }

      // 5) 线性插值 (yaw 走最短角路径)
      const auto it = std::upper_bound(ts.begin(), ts.end(), time);
      const size_t idx =
          static_cast<size_t>(std::distance(ts.begin(), it)) - 1U; // ts[idx] <= t < ts[idx+1]

      const scalar_t t0 = ts[idx];
      const scalar_t t1 = ts[idx + 1];
      const scalar_t dt = t1 - t0;

      const scalar_t alpha =
          (dt > 1e-9) ? std::min(std::max((time - t0) / dt, 0.0), 1.0) : 0.0;

      const vector_t &x0 = xs[idx];
      const vector_t &x1 = xs[idx + 1];

      if (x1.size() != stateDim)
      {
        return x0;
      }

      vector_t xd = (1.0 - alpha) * x0 + alpha * x1;

      if (yawIndex_ >= 0)
      {
        const scalar_t y0 = x0(yawIndex_);
        const scalar_t y1 = x1(yawIndex_);
        xd(yawIndex_) = y0 + alpha * wrapToPi(y1 - y0);
      }

      return xd;
    }

    scalar_t WholeBodyTrajectoryCost::getValue(
        scalar_t time, const vector_t &state,
        const TargetTrajectories &targetTrajectories,
        const PreComputation & /*preComputation*/) const
    {
      const vector_t xd = getDesiredState(time, targetTrajectories);
      if (xd.size() != state.size())
      {
        return 0.0; // reference 不可用 -> 本项不产生任何梯度
      }

      vector_t e = state - xd;
      if (yawIndex_ >= 0)
      {
        e(yawIndex_) = wrapToPi(e(yawIndex_));
      }

      return 0.5 * e.dot(Q_ * e);
    }

    ScalarFunctionQuadraticApproximation
    WholeBodyTrajectoryCost::getQuadraticApproximation(
        scalar_t time, const vector_t &state,
        const TargetTrajectories &targetTrajectories,
        const PreComputation & /*preComputation*/) const
    {
      ScalarFunctionQuadraticApproximation L;

      const vector_t xd = getDesiredState(time, targetTrajectories);
      if (xd.size() != state.size())
      {
        L.f = 0.0;
        L.dfdx = vector_t::Zero(state.rows());
        L.dfdxx = matrix_t::Zero(state.rows(), state.rows());
        return L;
      }

      vector_t e = state - xd;
      if (yawIndex_ >= 0)
      {
        // wrap 之后, d(e_yaw)/d(yaw) 仍然是 1 (除了 +-pi 的测度零点),
        // 所以一阶 / 二阶解析式保持不变。
        e(yawIndex_) = wrapToPi(e(yawIndex_));
      }

      L.f = 0.5 * e.dot(Q_ * e);
      L.dfdx = Q_ * e;
      L.dfdxx = Q_;
      return L;
    }

  } // namespace mobile_manipulator
} // namespace ocs2
