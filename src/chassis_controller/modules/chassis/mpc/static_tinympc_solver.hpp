#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <type_traits>

namespace wbr::control {

// Application-maintained, fixed-size TinyMPC online kernel for zero-reference
// regulation with input box constraints. Cache construction remains an offline
// operation; Solve() performs no setup, resizing, factorization, or allocation.
template <int Nx, int Nu, int Horizon, typename ScalarType = double>
class StaticTinyMpc {
  public:
    using Scalar = ScalarType;
    static_assert(Nx > 0);
    static_assert(Nu > 0);
    static_assert(Horizon > 1);
    static_assert(std::is_floating_point_v<Scalar>);
    using State = Eigen::Matrix<Scalar, Nx, 1>;
    using Input = Eigen::Matrix<Scalar, Nu, 1>;
    using StateMatrix = Eigen::Matrix<Scalar, Nx, Nx>;
    using InputMatrix = Eigen::Matrix<Scalar, Nx, Nu>;
    using GainMatrix = Eigen::Matrix<Scalar, Nu, Nx>;
    using InputSquareMatrix = Eigen::Matrix<Scalar, Nu, Nu>;
    using StateTrajectory = Eigen::Matrix<Scalar, Nx, Horizon>;
    using InputTrajectory = Eigen::Matrix<Scalar, Nu, Horizon - 1>;

    static_assert(StateTrajectory::SizeAtCompileTime == Nx * Horizon);
    static_assert(InputTrajectory::SizeAtCompileTime == Nu * (Horizon - 1));
    static_assert(StateTrajectory::RowsAtCompileTime != Eigen::Dynamic);
    static_assert(InputTrajectory::ColsAtCompileTime != Eigen::Dynamic);

    struct Cache {
        Scalar rho;
        GainMatrix kinf;
        StateMatrix pinf;
        InputSquareMatrix quu_inverse;
        StateMatrix a_minus_bk_transpose;
        State affine_state;
        Input affine_input;
    };

    struct Result {
        bool solved;
        int iterations;
        Input first_input;
        Scalar primal_residual;
        Scalar dual_residual;
    };

    StaticTinyMpc(const StateMatrix &a, const InputMatrix &b, const Cache &cache,
                  const Input &input_min, const Input &input_max)
        : a_(a), b_(b), cache_(cache),
          input_min_(input_min), input_max_(input_max)
    {
        Reset();
    }

    void Reset()
    {
        x_.setZero();
        u_.setZero();
        q_.setZero();
        r_.setZero();
        p_.setZero();
        d_.setZero();
        v_new_.setZero();
        z_new_.setZero();
        g_.setZero();
        y_.setZero();
    }

    Result Solve(const State &initial_state, int maximum_iterations,
                 Scalar primal_tolerance, Scalar dual_tolerance)
    {
        x_.col(0) = initial_state;
        Scalar primal_residual = 0.0;
        Scalar dual_residual = 0.0;

        for (int iteration = 1; iteration <= maximum_iterations; ++iteration) {
            // References and affine dynamics are zero in this regulator test.
            q_ = -cache_.rho * (v_new_ - g_);
            r_ = -cache_.rho * (z_new_ - y_);
            p_.col(Horizon - 1) =
                -cache_.rho * (v_new_.col(Horizon - 1) - g_.col(Horizon - 1));

            for (int stage = Horizon - 2; stage >= 0; --stage) {
                d_.col(stage).noalias() = cache_.quu_inverse *
                    (b_.transpose() * p_.col(stage + 1) + r_.col(stage) +
                     cache_.affine_input);
                p_.col(stage).noalias() =
                    q_.col(stage) +
                    cache_.a_minus_bk_transpose * p_.col(stage + 1) -
                    cache_.kinf.transpose() * r_.col(stage) + cache_.affine_state;
            }

            for (int stage = 0; stage < Horizon - 1; ++stage) {
                u_.col(stage).noalias() =
                    -cache_.kinf * x_.col(stage) - d_.col(stage);
                x_.col(stage + 1).noalias() =
                    a_ * x_.col(stage) + b_ * u_.col(stage);
            }

            // Update consensus and residuals in place. Computing the delta
            // before overwriting removes the two full previous-trajectory
            // copies used by the generic implementation.
            Scalar state_primal = Scalar(0);
            Scalar input_primal = Scalar(0);
            Scalar state_dual = Scalar(0);
            Scalar input_dual = Scalar(0);
            for (int stage = 0; stage < Horizon; ++stage) {
                for (int state = 0; state < Nx; ++state) {
                    const Scalar projected = x_(state, stage) + g_(state, stage);
                    state_primal = std::max(
                        state_primal, std::abs(x_(state, stage) - projected));
                    state_dual = std::max(
                        state_dual,
                        cache_.rho * std::abs(v_new_(state, stage) - projected));
                    v_new_(state, stage) = projected;
                    g_(state, stage) += x_(state, stage) - projected;
                }
            }
            for (int stage = 0; stage < Horizon - 1; ++stage) {
                for (int input = 0; input < Nu; ++input) {
                    const Scalar projected = std::clamp(
                        u_(input, stage) + y_(input, stage), input_min_(input),
                        input_max_(input));
                    input_primal = std::max(
                        input_primal, std::abs(u_(input, stage) - projected));
                    input_dual = std::max(
                        input_dual,
                        cache_.rho * std::abs(z_new_(input, stage) - projected));
                    z_new_(input, stage) = projected;
                    y_(input, stage) += u_(input, stage) - projected;
                }
            }

            primal_residual = std::max(state_primal, input_primal);
            dual_residual = std::max(state_dual, input_dual);

            if (primal_residual < primal_tolerance &&
                dual_residual < dual_tolerance) {
                return {true, iteration, z_new_.col(0), primal_residual,
                        dual_residual};
            }
        }

        return {false, maximum_iterations, z_new_.col(0), primal_residual,
                dual_residual};
    }

    static constexpr std::size_t WorkspaceBytes()
    {
        return sizeof(StaticTinyMpc);
    }

  private:
    StateMatrix a_;
    InputMatrix b_;
    Cache cache_;
    Input input_min_;
    Input input_max_;

    StateTrajectory x_;
    InputTrajectory u_;
    StateTrajectory q_;
    InputTrajectory r_;
    StateTrajectory p_;
    InputTrajectory d_;
    StateTrajectory v_new_;
    InputTrajectory z_new_;
    StateTrajectory g_;
    InputTrajectory y_;
};

}  // namespace wbr::control
