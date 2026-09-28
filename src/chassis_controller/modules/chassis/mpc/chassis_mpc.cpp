/* SPDX-License-Identifier: Apache-2.0 */

#include "chassis_mpc.h"

#include "chassis_mpc_model.hpp"
#include "static_tinympc_solver.hpp"

#include <Eigen/Core>

namespace {

constexpr int kNx = 6;
constexpr int kNu = 2;
constexpr int kHorizon = 21;
constexpr int kMaximumIterations = 5;
constexpr float kPrimalTolerance = 1.0e-3F;
constexpr float kDualTolerance = 1.0e-3F;
using Solver = wbr::control::StaticTinyMpc<kNx, kNu, kHorizon, float>;

template <int Rows, int Columns>
Eigen::Matrix<float, Rows, Columns> MapRowMajor(const double *data)
{
	return Eigen::Map<const Eigen::Matrix<double, Rows, Columns, Eigen::RowMajor>>(data)
		.template cast<float>();
}

Solver::Cache MakeCache()
{
	Solver::Cache cache{};
	cache.rho = wbr::control::mpc_model::kTinyMpcRho;
	cache.kinf = MapRowMajor<kNu, kNx>(wbr::control::mpc_model::kTinyMpcKinfRowMajor);
	cache.pinf = MapRowMajor<kNx, kNx>(wbr::control::mpc_model::kTinyMpcPinfRowMajor);
	cache.quu_inverse = MapRowMajor<kNu, kNu>(
		wbr::control::mpc_model::kTinyMpcQuuInverseRowMajor);
	cache.a_minus_bk_transpose = MapRowMajor<kNx, kNx>(
		wbr::control::mpc_model::kTinyMpcAmBkTransposeRowMajor);
	cache.affine_state.setZero();
	cache.affine_input.setZero();
	return cache;
}

Solver::Input MakeInputMin()
{
	Solver::Input input;
	input << -2.0F * 16384.0F / 3450.0F, -8.0F;
	return input;
}

Solver::Input MakeInputMax()
{
	return -MakeInputMin();
}

// Static storage keeps the fixed workspace off the 1 kHz thread stack.
Solver g_solver(MapRowMajor<kNx, kNx>(wbr::control::mpc_model::kAdRowMajor),
		MapRowMajor<kNx, kNu>(wbr::control::mpc_model::kBdRowMajor), MakeCache(),
		MakeInputMin(), MakeInputMax());

}  // namespace

namespace wbr::control {

ChassisMpcResult SolveChassisMpc(const std::array<double, 6> &state)
{
	const Solver::State initial_state =
		Eigen::Map<const Eigen::Matrix<double, kNx, 1>>(state.data()).cast<float>();
	const Solver::Result result = g_solver.Solve(initial_state, kMaximumIterations,
						 kPrimalTolerance, kDualTolerance);
	return {
		.solved = result.solved,
		.iterations = result.iterations,
		.first_input = {result.first_input(0), result.first_input(1)},
		.primal_residual = result.primal_residual,
		.dual_residual = result.dual_residual,
	};
}

void ResetChassisMpc()
{
	g_solver.Reset();
}

std::size_t ChassisMpcWorkspaceBytes()
{
	return Solver::WorkspaceBytes();
}

}  // namespace wbr::control
