#include "generated_chassis_model.hpp"
#include "modules/chassis/mpc/static_tinympc_solver.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

namespace {

constexpr int kNx = 6;
constexpr int kNu = 2;
constexpr int kHorizon = 21;
constexpr int kSteps = 1500;
using Solver = wbr::control::StaticTinyMpc<kNx, kNu, kHorizon, float>;
using State = Solver::State;
using Input = Solver::Input;

template <int Rows, int Cols>
Eigen::Matrix<float, Rows, Cols> Map(const double *data)
{
	return Eigen::Map<const Eigen::Matrix<double, Rows, Cols, Eigen::RowMajor>>(data)
		.template cast<float>();
}

Solver MakeSolver()
{
	Solver::Cache cache{};
	cache.rho = static_cast<float>(wbr_tinympc_test::kTinyMpcRho);
	cache.kinf = Map<kNu, kNx>(wbr_tinympc_test::kTinyMpcKinfRowMajor);
	cache.pinf = Map<kNx, kNx>(wbr_tinympc_test::kTinyMpcPinfRowMajor);
	cache.quu_inverse = Map<kNu, kNu>(wbr_tinympc_test::kTinyMpcQuuInverseRowMajor);
	cache.a_minus_bk_transpose =
		Map<kNx, kNx>(wbr_tinympc_test::kTinyMpcAmBkTransposeRowMajor);
	cache.affine_state.setZero();
	cache.affine_input.setZero();
	Input lower;
	lower << -2.0F * 16384.0F / 3450.0F, -8.0F;
	return Solver(Map<kNx, kNx>(wbr_tinympc_test::kAdRowMajor),
		Map<kNx, kNu>(wbr_tinympc_test::kBdRowMajor), cache, lower, -lower);
}

struct Metrics {
	double cost = 0.0;
	float final_norm = 0.0F;
	float max_violation = 0.0F;
	float max_primal = 0.0F;
	float max_dual = 0.0F;
	int cap_hits = 0;
};

Metrics Simulate(const State &initial, int iterations)
{
	auto solver = MakeSolver();
	const auto a = Map<kNx, kNx>(wbr_tinympc_test::kAdRowMajor);
	const auto b = Map<kNx, kNu>(wbr_tinympc_test::kBdRowMajor);
	State state = initial;
	Metrics metrics;
	for (int step = 0; step < kSteps; ++step) {
		const auto result = solver.Solve(state, iterations, 1.0e-3F, 1.0e-3F);
		metrics.cost += static_cast<double>(state.squaredNorm()) * 0.001;
		metrics.max_violation = std::max(metrics.max_violation,
			std::max({0.0F, std::abs(result.first_input(0)) - 2.0F * 16384.0F / 3450.0F,
				std::abs(result.first_input(1)) - 8.0F}));
		metrics.max_primal = std::max(metrics.max_primal, result.primal_residual);
		metrics.max_dual = std::max(metrics.max_dual, result.dual_residual);
		if (!result.solved) {
			++metrics.cap_hits;
		}
		state.noalias() = a * state + b * result.first_input;
		if (!state.allFinite()) {
			metrics.final_norm = INFINITY;
			return metrics;
		}
	}
	metrics.final_norm = state.norm();
	return metrics;
}

}  // namespace

int main()
{
	constexpr float kDeg = 0.01745329251994329577F;
	const std::array<State, 3> scenarios = {
		(State() << 0.0F, 0.0F, 0.0F, 0.0F, 8.0F * kDeg, 0.0F).finished(),
		(State() << 8.0F * kDeg, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F).finished(),
		(State() << 5.0F * kDeg, 0.0F, 0.1F, 0.5F, 5.0F * kDeg, 0.0F).finished(),
	};
	bool safe = true;
	for (int iterations = 5; iterations <= 8; ++iterations) {
		for (std::size_t scenario = 0; scenario < scenarios.size(); ++scenario) {
			const Metrics m = Simulate(scenarios[scenario], iterations);
			std::printf("iter=%d scenario=%zu cost=%.6f final=%.6f violation=%.3g "
				"pri=%.4g dua=%.4g cap_hits=%d/%d\n", iterations, scenario,
				m.cost, m.final_norm, m.max_violation, m.max_primal, m.max_dual,
				m.cap_hits, kSteps);
			safe &= std::isfinite(m.final_norm) && m.max_violation <= 1.0e-5F;
		}
	}
	return safe ? 0 : 1;
}
