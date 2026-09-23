#include <tinympc/tiny_api.hpp>

#include "generated_chassis_model.hpp"
#include "modules/chassis/mpc/static_tinympc_solver.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <iostream>
#include <streambuf>
#include <string_view>
#include <vector>

namespace {

constexpr int kNx = 6;
constexpr int kNu = 2;
constexpr int kHorizon = 81;  // 80 ms at the production 1 kHz rate.
constexpr int kSimulationSteps = 1500;
constexpr int kBenchmarkSolves = 2000;

constexpr double kWheelCurrentLimit = 16384.0;
constexpr double kWheelCurrentPerNm = 3450.0;
// The plant input T is the sum of both wheel torques.
constexpr double kTotalWheelTorqueLimitNm =
    2.0 * kWheelCurrentLimit / kWheelCurrentPerNm;
// Conservative total body-on-leg torque limit. The production VMC subsequently
// maps half of Tp plus support force into each joint and clamps each motor at
// 54 Nm. A configuration-dependent VMC inequality is deliberately left for a
// later time-varying-constraint experiment; this bound prevents the test MPC
// from requesting the unbounded Tp produced by the unconstrained LQR.
constexpr double kTotalLegTorqueLimitNm = 8.0;

using StateVector = Eigen::Matrix<tinytype, kNx, 1>;
using InputVector = Eigen::Matrix<tinytype, kNu, 1>;
using MatrixA = Eigen::Matrix<tinytype, kNx, kNx>;
using MatrixB = Eigen::Matrix<tinytype, kNx, kNu>;
using MatrixK = Eigen::Matrix<tinytype, kNu, kNx>;
using StaticSolver = wbr::control::StaticTinyMpc<kNx, kNu, kHorizon>;

struct Scenario {
    std::string_view name;
    StateVector initial_state;
};

struct Metrics {
    double integrated_state_error = 0.0;
    double maximum_input_violation = 0.0;
    double maximum_abs_theta = 0.0;
    double maximum_abs_pitch = 0.0;
    double final_state_norm = 0.0;
    int saturated_steps = 0;
    int failed_solves = 0;
    int maximum_iterations = 0;
};

class NullStreamBuffer final : public std::streambuf {
  public:
    int overflow(int character) override { return character; }
};

class ScopedCoutSilencer {
  public:
    ScopedCoutSilencer() : previous_(std::cout.rdbuf(&sink_)) {}
    ~ScopedCoutSilencer() { std::cout.rdbuf(previous_); }

    ScopedCoutSilencer(const ScopedCoutSilencer &) = delete;
    ScopedCoutSilencer &operator=(const ScopedCoutSilencer &) = delete;

  private:
    NullStreamBuffer sink_;
    std::streambuf *previous_;
};

class ScopedEigenMallocDisabler {
  public:
    ScopedEigenMallocDisabler() { Eigen::internal::set_is_malloc_allowed(false); }
    ~ScopedEigenMallocDisabler() { Eigen::internal::set_is_malloc_allowed(true); }

    ScopedEigenMallocDisabler(const ScopedEigenMallocDisabler &) = delete;
    ScopedEigenMallocDisabler &operator=(const ScopedEigenMallocDisabler &) = delete;
};

InputVector clamp_input(const InputVector &input)
{
    InputVector result = input;
    result(0) = std::clamp(result(0), -kTotalWheelTorqueLimitNm,
                           kTotalWheelTorqueLimitNm);
    result(1) = std::clamp(result(1), -kTotalLegTorqueLimitNm,
                           kTotalLegTorqueLimitNm);
    return result;
}

double input_violation(const InputVector &input)
{
    return std::max({0.0,
                     std::abs(input(0)) - kTotalWheelTorqueLimitNm,
                     std::abs(input(1)) - kTotalLegTorqueLimitNm});
}

void update_metrics(Metrics &metrics, const StateVector &state, const InputVector &raw_input,
                    const InputVector &applied_input)
{
    metrics.integrated_state_error += state.squaredNorm() * wbr_tinympc_test::kSampleTimeS;
    metrics.maximum_input_violation =
        std::max(metrics.maximum_input_violation, input_violation(raw_input));
    metrics.maximum_abs_theta = std::max(metrics.maximum_abs_theta, std::abs(state(0)));
    metrics.maximum_abs_pitch = std::max(metrics.maximum_abs_pitch, std::abs(state(4)));
    if (!raw_input.isApprox(applied_input, 1.0e-9)) {
        ++metrics.saturated_steps;
    }
}

Metrics simulate_lqr(const Scenario &scenario, const MatrixA &ad, const MatrixB &bd,
                     const MatrixK &gain)
{
    Metrics metrics;
    StateVector state = scenario.initial_state;
    for (int step = 0; step < kSimulationSteps; ++step) {
        const InputVector raw_input = -gain * state;
        const InputVector applied_input = clamp_input(raw_input);
        update_metrics(metrics, state, raw_input, applied_input);
        state = ad * state + bd * applied_input;
        if (!state.allFinite()) {
            metrics.failed_solves = 1;
            break;
        }
    }
    metrics.final_state_norm = state.norm();
    return metrics;
}

TinySolver *make_solver(const MatrixA &ad, const MatrixB &bd)
{
    TinySolver *solver = nullptr;
    const tinyVector fdyn = tinyVector::Zero(kNx);
    tinyVector q(kNx);
    q << 1500.0, 100.0, 500.0, 300.0, 24000.0, 800.0;
    tinyVector r(kNu);
    r << 90.0, 1.0;

    constexpr tinytype rho = 10.0;
    if (tiny_setup(&solver, ad, bd, fdyn, q.asDiagonal(), r.asDiagonal(), rho,
                   kNx, kNu, kHorizon, 0) != 0) {
        return nullptr;
    }

    tinyMatrix x_min = tinyMatrix::Constant(kNx, kHorizon, -1.0e12);
    tinyMatrix x_max = tinyMatrix::Constant(kNx, kHorizon, 1.0e12);
    tinyMatrix u_min = tinyMatrix::Zero(kNu, kHorizon - 1);
    tinyMatrix u_max = tinyMatrix::Zero(kNu, kHorizon - 1);
    u_min.row(0).setConstant(-kTotalWheelTorqueLimitNm);
    u_max.row(0).setConstant(kTotalWheelTorqueLimitNm);
    u_min.row(1).setConstant(-kTotalLegTorqueLimitNm);
    u_max.row(1).setConstant(kTotalLegTorqueLimitNm);
    if (tiny_set_bound_constraints(solver, x_min, x_max, u_min, u_max) != 0) {
        return nullptr;
    }

    solver->settings->max_iter = 80;
    solver->settings->check_termination = 1;
    solver->settings->abs_pri_tol = 1.0e-3;
    solver->settings->abs_dua_tol = 1.0e-3;
    solver->settings->en_state_bound = 0;
    solver->settings->en_input_bound = 1;
    solver->settings->adaptive_rho = 0;
    return solver;
}

StaticSolver make_static_solver(const MatrixA &ad, const MatrixB &bd)
{
    const MatrixK kinf =
        Eigen::Map<const Eigen::Matrix<double, kNu, kNx, Eigen::RowMajor>>(
            wbr_tinympc_test::kTinyMpcKinfRowMajor);
    const MatrixA pinf =
        Eigen::Map<const Eigen::Matrix<double, kNx, kNx, Eigen::RowMajor>>(
            wbr_tinympc_test::kTinyMpcPinfRowMajor);
    const Eigen::Matrix<double, kNu, kNu> quu_inverse =
        Eigen::Map<const Eigen::Matrix<double, kNu, kNu, Eigen::RowMajor>>(
            wbr_tinympc_test::kTinyMpcQuuInverseRowMajor);
    const MatrixA am_bk_transpose =
        Eigen::Map<const Eigen::Matrix<double, kNx, kNx, Eigen::RowMajor>>(
            wbr_tinympc_test::kTinyMpcAmBkTransposeRowMajor);
    StaticSolver::Cache cache{
        wbr_tinympc_test::kTinyMpcRho,
        kinf,
        pinf,
        quu_inverse,
        am_bk_transpose,
        StateVector::Zero(),
        InputVector::Zero(),
    };
    InputVector input_min;
    input_min << -kTotalWheelTorqueLimitNm, -kTotalLegTorqueLimitNm;
    InputVector input_max = -input_min;
    return StaticSolver(ad, bd, cache, input_min, input_max);
}

Metrics simulate_mpc(const Scenario &scenario, TinySolver *solver,
                     const MatrixA &ad, const MatrixB &bd)
{
    ScopedCoutSilencer silence_solver_progress;
    Metrics metrics;
    StateVector state = scenario.initial_state;
    for (int step = 0; step < kSimulationSteps; ++step) {
        tiny_set_x0(solver, state);
        const int status = tiny_solve(solver);
        const InputVector raw_input = solver->work->u.col(0);
        const InputVector applied_input = clamp_input(raw_input);
        metrics.maximum_iterations =
            std::max(metrics.maximum_iterations, solver->solution->iter);
        if (status != 0 || !solver->solution->solved || !raw_input.allFinite()) {
            ++metrics.failed_solves;
        }
        update_metrics(metrics, state, raw_input, applied_input);
        state = ad * state + bd * applied_input;
        if (!state.allFinite()) {
            ++metrics.failed_solves;
            break;
        }
    }
    metrics.final_state_norm = state.norm();
    return metrics;
}

Metrics simulate_static_mpc(const Scenario &scenario, StaticSolver &solver,
                            const MatrixA &ad, const MatrixB &bd)
{
    Metrics metrics;
    StateVector state = scenario.initial_state;
    for (int step = 0; step < kSimulationSteps; ++step) {
        StaticSolver::Result result;
        {
            // Eigen aborts immediately if any expression in the online solve
            // attempts heap allocation while EIGEN_RUNTIME_NO_MALLOC is active.
            ScopedEigenMallocDisabler forbid_heap;
            result = solver.Solve(state, 80, 1.0e-3, 1.0e-3);
        }
        const InputVector applied_input = clamp_input(result.first_input);
        metrics.maximum_iterations =
            std::max(metrics.maximum_iterations, result.iterations);
        if (!result.solved || !result.first_input.allFinite()) {
            ++metrics.failed_solves;
        }
        update_metrics(metrics, state, result.first_input, applied_input);
        state = ad * state + bd * applied_input;
        if (!state.allFinite()) {
            ++metrics.failed_solves;
            break;
        }
    }
    metrics.final_state_norm = state.norm();
    return metrics;
}

void print_metrics(std::string_view controller, const Metrics &metrics)
{
    std::printf("  %-7.*s cost=%10.5f final_norm=%9.5f max_theta=%7.3fdeg "
                "max_pitch=%7.3fdeg saturation=%4d violation=%9.3g "
                "failures=%d max_iter=%d\n",
                static_cast<int>(controller.size()), controller.data(),
                metrics.integrated_state_error, metrics.final_state_norm,
                metrics.maximum_abs_theta * 180.0 / M_PI,
                metrics.maximum_abs_pitch * 180.0 / M_PI,
                metrics.saturated_steps, metrics.maximum_input_violation,
                metrics.failed_solves, metrics.maximum_iterations);
}

void benchmark(TinySolver *solver, const StateVector &state)
{
    ScopedCoutSilencer silence_solver_progress;
    std::vector<double> samples_us;
    samples_us.reserve(kBenchmarkSolves);
    for (int index = 0; index < kBenchmarkSolves; ++index) {
        tiny_set_x0(solver, state);
        const auto start = std::chrono::steady_clock::now();
        tiny_solve(solver);
        const auto end = std::chrono::steady_clock::now();
        samples_us.push_back(
            std::chrono::duration<double, std::micro>(end - start).count());
    }
    std::sort(samples_us.begin(), samples_us.end());
    const auto percentile = [&](double p) {
        const std::size_t index = static_cast<std::size_t>(p * (samples_us.size() - 1));
        return samples_us[index];
    };
    double sum = 0.0;
    for (double sample : samples_us) {
        sum += sample;
    }
    std::printf("host solve timing (%d warm solves): mean=%.2fus p99=%.2fus "
                "p99.9=%.2fus max=%.2fus\n",
                kBenchmarkSolves, sum / samples_us.size(), percentile(0.99),
                percentile(0.999), samples_us.back());
}

void benchmark_static(StaticSolver &solver, const StateVector &state)
{
    std::vector<double> samples_us;
    samples_us.reserve(kBenchmarkSolves);
    for (int index = 0; index < kBenchmarkSolves; ++index) {
        const auto start = std::chrono::steady_clock::now();
        {
            ScopedEigenMallocDisabler forbid_heap;
            (void)solver.Solve(state, 80, 1.0e-3, 1.0e-3);
        }
        const auto end = std::chrono::steady_clock::now();
        samples_us.push_back(
            std::chrono::duration<double, std::micro>(end - start).count());
    }
    std::sort(samples_us.begin(), samples_us.end());
    const auto percentile = [&](double p) {
        return samples_us[static_cast<std::size_t>(p * (samples_us.size() - 1))];
    };
    double sum = 0.0;
    for (double sample : samples_us) {
        sum += sample;
    }
    std::printf("static solve timing (%d warm solves): mean=%.2fus p99=%.2fus "
                "p99.9=%.2fus max=%.2fus; workspace=%zu bytes\n",
                kBenchmarkSolves, sum / samples_us.size(), percentile(0.99),
                percentile(0.999), samples_us.back(), StaticSolver::WorkspaceBytes());
}

}  // namespace

int main()
{
    const MatrixA ad = Eigen::Map<const Eigen::Matrix<double, kNx, kNx, Eigen::RowMajor>>(
        wbr_tinympc_test::kAdRowMajor);
    const MatrixB bd = Eigen::Map<const Eigen::Matrix<double, kNx, kNu, Eigen::RowMajor>>(
        wbr_tinympc_test::kBdRowMajor);
    const MatrixK gain =
        Eigen::Map<const Eigen::Matrix<double, kNu, kNx, Eigen::RowMajor>>(
            wbr_tinympc_test::kContinuousLqrGainRowMajor);

    std::array<Scenario, 3> scenarios{{
        {"pitch_8deg", (StateVector() << 0.0, 0.0, 0.0, 0.0,
                         8.0 * M_PI / 180.0, 0.0).finished()},
        {"theta_8deg", (StateVector() << 8.0 * M_PI / 180.0, 0.0, 0.0, 0.0,
                         0.0, 0.0).finished()},
        {"combined", (StateVector() << 5.0 * M_PI / 180.0, 0.0, 0.10, 0.5,
                        5.0 * M_PI / 180.0, 0.0).finished()},
    }};

    std::printf("model_leg_length=%.5fm dt=%.3fms horizon=%d (%.1fms)\n",
                wbr_tinympc_test::kModelLegLengthM,
                1000.0 * wbr_tinympc_test::kSampleTimeS, kHorizon,
                1000.0 * (kHorizon - 1) * wbr_tinympc_test::kSampleTimeS);
    std::printf("constraints: total wheel |T|<=%.4fNm, total leg |Tp|<=%.2fNm\n",
                kTotalWheelTorqueLimitNm, kTotalLegTorqueLimitNm);

    bool passed = true;
    bool cache_checked = false;
    TinySolver *last_solver = nullptr;
    StaticSolver *last_static_solver = nullptr;
    for (const Scenario &scenario : scenarios) {
        TinySolver *solver = make_solver(ad, bd);
        if (solver == nullptr) {
            std::fprintf(stderr, "TinyMPC setup failed\n");
            return 2;
        }
        if (!cache_checked) {
            const MatrixK generated_kinf =
                Eigen::Map<const Eigen::Matrix<double, kNu, kNx, Eigen::RowMajor>>(
                    wbr_tinympc_test::kTinyMpcKinfRowMajor);
            const MatrixA generated_pinf =
                Eigen::Map<const Eigen::Matrix<double, kNx, kNx, Eigen::RowMajor>>(
                    wbr_tinympc_test::kTinyMpcPinfRowMajor);
            std::printf("offline cache delta: Kinf=%.3g Pinf=%.3g\n",
                        (generated_kinf - solver->cache->Kinf).cwiseAbs().maxCoeff(),
                        (generated_pinf - solver->cache->Pinf).cwiseAbs().maxCoeff());
            cache_checked = true;
        }
        auto static_solver = make_static_solver(ad, bd);
        const Metrics lqr = simulate_lqr(scenario, ad, bd, gain);
        const Metrics mpc = simulate_mpc(scenario, solver, ad, bd);
        const Metrics static_mpc =
            simulate_static_mpc(scenario, static_solver, ad, bd);
        std::printf("scenario %.*s\n", static_cast<int>(scenario.name.size()),
                    scenario.name.data());
        print_metrics("LQR", lqr);
        print_metrics("Dynamic", mpc);
        print_metrics("Static", static_mpc);
        passed &= static_mpc.maximum_input_violation <= 5.0e-3;
        passed &= static_mpc.failed_solves == 0;
        passed &= std::isfinite(static_mpc.final_state_norm);
        passed &= std::abs(static_mpc.integrated_state_error -
                           mpc.integrated_state_error) <= 1.0e-8;
        last_solver = solver;
        // Keep a separately initialized solver for the timing run because the
        // scenario-local object goes out of scope here.
        delete last_static_solver;
        last_static_solver = new StaticSolver(make_static_solver(ad, bd));
    }

    benchmark(last_solver, scenarios.back().initial_state);
    benchmark_static(*last_static_solver, scenarios.back().initial_state);
    if (!passed) {
        std::fprintf(stderr, "FAIL: constrained TinyMPC acceptance criteria not met\n");
        return 1;
    }
    std::puts("PASS: TinyMPC respected torque bounds and completed all closed-loop solves");
    return 0;
}
