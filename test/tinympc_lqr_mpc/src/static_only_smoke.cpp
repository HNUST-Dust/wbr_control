#include "generated_chassis_model.hpp"
#include "modules/chassis/mpc/static_tinympc_solver.hpp"

#include <cmath>
#include <cstdio>

namespace {

constexpr int kNx = 6;
constexpr int kNu = 2;
constexpr int kHorizon = 81;
using Solver = wbr::control::StaticTinyMpc<kNx, kNu, kHorizon>;

template <int Rows, int Columns>
Eigen::Matrix<double, Rows, Columns> map_row_major(const double *data)
{
    return Eigen::Map<
        const Eigen::Matrix<double, Rows, Columns, Eigen::RowMajor>>(data);
}

}  // namespace

int main()
{
    const auto a = map_row_major<kNx, kNx>(wbr_tinympc_test::kAdRowMajor);
    const auto b = map_row_major<kNx, kNu>(wbr_tinympc_test::kBdRowMajor);
    Solver::Cache cache{
        wbr_tinympc_test::kTinyMpcRho,
        map_row_major<kNu, kNx>(wbr_tinympc_test::kTinyMpcKinfRowMajor),
        map_row_major<kNx, kNx>(wbr_tinympc_test::kTinyMpcPinfRowMajor),
        map_row_major<kNu, kNu>(wbr_tinympc_test::kTinyMpcQuuInverseRowMajor),
        map_row_major<kNx, kNx>(wbr_tinympc_test::kTinyMpcAmBkTransposeRowMajor),
        Solver::State::Zero(),
        Solver::Input::Zero(),
    };
    Solver::Input input_min;
    input_min << -9.4980, -8.0;
    const Solver::Input input_max = -input_min;
    Solver solver(a, b, cache, input_min, input_max);

    Solver::State state;
    state << 5.0 * 3.141592653589793 / 180.0, 0.0, 0.1, 0.5,
             5.0 * 3.141592653589793 / 180.0, 0.0;

    Eigen::internal::set_is_malloc_allowed(false);
    for (int step = 0; step < 100; ++step) {
        const auto result = solver.Solve(state, 80, 1.0e-3, 1.0e-3);
        if (!result.solved || !result.first_input.allFinite() ||
            std::abs(result.first_input(0)) > 9.4981 ||
            std::abs(result.first_input(1)) > 8.0001) {
            return 1;
        }
        state.noalias() = a * state + b * result.first_input;
    }
    Eigen::internal::set_is_malloc_allowed(true);

    std::printf("PASS: static-only solve, workspace=%zu bytes, no TinyMPC dynamic library\n",
                Solver::WorkspaceBytes());
    return Solver::WorkspaceBytes() <= 384U * 1024U ? 0 : 2;
}
