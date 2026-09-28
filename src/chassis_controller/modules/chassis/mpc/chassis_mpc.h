/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <array>
#include <cstddef>

namespace wbr::control {

struct ChassisMpcResult {
	bool solved;
	int iterations;
	std::array<double, 2> first_input;
	double primal_residual;
	double dual_residual;
};

ChassisMpcResult SolveChassisMpc(const std::array<double, 6> &state);
void ResetChassisMpc();
std::size_t ChassisMpcWorkspaceBytes();

}  // namespace wbr::control
