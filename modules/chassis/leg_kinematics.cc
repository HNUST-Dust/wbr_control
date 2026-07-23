#include "leg_kinematics.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kLengthAB = 0.0945;
constexpr double kLengthBC = 0.1125;
constexpr double kLengthCD = 0.116;
constexpr double kLengthAD = 0.090;
constexpr double kLengthAG = 0.210;
constexpr double kLengthGH = 0.250;
constexpr double kJacobianStep = 1.0e-6;
constexpr double kMinLegLength = 1.0e-4;

}  // namespace

namespace modules {

bool ForwardKinematics(double phi1, double phi2, int branch,
                       double& hx, double& hz) {
  const double bx = kLengthAB * std::cos(phi2);
  const double bz = -kLengthAB * std::sin(phi2);
  const double dx = kLengthAD * std::cos(phi1);
  const double dz = -kLengthAD * std::sin(phi1);
  const double gx = kLengthAG * std::cos(phi1);
  const double gz = -kLengthAG * std::sin(phi1);

  const double dbx = dx - bx;
  const double dbz = dz - bz;
  const double distance = std::hypot(dbx, dbz);
  if (distance > kLengthBC + kLengthCD - 1e-9 || distance < 1e-9) {
    return false;
  }

  const double a = (kLengthBC * kLengthBC - kLengthCD * kLengthCD +
                    distance * distance) /
                   (2.0 * distance);
  const double h = std::sqrt(std::fmax(0.0, kLengthBC * kLengthBC - a * a));
  const double px = bx + a * dbx / distance;
  const double pz = bz + a * dbz / distance;
  const double cx = branch == 1 ? px - h * dbz / distance
                                : px + h * dbz / distance;
  const double cz = branch == 1 ? pz + h * dbx / distance
                                : pz - h * dbx / distance;

  const double dcx = cx - dx;
  const double dcz = cz - dz;
  const double dc_length = std::hypot(dcx, dcz);
  if (dc_length < 1e-12) return false;
  hx = gx + kLengthGH * dcx / dc_length;
  hz = gz + kLengthGH * dcz / dc_length;
  return true;
}

bool InverseKinematics(double target_hx, double target_hz, int branch,
                       double seed_phi1, double seed_phi2,
                       double& phi1, double& phi2) {
  if (!std::isfinite(target_hx) || !std::isfinite(target_hz) ||
      !std::isfinite(seed_phi1) || !std::isfinite(seed_phi2)) {
    return false;
  }

  phi1 = seed_phi1;
  phi2 = seed_phi2;
  constexpr int kMaxIterations = 24;
  constexpr double kDamping = 1e-5;
  constexpr double kPositionTolerance = 2e-5;
  constexpr double kMaxJointStep = 0.12;

  for (int iteration = 0; iteration < kMaxIterations; ++iteration) {
    double hx = 0.0;
    double hz = 0.0;
    double jacobian[2][2] = {};
    if (!ForwardKinematics(phi1, phi2, branch, hx, hz) ||
        !NumericalJacobian(phi1, phi2, branch, jacobian)) {
      return false;
    }

    const double error_x = target_hx - hx;
    const double error_z = target_hz - hz;
    if (std::hypot(error_x, error_z) <= kPositionTolerance) {
      return true;
    }

    /* Damped least squares: dq = J^T (J J^T + lambda I)^-1 e. */
    const double a00 = jacobian[0][0] * jacobian[0][0] +
                       jacobian[0][1] * jacobian[0][1] + kDamping;
    const double a01 = jacobian[0][0] * jacobian[1][0] +
                       jacobian[0][1] * jacobian[1][1];
    const double a11 = jacobian[1][0] * jacobian[1][0] +
                       jacobian[1][1] * jacobian[1][1] + kDamping;
    const double determinant = a00 * a11 - a01 * a01;
    if (std::abs(determinant) < 1e-12) {
      return false;
    }
    const double y0 = (a11 * error_x - a01 * error_z) / determinant;
    const double y1 = (-a01 * error_x + a00 * error_z) / determinant;
    const double step_phi1 = std::clamp(
        jacobian[0][0] * y0 + jacobian[1][0] * y1,
        -kMaxJointStep, kMaxJointStep);
    const double step_phi2 = std::clamp(
        jacobian[0][1] * y0 + jacobian[1][1] * y1,
        -kMaxJointStep, kMaxJointStep);
    phi1 += step_phi1;
    phi2 += step_phi2;
  }

  double hx = 0.0;
  double hz = 0.0;
  return ForwardKinematics(phi1, phi2, branch, hx, hz) &&
      std::hypot(target_hx - hx, target_hz - hz) <= 5e-4;
}

bool NumericalJacobian(double phi1, double phi2, int branch,
                       double jacobian[2][2]) {
  double hx_plus;
  double hz_plus;
  double hx_minus;
  double hz_minus;
  if (!ForwardKinematics(phi1 + kJacobianStep, phi2, branch,
                         hx_plus, hz_plus) ||
      !ForwardKinematics(phi1 - kJacobianStep, phi2, branch,
                         hx_minus, hz_minus)) {
    return false;
  }
  jacobian[0][0] = (hx_plus - hx_minus) / (2.0 * kJacobianStep);
  jacobian[1][0] = (hz_plus - hz_minus) / (2.0 * kJacobianStep);
  if (!ForwardKinematics(phi1, phi2 + kJacobianStep, branch,
                         hx_plus, hz_plus) ||
      !ForwardKinematics(phi1, phi2 - kJacobianStep, branch,
                         hx_minus, hz_minus)) {
    return false;
  }
  jacobian[0][1] = (hx_plus - hx_minus) / (2.0 * kJacobianStep);
  jacobian[1][1] = (hz_plus - hz_minus) / (2.0 * kJacobianStep);
  return true;
}

bool ComputeLegKinematics(double phi1, double phi2,
                          double dphi1, double dphi2, int branch,
                          LegKinematics& leg) {
  if (!ForwardKinematics(phi1, phi2, branch, leg.hx, leg.hz) ||
      !NumericalJacobian(phi1, phi2, branch, leg.jacobian)) {
    return false;
  }
  leg.length = std::hypot(leg.hx, leg.hz);
  if (leg.length < kMinLegLength) return false;
  const double velocity_x =
      leg.jacobian[0][0] * dphi1 + leg.jacobian[0][1] * dphi2;
  const double velocity_z =
      leg.jacobian[1][0] * dphi1 + leg.jacobian[1][1] * dphi2;
  leg.length_rate =
      (leg.hx * velocity_x + leg.hz * velocity_z) / leg.length;
  leg.angle = std::atan2(leg.hx, -leg.hz);
  leg.angle_rate = (-leg.hz * velocity_x + leg.hx * velocity_z) /
                   (leg.length * leg.length);
  return true;
}

}  // namespace modules
