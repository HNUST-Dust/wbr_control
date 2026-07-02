#include "leg_kinematics.h"

#include <cmath>

#include "control_parameters.h"
#include "math_utils.h"

namespace wbr::v2 {

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

LegVmcOutput ComputeLegVmc(const LegKinematics& leg,
                           double target_leg_length,
                           double support_feedforward,
                           double integral_force,
                           double leg_angle_torque,
                           double filtered_leg_speed,
                           double target_leg_length_rate) {
  LegVmcOutput output;
  const double length_error = target_leg_length - leg.length;
  double retract_feedforward = 0.0;
  if (length_error < -kLegRetractFeedforwardDeadband) {
    retract_feedforward = -kLegRetractFeedforward;
  }
  double velocity_feedforward = 0.0;
  if (target_leg_length_rate > kLegVelocityFeedforwardDeadband) {
    velocity_feedforward =
        kLegExtendVelocityFeedforward * target_leg_length_rate;
  }
  output.axial_force = Clamp(
      kLegLengthKp * length_error -
          kLegLengthKd * filtered_leg_speed + support_feedforward +
          integral_force + retract_feedforward + velocity_feedforward,
      -kLegForceLimit, kLegForceLimit);
  const double radial_x = leg.hx / leg.length;
  const double radial_z = leg.hz / leg.length;
  const double tangential_force = leg_angle_torque / leg.length;
  const double force_x =
      output.axial_force * radial_x - tangential_force * radial_z;
  const double force_z =
      output.axial_force * radial_z + tangential_force * radial_x;
  output.joint_torque[0] =
      leg.jacobian[0][0] * force_x + leg.jacobian[1][0] * force_z;
  output.joint_torque[1] =
      leg.jacobian[0][1] * force_x + leg.jacobian[1][1] * force_z;
  return output;
}

}  // namespace wbr::v2
