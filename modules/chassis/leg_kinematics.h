#ifndef WBR_CONTROL_CORE_LEG_KINEMATICS_H_
#define WBR_CONTROL_CORE_LEG_KINEMATICS_H_

namespace wbr::v2 {

struct LegKinematics {
  double hx = 0.0;
  double hz = 0.0;
  double length = 0.0;
  double length_rate = 0.0;
  double angle = 0.0;
  double angle_rate = 0.0;
  double jacobian[2][2] = {};
};

struct LegVmcOutput {
  double axial_force = 0.0;
  double joint_torque[2] = {};  // phi1, phi2
};

bool ForwardKinematics(double phi1, double phi2, int branch,
                       double& hx, double& hz);
bool NumericalJacobian(double phi1, double phi2, int branch,
                       double jacobian[2][2]);
bool ComputeLegKinematics(double phi1, double phi2,
                          double dphi1, double dphi2, int branch,
                          LegKinematics& leg);
LegVmcOutput ComputeLegVmc(const LegKinematics& leg,
                           double target_leg_length,
                           double support_feedforward,
                           double integral_force,
                           double leg_angle_torque,
                           double filtered_leg_speed,
                           double target_leg_length_rate);

}  // namespace wbr::v2

#endif  // WBR_CONTROL_CORE_LEG_KINEMATICS_H_
