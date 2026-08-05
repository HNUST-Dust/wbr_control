#ifndef WBR_CONTROL_CORE_LEG_KINEMATICS_H_
#define WBR_CONTROL_CORE_LEG_KINEMATICS_H_

namespace modules
{

struct LegKinematics {
	double hx = 0.0;
	double hz = 0.0;
	double length = 0.0;
	double length_rate = 0.0;
	double angle = 0.0;
	double angle_rate = 0.0;
	double jacobian[2][2] = {};
};

bool ForwardKinematics(double phi1, double phi2, int branch, double &hx, double &hz);
/*
 * 求解与正运动学相同的五连杆几何。调用方以当前关节角作为初值，
 * 确保返回解保持在腿部当前所在的物理装配支路。
 */
bool InverseKinematics(double target_hx, double target_hz, int branch, double seed_phi1,
		       double seed_phi2, double &phi1, double &phi2);
bool NumericalJacobian(double phi1, double phi2, int branch, double jacobian[2][2]);
bool ComputeLegKinematics(double phi1, double phi2, double dphi1, double dphi2, int branch,
			  LegKinematics &leg);
} // namespace modules

#endif // WBR_CONTROL_CORE_LEG_KINEMATICS_H_
