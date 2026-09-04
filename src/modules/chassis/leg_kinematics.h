/**
* @file src/modules/chassis/leg_kinematics.h
 * @ingroup wbr_modules
 * @brief 实现五连杆腿部的正逆运动学与雅可比计算。
 * @details 模块遵循 `ModuleBase` 生命周期：`Start()` 只负责一次性资源初始化和线程创建，`RunLoop()` 持有周期状态。跨线程数据通过 channels 层交换。
 */

#ifndef WBR_CONTROL_CORE_LEG_KINEMATICS_H_
#define WBR_CONTROL_CORE_LEG_KINEMATICS_H_

namespace modules
{

/** @brief 五连杆腿部末端位姿、速度和雅可比矩阵。 */
struct LegKinematics {
	double hx = 0.0; ///< 足端相对髋部的水平坐标，单位为米。
	double hz = 0.0; ///< 足端相对髋部的竖直坐标，单位为米。
	double length = 0.0; ///< 等效腿长，单位为米。
	double length_rate = 0.0; ///< 等效腿长变化率，单位为米每秒。
	double angle = 0.0; ///< 腿部或关节角度，单位为弧度。
	double angle_rate = 0.0; ///< 角速度，单位为弧度每秒。
	/** @brief 关节角速度到足端平面速度的 2×2 雅可比矩阵。 */
	double jacobian[2][2] = {};
};

/**
 * @brief 由两个关节角求解腿部末端位置。
 * @param phi1 第一主动关节角，单位为弧度。
 * @param phi2 第二主动关节角，单位为弧度。
 * @param branch 五连杆装配支路标识。
 * @param[out] hx 足端相对髋部的水平坐标。
 * @param[out] hz 足端相对髋部的竖直坐标。
 * @return 几何解有效时返回 `true`，否则返回 `false`。
 */
bool ForwardKinematics(double phi1, double phi2, int branch, double &hx, double &hz);
/**
 * @brief 由目标末端位置求解连续的关节角。
 *
 * 调用方应传入当前关节角作为迭代初值，使结果保持在当前物理装配支路。
 *
 * @param target_hx 目标足端水平坐标。
 * @param target_hz 目标足端竖直坐标。
 * @param branch 五连杆装配支路标识。
 * @param seed_phi1 第一主动关节角的迭代初值。
 * @param seed_phi2 第二主动关节角的迭代初值。
 * @param[out] phi1 求解得到的第一主动关节角。
 * @param[out] phi2 求解得到的第二主动关节角。
 * @return 迭代收敛且几何解有效时返回 `true`。
 */
bool InverseKinematics(double target_hx, double target_hz, int branch, double seed_phi1,
		       double seed_phi2, double &phi1, double &phi2);
/**
 * @brief 数值计算关节角到末端位置的雅可比矩阵。
 * @param phi1 第一主动关节角，单位为弧度。
 * @param phi2 第二主动关节角，单位为弧度。
 * @param branch 五连杆装配支路标识。
 * @param[out] jacobian 输出的 2×2 雅可比矩阵。
 * @return 矩阵计算成功时返回 `true`。
 */
bool NumericalJacobian(double phi1, double phi2, int branch, double jacobian[2][2]);
/**
 * @brief 计算腿部末端位置、速度和雅可比矩阵。
 * @param phi1 第一主动关节角，单位为弧度。
 * @param phi2 第二主动关节角，单位为弧度。
 * @param dphi1 第一主动关节角速度，单位为弧度每秒。
 * @param dphi2 第二主动关节角速度，单位为弧度每秒。
 * @param branch 五连杆装配支路标识。
 * @param[out] leg 输出的腿部运动学状态。
 * @return 全部运动学量有效时返回 `true`。
 */
bool ComputeLegKinematics(double phi1, double phi2, double dphi1, double dphi2, int branch,
			  LegKinematics &leg);
} // namespace modules

#endif // WBR_CONTROL_CORE_LEG_KINEMATICS_H_
