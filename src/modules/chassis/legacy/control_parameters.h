/**
* @file src/modules/chassis/legacy/control_parameters.h
 * @ingroup wbr_modules
 * @brief 定义传统底盘控制器使用的参数集合。
 * @details 该组件属于可独立计算的传统底盘控制链路。调用方按固定周期提供一致输入快照；组件保存的积分或滤波状态必须在失能、故障或时间跳变后复位。
 */

// Legacy controller stack; excluded from the firmware build.
#ifndef WBR_CONTROL_CORE_CONTROL_PARAMETERS_H_
#define WBR_CONTROL_CORE_CONTROL_PARAMETERS_H_

namespace wbr::v2 {

inline constexpr double kPi = 3.14159265358979323846; ///< 圆周率。
inline constexpr double kTwoPi = 2.0 * kPi; ///< 两倍圆周率。

inline constexpr double kLengthAB = 0.0945; ///< `kLengthAB` 几何长度参数，单位为米。
inline constexpr double kLengthBC = 0.1125; ///< `kLengthBC` 几何长度参数，单位为米。
inline constexpr double kLengthCD = 0.116; ///< `kLengthCD` 几何长度参数，单位为米。
inline constexpr double kLengthAD = 0.090; ///< `kLengthAD` 几何长度参数，单位为米。
inline constexpr double kLengthAG = 0.210; ///< `kLengthAG` 几何长度参数，单位为米。
inline constexpr double kLengthGH = 0.250; ///< `kLengthGH` 几何长度参数，单位为米。
inline constexpr double kTargetHRadius = 0.30347; ///< `kTargetHRadius` 几何长度参数，单位为米。
inline constexpr double kJacobianStep = 1e-6; ///< 数值雅可比中央差分使用的关节角步长，单位为弧度。
inline constexpr double kMinLegLength = 1e-4; ///< `kMinLegLength` 几何长度参数，单位为米。
inline constexpr double kMinTargetLegLength = 0.15133; ///< `kMinTargetLegLength` 几何长度参数，单位为米。
inline constexpr double kMaxTargetLegAngle = 0.6; ///< `kMaxTargetLegAngle` 角度参数，单位为弧度。

inline constexpr double kLegLengthKp = 600.0; ///< 腿长闭环比例增益。
inline constexpr double kLegLengthKi = 100.0; ///< 腿长闭环积分增益；旧调参值为 400。
inline constexpr double kLegLengthKd = 120.0; ///< 腿长闭环微分增益；旧调参值为 20。
inline constexpr double kLegRetractFeedforward = 10.0; ///< `kLegRetractFeedforward` 前馈控制系数。
inline constexpr double kLegRetractFeedforwardDeadband = 0.003; ///< `kLegRetractFeedforwardDeadband` 状态切换或有效性判定阈值。
inline constexpr double kLegExtendVelocityFeedforward = 60.0; ///< `kLegExtendVelocityFeedforward` 前馈控制系数。
inline constexpr double kLegVelocityFeedforwardDeadband = 0.005; ///< `kLegVelocityFeedforwardDeadband` 状态切换或有效性判定阈值。
inline constexpr double kLegIntegralForceLimit = 60.0; ///< `kLegIntegralForceLimit` 力限制或控制量，单位为牛。
inline constexpr double kLegForceLimit = 150.0; ///< `kLegForceLimit` 力限制或控制量，单位为牛。
inline constexpr double kLegAngleKp = 10.0; ///< 腿角闭环比例增益。
inline constexpr double kLegAngleKd = 1.0; ///< 腿角闭环微分增益。
inline constexpr double kLegAngleTorqueLimit = 8.0; ///< 腿角控制力矩上限，单位为牛·米。
inline constexpr double kIndependentLegAngleHoldKp = 6.0; ///< 单腿角保持控制比例增益。
inline constexpr double kIndependentLegAngleHoldKd = 0.6; ///< 单腿角保持控制微分增益。
inline constexpr double kIndependentLegAngleHoldTorqueLimit = 5.0; ///< 单腿角保持力矩上限，单位为牛·米。
inline constexpr double kWorldLegAngleKp = 12.0; ///< 世界坐标系腿角控制比例增益。
inline constexpr double kWorldLegAngleKd = 2.0; ///< 世界坐标系腿角控制微分增益。
inline constexpr double kWorldLegAngleTorqueLimit = 8.0; ///< 世界坐标系腿角控制力矩上限，单位为牛·米。
inline constexpr double kLegSplitHoldKp = 6.0; ///< `kLegSplitHoldKp` 比例增益。
inline constexpr double kLegSplitHoldKd = 0.6; ///< `kLegSplitHoldKd` 微分或速度反馈增益。
inline constexpr double kLegSplitHoldTorqueLimit = 4.0; ///< `kLegSplitHoldTorqueLimit` 力矩限制或控制量，单位为牛·米。
inline constexpr double kJointTorqueLimit = 20.0; ///< `kJointTorqueLimit` 力矩限制或控制量，单位为牛·米。
inline constexpr double kLegSpeedFilter = 0.2; ///< `kLegSpeedFilter` 低通滤波或平滑系数。
inline constexpr double kStateSpeedFilter = 0.2; ///< `kStateSpeedFilter` 低通滤波或平滑系数。
inline constexpr double kPitchRateFilterTimeConstant = 0.02; ///< 俯仰角速度低通滤波时间常数，单位为秒。
inline constexpr double kPitchRateControlLimit = 1.5; ///< `kPitchRateControlLimit` 角速度参数，单位为弧度每秒。
inline constexpr double kTargetLengthSlewRate = 0.15; ///< 目标腿长最大变化率，单位为米每秒。
inline constexpr double kTargetAngleSlewRate = 0.5; ///< 目标腿角最大变化率，单位为弧度每秒。
inline constexpr double kSupportFilterTimeConstant = 0.05; ///< `kSupportFilterTimeConstant` 时间常数，单位为秒。
inline constexpr double kTotalWheelTorqueLimit = 10.0; ///< `kTotalWheelTorqueLimit` 力矩限制或控制量，单位为牛·米。
inline constexpr double kTotalLegAngleTorqueLimit = 20.0; ///< 双腿腿角控制总力矩上限，单位为牛·米。

inline constexpr double kRollForceKp = 67.0; ///< `kRollForceKp` 力限制或控制量，单位为牛。
inline constexpr double kRollForceKd = 30.0; ///< `kRollForceKd` 力限制或控制量，单位为牛。
inline constexpr double kRollDifferentialForceLimit = 40.0; ///< `kRollDifferentialForceLimit` 力限制或控制量，单位为牛。

inline constexpr double kYawRateKp = 4.0; ///< `kYawRateKp` 角速度参数，单位为弧度每秒。
inline constexpr double kYawAccelerationKd = 0.05; ///< `kYawAccelerationKd` 微分或速度反馈增益。
inline constexpr double kDecoupledYawWorkingPointGain = 30.0; ///< `kDecoupledYawWorkingPointGain` 控制或模型缩放系数。
inline constexpr double kDecoupledYawWheelInputScale =
	0.028222 * kDecoupledYawWorkingPointGain; ///< 解耦偏航控制命令到车轮力矩的缩放系数。
inline constexpr double kDecoupledYawLegTorquePerCommand =
	0.160481 * kDecoupledYawWorkingPointGain; ///< 单位解耦偏航命令对应的腿部力矩。
inline constexpr double kTotalYawTorqueLimit = 4.0; ///< `kTotalYawTorqueLimit` 力矩限制或控制量，单位为牛·米。
inline constexpr double kYawTorqueRiseRate = 1.5; ///< `kYawTorqueRiseRate` 力矩限制或控制量，单位为牛·米。
inline constexpr double kYawTorqueBrakeRate = 20.0; ///< `kYawTorqueBrakeRate` 力矩限制或控制量，单位为牛·米。
inline constexpr double kPerWheelTorqueLimit = 5.0; ///< `kPerWheelTorqueLimit` 力矩限制或控制量，单位为牛·米。
inline constexpr double kYawRateFilterTimeConstant = 0.02; ///< 偏航角速度低通滤波时间常数，单位为秒。
inline constexpr double kYawAccelerationFilterTimeConstant = 0.04; ///< `kYawAccelerationFilterTimeConstant` 时间常数，单位为秒。

inline constexpr double kChassisVelocityCorrectionTimeConstant = 0.02; ///< `kChassisVelocityCorrectionTimeConstant` 时间常数，单位为秒。
inline constexpr double kContactGraceTime = 0.08; ///< 接地信号短时异常仍维持支撑判定的宽限时间，单位为秒。
inline constexpr double kContactLossDebounceTime = 0.050; ///< 确认失去接地前的去抖时间，单位为秒。
inline constexpr double kContactRecoveryDebounceTime = 0.020; ///< 确认重新接地前的去抖时间，单位为秒。
inline constexpr double kContactRecoveryRampTime = 0.15; ///< 接地恢复后控制输出由零爬升至全量的时间，单位为秒。
inline constexpr double kSingleSupportLqrScale = 0.0; ///< `kSingleSupportLqrScale` 控制或模型缩放系数。
inline constexpr double kRecoveryMaxRoll = 0.12; ///< 允许进入接地恢复流程的最大横滚角，单位为弧度。
inline constexpr double kRecoveryMaxPitch = 0.25; ///< 允许进入接地恢复流程的最大俯仰角，单位为弧度。
inline constexpr double kAirborneLegSearchExtension = 0.025; ///< 腾空腿用于搜索地面的附加伸长量，单位为米。
inline constexpr double kGroundedLegYield = 0.010; ///< 单侧接地时支撑腿主动让出的腿长，单位为米。
inline constexpr double kContactLegOffsetSlewRate = 0.08; ///< `kContactLegOffsetSlewRate` 目标值的最大变化率。
inline constexpr double kWheelOdometryCorrectionTimeConstant = 0.08; ///< `kWheelOdometryCorrectionTimeConstant` 时间常数，单位为秒。
inline constexpr double kWheelSlipSoftSpeed = 0.12; ///< 开始降低轮速里程计权重的滑移速度阈值，单位为米每秒。
inline constexpr double kWheelSlipHardSpeed = 0.50; ///< 完全拒绝轮速里程计的滑移速度阈值，单位为米每秒。

inline constexpr double kEmergencyLinearDeceleration = 1.5; ///< 紧急制动时允许的最大线减速度，单位为米每二次方秒。
inline constexpr double kMaxLinearVelocity = 1.5; ///< `kMaxLinearVelocity` 输出或状态的安全上限。
inline constexpr double kSustainedYawRateLimit = 13.0; ///< `kSustainedYawRateLimit` 角速度参数，单位为弧度每秒。
inline constexpr double kLinearAccelerationLimit = 1.5; ///< `kLinearAccelerationLimit` 输出或状态的安全上限。
inline constexpr double kYawCommandAcceleration = 3.0; ///< 偏航角速度命令的最大角加速度，单位为弧度每二次方秒。
inline constexpr double kMaxPositionTrackingError = 0.5; ///< `kMaxPositionTrackingError` 输出或状态的安全上限。

inline constexpr double kDifferentialLegAngleKp = 16.0; ///< `kDifferentialLegAngleKp` 角度参数，单位为弧度。
inline constexpr double kDifferentialLegAngleKd = 1.5; ///< `kDifferentialLegAngleKd` 角度参数，单位为弧度。
inline constexpr double kDifferentialLegAngleTorqueLimit = 20.0; ///< `kDifferentialLegAngleTorqueLimit` 角度参数，单位为弧度。
inline constexpr double kDifferentialLegAngleTorqueSlewRate = 120.0; ///< `kDifferentialLegAngleTorqueSlewRate` 力矩限制或控制量，单位为牛·米。
inline constexpr double kYawSplitReferencePerRate = -0.60; ///< 单位偏航角速度对应的左右腿角差参考系数。
inline constexpr double kMaxYawSplitReference = 0.14; ///< `kMaxYawSplitReference` 输出或状态的安全上限。
inline constexpr double kYawSplitReferenceSlewRate = 0.6; ///< `kYawSplitReferenceSlewRate` 目标值的最大变化率。
inline constexpr double kAbsoluteLegSplitSoftAngle = 0.24; ///< `kAbsoluteLegSplitSoftAngle` 角度参数，单位为弧度。
inline constexpr double kAbsoluteLegSplitHardAngle = 0.35; ///< `kAbsoluteLegSplitHardAngle` 角度参数，单位为弧度。
inline constexpr double kAbsoluteLegSplitSoftRate = 1.0; ///< 开始软限制的左右腿角差变化率，单位为弧度每秒。
inline constexpr double kAbsoluteLegSplitHardRate = 2.5; ///< 触发硬限制的左右腿角差变化率，单位为弧度每秒。

inline constexpr double kYawPitchSoftLimit = 0.12; ///< `kYawPitchSoftLimit` 输出或状态的安全上限。
inline constexpr double kYawPitchHardLimit = 0.35; ///< `kYawPitchHardLimit` 输出或状态的安全上限。
inline constexpr double kYawContactForceHard = 3.0; ///< `kYawContactForceHard` 力限制或控制量，单位为牛。
inline constexpr double kYawContactForceSoft = 15.0; ///< `kYawContactForceSoft` 力限制或控制量，单位为牛。
inline constexpr double kYawLoadRatioHard = 0.15; ///< 偏航控制硬降额使用的最小支撑载荷比例。
inline constexpr double kYawLoadRatioSoft = 0.45; ///< 偏航控制开始软降额的支撑载荷比例。
inline constexpr double kSpinModeEntryYawRate = 0.5; ///< `kSpinModeEntryYawRate` 角速度参数，单位为弧度每秒。
inline constexpr double kSpinModeFullYawRate = 2.0; ///< `kSpinModeFullYawRate` 角速度参数，单位为弧度每秒。
inline constexpr double kSpinYawReservePerWheel = 2.0; ///< 小陀螺模式为每个车轮预留的偏航力矩，单位为牛·米。
inline constexpr double kSpinYawReserveBufferPerWheel = 0.15; ///< 每轮偏航力矩预留之外的附加安全余量，单位为牛·米。
inline constexpr double kYawPredictionHorizon = 0.15; ///< 偏航安全协调器向前预测的时间范围，单位为秒。
inline constexpr double kYawPredictedSplitSoft = 0.16; ///< 预测腿角差开始触发偏航软降额的阈值，单位为弧度。
inline constexpr double kYawPredictedSplitHard = 0.30; ///< 预测腿角差触发偏航硬限制的阈值，单位为弧度。
inline constexpr double kYawPredictedRollSoft = 0.06; ///< 预测横滚角开始触发偏航软降额的阈值，单位为弧度。
inline constexpr double kYawPredictedRollHard = 0.16; ///< 预测横滚角触发偏航硬限制的阈值，单位为弧度。
inline constexpr double kYawForceFilterTimeConstant = 0.025; ///< `kYawForceFilterTimeConstant` 力限制或控制量，单位为牛。
inline constexpr double kYawForceRateFilterTimeConstant = 0.08; ///< `kYawForceRateFilterTimeConstant` 力限制或控制量，单位为牛。
inline constexpr double kYawCoordinatorDeceleration = 3.0; ///< 安全协调器收回偏航角速度命令的最大角减速度，单位为弧度每二次方秒。

}  // namespace wbr::v2

#endif  // WBR_CONTROL_CORE_CONTROL_PARAMETERS_H_
