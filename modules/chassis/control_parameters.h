#ifndef WBR_CONTROL_CORE_CONTROL_PARAMETERS_H_
#define WBR_CONTROL_CORE_CONTROL_PARAMETERS_H_

namespace wbr::v2 {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kTwoPi = 2.0 * kPi;

inline constexpr double kLengthAB = 0.0945;
inline constexpr double kLengthBC = 0.1125;
inline constexpr double kLengthCD = 0.116;
inline constexpr double kLengthAD = 0.090;
inline constexpr double kLengthAG = 0.210;
inline constexpr double kLengthGH = 0.250;
inline constexpr double kTargetHRadius = 0.34;
inline constexpr double kJacobianStep = 1e-6;
inline constexpr double kMinLegLength = 1e-4;
inline constexpr double kMinTargetLegLength = 0.15;
inline constexpr double kMaxTargetLegAngle = 0.6;

inline constexpr double kLegLengthKp = 300.0;
inline constexpr double kLegLengthKi = 400.0;//400
inline constexpr double kLegLengthKd = 10.0;//20
inline constexpr double kLegRetractFeedforward = 10.0;
inline constexpr double kLegRetractFeedforwardDeadband = 0.003;
inline constexpr double kLegExtendVelocityFeedforward = 60.0;
inline constexpr double kLegVelocityFeedforwardDeadband = 0.005;
inline constexpr double kLegIntegralForceLimit = 60.0;
inline constexpr double kLegForceLimit = 150.0;
inline constexpr double kLegAngleKp = 10.0;
inline constexpr double kLegAngleKd = 1.0;
inline constexpr double kLegAngleTorqueLimit = 8.0;
inline constexpr double kJointTorqueLimit = 20.0;
inline constexpr double kLegSpeedFilter = 0.2;
inline constexpr double kStateSpeedFilter = 0.2;
inline constexpr double kTargetLengthSlewRate = 0.15;
inline constexpr double kTargetAngleSlewRate = 0.5;
inline constexpr double kSupportFilterTimeConstant = 0.05;
inline constexpr double kTotalWheelTorqueLimit = 10.0;
inline constexpr double kTotalLegAngleTorqueLimit = 20.0;

inline constexpr double kRollForceKp = 67.0;
inline constexpr double kRollForceKd = 30.0;
inline constexpr double kRollDifferentialForceLimit = 40.0;

inline constexpr double kYawRateKp = 4.0;
inline constexpr double kYawAccelerationKd = 0.05;
inline constexpr double kDecoupledYawWorkingPointGain = 30.0;
inline constexpr double kDecoupledYawWheelInputScale =
    0.028222 * kDecoupledYawWorkingPointGain;
inline constexpr double kDecoupledYawLegTorquePerCommand =
    0.160481 * kDecoupledYawWorkingPointGain;
inline constexpr double kTotalYawTorqueLimit = 4.0;
inline constexpr double kYawTorqueRiseRate = 1.5;
inline constexpr double kYawTorqueBrakeRate = 20.0;
inline constexpr double kPerWheelTorqueLimit = 5.0;
inline constexpr double kYawRateFilterTimeConstant = 0.02;
inline constexpr double kYawAccelerationFilterTimeConstant = 0.04;

inline constexpr double kChassisVelocityCorrectionTimeConstant = 0.02;
inline constexpr double kContactGraceTime = 0.08;
inline constexpr double kContactLossDebounceTime = 0.050;
inline constexpr double kContactRecoveryDebounceTime = 0.020;
inline constexpr double kContactRecoveryRampTime = 0.15;
inline constexpr double kSingleSupportLqrScale = 0.0;
inline constexpr double kRecoveryMaxRoll = 0.12;
inline constexpr double kRecoveryMaxPitch = 0.25;
inline constexpr double kAirborneLegSearchExtension = 0.025;
inline constexpr double kGroundedLegYield = 0.010;
inline constexpr double kContactLegOffsetSlewRate = 0.08;
inline constexpr double kWheelOdometryCorrectionTimeConstant = 0.08;
inline constexpr double kWheelSlipSoftSpeed = 0.12;
inline constexpr double kWheelSlipHardSpeed = 0.50;

inline constexpr double kEmergencyLinearDeceleration = 1.5;
inline constexpr double kMaxLinearVelocity = 1.5;
inline constexpr double kSustainedYawRateLimit = 13.0;
inline constexpr double kLinearAccelerationLimit = 1.5;
inline constexpr double kYawCommandAcceleration = 3.0;
inline constexpr double kMaxPositionTrackingError = 0.5;

inline constexpr double kDifferentialLegAngleKp = 16.0;
inline constexpr double kDifferentialLegAngleKd = 1.5;
inline constexpr double kDifferentialLegAngleTorqueLimit = 20.0;
inline constexpr double kDifferentialLegAngleTorqueSlewRate = 120.0;
inline constexpr double kYawSplitReferencePerRate = -0.60;
inline constexpr double kMaxYawSplitReference = 0.14;
inline constexpr double kYawSplitReferenceSlewRate = 0.6;
inline constexpr double kAbsoluteLegSplitSoftAngle = 0.24;
inline constexpr double kAbsoluteLegSplitHardAngle = 0.35;
inline constexpr double kAbsoluteLegSplitSoftRate = 1.0;
inline constexpr double kAbsoluteLegSplitHardRate = 2.5;

inline constexpr double kYawPitchSoftLimit = 0.12;
inline constexpr double kYawPitchHardLimit = 0.35;
inline constexpr double kYawContactForceHard = 3.0;
inline constexpr double kYawContactForceSoft = 15.0;
inline constexpr double kYawLoadRatioHard = 0.15;
inline constexpr double kYawLoadRatioSoft = 0.45;
inline constexpr double kSpinModeEntryYawRate = 0.5;
inline constexpr double kSpinModeFullYawRate = 2.0;
inline constexpr double kSpinYawReservePerWheel = 2.0;
inline constexpr double kSpinYawReserveBufferPerWheel = 0.15;
inline constexpr double kYawPredictionHorizon = 0.15;
inline constexpr double kYawPredictedSplitSoft = 0.16;
inline constexpr double kYawPredictedSplitHard = 0.30;
inline constexpr double kYawPredictedRollSoft = 0.06;
inline constexpr double kYawPredictedRollHard = 0.16;
inline constexpr double kYawForceFilterTimeConstant = 0.025;
inline constexpr double kYawForceRateFilterTimeConstant = 0.08;
inline constexpr double kYawCoordinatorDeceleration = 3.0;

}  // namespace wbr::v2

#endif  // WBR_CONTROL_CORE_CONTROL_PARAMETERS_H_
