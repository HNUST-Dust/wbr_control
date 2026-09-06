/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <Eigen/Core>
#include <sophus/so3.hpp>

namespace wbr_control {

/**
 * @brief 机器人两套 IMU 使用的静态旋转森林。
 * @details 当前拓扑由两棵互不连通的树组成：
 *          `chassis_imu -> chassis` 与
 *          `gimbal_imu -> gimbal -> pc_link`。统一采用
 *          `R_target_from_source` 语义。所有矩阵均为固定尺寸，不进行动态
 *          内存分配、锁等待或运行时图搜索。
 */
class TfTree final {
public:
	/** 静态 TF 中的坐标系节点。 */
	enum class Frame {
		kChassisImu,
		kChassis,
		kGimbalImu,
		kGimbal,
		kPcLink,
	};

	using Rotation = Sophus::SO3f;
	using Vector3 = Eigen::Vector3f;
	using Matrix3 = Eigen::Matrix3f;

	/** 构造尚未配置任何 IMU 分支的 TF。 */
	TfTree() = default;

	/**
	 * @brief 配置底盘 IMU 到底盘坐标系的静态旋转。
	 * @param chassis_from_chassis_imu `R_chassis_from_chassis_imu`。
	 * @note 仅允许在线程启动前配置。
	 */
	void ConfigureChassisBranch(const Rotation &chassis_from_chassis_imu)
	{
		chassis_from_chassis_imu_ = chassis_from_chassis_imu;
		chassis_branch_configured_ = true;
	}

	/**
	 * @brief 预留配置云台 IMU、云台和 pc_link 坐标系三节点分支。
	 * @param gimbal_from_gimbal_imu `R_gimbal_from_gimbal_imu`。
	 * @param pc_link_from_gimbal `R_pc_link_from_gimbal`。
	 * @note 当前尚未接入云台 IMU，不应由底盘 AHRS 调用。
	 */
	void ConfigureGimbalBranch(const Rotation &gimbal_from_gimbal_imu,
				   const Rotation &pc_link_from_gimbal)
	{
		gimbal_from_gimbal_imu_ = gimbal_from_gimbal_imu;
		pc_link_from_gimbal_ = pc_link_from_gimbal;
		gimbal_branch_configured_ = true;
	}

	/**
	 * @brief 将三维向量从 source 转换到 target。
	 * @param target 目标坐标系。
	 * @param source 源坐标系。
	 * @param source_vector 源坐标系中表达的向量。
	 * @param target_vector 接收目标坐标系中表达的向量。
	 * @return 两节点位于同一条已配置分支时返回 true；跨分支或分支未配置时
	 *         返回 false，且不修改 target_vector。
	 */
	bool TransformVector(Frame target, Frame source,
			     const Vector3 &source_vector, Vector3 &target_vector) const
	{
		Matrix3 target_from_source;
		if (!RotationMatrixBetween(target, source, target_from_source)) {
			return false;
		}
		target_vector = target_from_source * source_vector;
		return true;
	}

	/**
	 * @brief 将完整姿态从 source 坐标约定重表达到 target 坐标约定。
	 * @param target 输出姿态采用的坐标系约定。
	 * @param source 输入姿态采用的坐标系约定。
	 * @param source_attitude 在 source 坐标约定下表达的姿态矩阵。
	 * @param target_attitude 接收目标坐标约定下的姿态矩阵。
	 * @return 转换路径可用时返回 true，否则返回 false 且不修改输出。
	 * @details 使用 `A * source_attitude * A.transpose()`，其中
	 *          `A = R_target_from_source`。单位姿态转换后仍为单位姿态，不会
	 *          引入固定安装角对应的虚假 yaw。
	 */
	bool TransformAttitude(Frame target, Frame source,
			       const Matrix3 &source_attitude, Matrix3 &target_attitude) const
	{
		Matrix3 target_from_source;
		if (!RotationMatrixBetween(target, source, target_from_source)) {
			return false;
		}
		target_attitude =
			target_from_source * source_attitude * target_from_source.transpose();
		return true;
	}

private:
	/** 查询两节点之间的旋转；不允许跨越两棵互不连通的树。 */
	bool RotationMatrixBetween(Frame target, Frame source,
				   Matrix3 &target_from_source) const
	{
		if (target == source) {
			target_from_source = Matrix3::Identity();
			return true;
		}

		if (chassis_branch_configured_) {
			if (target == Frame::kChassis && source == Frame::kChassisImu) {
				target_from_source = chassis_from_chassis_imu_.matrix();
				return true;
			}
			if (target == Frame::kChassisImu && source == Frame::kChassis) {
				target_from_source = chassis_from_chassis_imu_.matrix().transpose();
				return true;
			}
		}

		if (gimbal_branch_configured_) {
			if (target == Frame::kGimbal && source == Frame::kGimbalImu) {
				target_from_source = gimbal_from_gimbal_imu_.matrix();
				return true;
			}
			if (target == Frame::kGimbalImu && source == Frame::kGimbal) {
				target_from_source = gimbal_from_gimbal_imu_.matrix().transpose();
				return true;
			}
			if (target == Frame::kPcLink && source == Frame::kGimbal) {
				target_from_source = pc_link_from_gimbal_.matrix();
				return true;
			}
			if (target == Frame::kGimbal && source == Frame::kPcLink) {
				target_from_source = pc_link_from_gimbal_.matrix().transpose();
				return true;
			}
			if (target == Frame::kPcLink && source == Frame::kGimbalImu) {
				target_from_source = pc_link_from_gimbal_.matrix() *
						     gimbal_from_gimbal_imu_.matrix();
				return true;
			}
			if (target == Frame::kGimbalImu && source == Frame::kPcLink) {
				target_from_source = gimbal_from_gimbal_imu_.matrix().transpose() *
						     pc_link_from_gimbal_.matrix().transpose();
				return true;
			}
		}

		return false;
	}

	Rotation chassis_from_chassis_imu_;
	Rotation gimbal_from_gimbal_imu_;
	Rotation pc_link_from_gimbal_;
	bool chassis_branch_configured_ = false;
	bool gimbal_branch_configured_ = false;
};

/**
 * @brief 全机唯一的静态 TF 实例。
 * @note 各 IMU 模块必须在其周期线程启动前配置对应分支；运行期间只读。
 */
inline TfTree robot_tf_tree;

}  // namespace wbr_control
