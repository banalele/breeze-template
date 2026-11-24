/**
 * @file motor.hpp
 * @brief 统一电机包: 面向应用层只暴露一种电机, 协议差异全部封装在 Motor 内部.
 *
 * @note 单位统一在输出轴, 角度制: 位置(度), 速度(度/秒), 力矩(N·m).
 *       GearRatio / EncoderCounts 从驱动 cfg(设备树)自动读取, 不需要手工填写.
 *       PID 参数只保存不执行, 闭环由应用层或电机内部(仅 DM MIT / LK)完成.
 *
 * @copyright Copyright (c) 2026 RobotPilots-SZU.
 * @license SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cassert>
#include <cstdint>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <drivers/bldcm/bldcm.h>
#include <drivers/bldcm/bldcm_lk.h>
#include <drivers/bldcm/bldcm_dm.h>

#include <ramen.hpp>

#include <robotpilots/algorithm/pid.hpp>

namespace breeze::device{

/**
 * @brief 电机类型
 * @note 使用 BREEZE_MOTOR_DT() 时由设备树 compatible 自动推导,
 *       不需要手写: rp,dji-can-motor -> Dji, rp,lk-can-motor -> Lk, rp,dm-can-motor -> Dm.
 */
enum class MotorType : std::uint8_t
{
	Dji,     ///< DJI/RM, 只支持力矩(电流)控制
	Lk,      ///< 瓴控 LK
	Dm,      ///< 达妙 DM
	Unknown, ///< compatible 不是三种电机之一
};

/**
 * @brief 由设备树 compatible 推导电机类型
 */
constexpr MotorType MotorTypeFromCompat(bool isDji, bool isLk, bool isDm)
{
	if (isDji)
	{
		return MotorType::Dji;
	}
	if (isLk)
	{
		return MotorType::Lk;
	}
	if (isDm)
	{
		return MotorType::Dm;
	}
	return MotorType::Unknown;
}

/**
 * @brief LK 控制模式
 * @note 默认使用单电机模式; 多电机模式仅作兼容, 受 tx-id 约束.
 */
enum class LkControlMode : std::uint8_t
{
	Single,
	Multi,
};

/**
 * @brief 必填的协议满量程输出
 * @note 没有默认构造, 因此在 MotorConversion 里漏写 .MaxOutput 会编译失败.
 *       值是协议原始码满量程对应的物理量: DJI 16384 / LK 单电机 2048 / LK 多电机 2000
 *       对应的电流(A); DM 为力矩(N·m).
 */
struct OutputScale
{
	float Value;

	OutputScale() = delete;
	constexpr OutputScale(float value) : Value(value) {}

	constexpr operator float() const { return Value; }
};

/**
 * @brief 需要人工标定的换算参数
 * @note GearRatio / EncoderCounts 从设备树自动读取; 满量程电流码由协议推导.
 */
struct MotorConversion
{
	OutputScale MaxOutput;  ///< 满量程输出(必填): DJI/LK 为电流(A), DM 为力矩(N·m)
	float Direction = 1.0f; ///< +1 / -1, 作用于指令与 DM 力矩反馈; 位置/速度反馈不乘方向
};

/**
 * @brief 电机配置
 */
struct MotorConfig
{
	const struct device *Dev = nullptr;
	float GearRatio = 0.0f;     ///< 减速比; 0 表示未填, 用 BREEZE_MOTOR_DT() 从设备树获取
	float EncoderCounts = 0.0f; ///< 电机轴每圈编码器计数; 0 表示未填, 用 BREEZE_MOTOR_DT() 从设备树获取
	MotorType Type = MotorType::Dji;
	LkControlMode LkMode = LkControlMode::Single;
	MotorConversion Conversion;

	/* DM MIT 直接取这三组参数的 Kp / Kd, 不额外定义增益 */
	robotpilots::algorithm::Pid<float>::Config SpeedPid{};            ///< 单速度环
	robotpilots::algorithm::Pid<float>::Config PositionSpeedPid{};    ///< 位置环内速度环
	robotpilots::algorithm::Pid<float>::Config PositionPid{};         ///< 位置环外环: 用 Kp / Kd

};

/**
 * @brief 电机反馈, 输出轴角度制
 * @note 掉线或驱动无数据时所有字段归 0, 只有 Online 表示状态.
 */
struct MotorFeedback
{
	float Position = 0.0f;     ///< 度, 输出轴单圈位置(绝对编码器值换算, 不含方向; DM 由驱动给出)
	float Speed = 0.0f;        ///< 度/秒, 输出轴, 不含方向
	float Torque = 0.0f;       ///< N·m, 仅 DM 直接给出; DJI/LK 用 Current × Kt 自行换算
	float Current = 0.0f;      ///< A
	float Temperature = 0.0f;  ///< °C
	std::uint16_t Encoder = 0; ///< 原始单圈编码器
	bool Online = false;
};

/**
 * @brief 统一电机
 * @note 含 ramen::Pullable, 捕获 this, 因此禁止拷贝与移动.
 */
class Motor
{
public:
	explicit Motor(const MotorConfig &config);
	~Motor() = default;

	Motor(const Motor &) = delete;
	Motor(Motor &&) = delete;
	Motor &operator=(const Motor &) = delete;
	Motor &operator=(Motor &&) = delete;

	/**
	 * @brief 设备就绪检查 + 注册 + 必要的使能
	 */
	int Init();

	int Enable();
	int Disable();
	int ClearError();

	/**
	 * @brief 刷新在线状态(不 pull 反馈时也可调用)
	 */
	int Heartbeat();

	/**
	 * @brief 输出指令, 不区分电流与力矩
	 * @param value DJI/LK: 协议原始码(DJI ±16384, LK 单电机 ±2048 / 多电机 ±2000); DM: 力矩(N·m)
	 */
	int SetOutput(float value);

	/**
	 * @brief 速度指令
	 * @param outputDegPerSec 输出轴速度, 度/秒
	 * @param limitRaw iq 限幅原始码; <=0 表示用满量程(LK 单电机 2048)
	 */
	int SetSpeed(float outputDegPerSec, float limitRaw = 0.0f);

	/**
	 * @brief 位置指令(多圈绝对位置)
	 * @param positionDeg 输出轴位置, 度
	 * @param maxSpeedDegPerSec 最大速度, 度/秒; <=0 表示不带限速(LK 多电机不支持限速)
	 */
	int SetPosition(float positionDeg, float maxSpeedDegPerSec = 0.0f);

	[[nodiscard]] const struct device *Device() const { return Config.Dev; }
	[[nodiscard]] float GearRatio() const { return Config.GearRatio; }
	[[nodiscard]] float EncoderCounts() const { return Config.EncoderCounts; }

	/**
	 * @brief LK 专有 API, 非 LK 设备返回 nullptr
	 */
	[[nodiscard]] const lk_special_api_t *Lk() const;

	/**
	 * @brief DM 专有 API, 非 DM 设备返回 nullptr
	 */
	[[nodiscard]] const dm_special_api_t *Dm() const;

	MotorConfig Config;

	/**
	 * @brief 反馈数据源, 被 pull 时直接从驱动取数
	 */
	ramen::Pullable<MotorFeedback> Feedback;

private:
	void ReadInto(MotorFeedback &out);

	[[nodiscard]] const motor_driver_api_t *Api() const;
};

/**
 * @brief 从设备树节点填 MotorConfig 的 DT 派生字段
 * @param node_id 设备树节点标识, 例如 DT_NODELABEL(chassis_lf)
 * @note 使用官方 devicetree API 展开为
 *       .Dev / .GearRatio(减速比) / .EncoderCounts / .Type 四个指定初始化项,
 *       其中 .Type 由 compatible 自动推导, 必须放在 MotorConfig 初始化列表最前面,
 *       其余字段按声明顺序继续写:
 *       @code
 *       Motor m{MotorConfig{
 *           BREEZE_MOTOR_DT(DT_NODELABEL(chassis_lf)),
 *           .Conversion = {.MaxOutput = 20.0f, .Direction = 1.0f},
 *       }};
 *       @endcode
 */
#define BREEZE_MOTOR_DT(node_id)                                                      \
	.Dev = DEVICE_DT_GET(node_id),                                                      \
	.GearRatio = static_cast<float>(DT_PROP(node_id, motor_transmission_ratio)),        \
	.EncoderCounts = static_cast<float>(DT_PROP(node_id, motor_encoder)),               \
	.Type = ::breeze::device::MotorTypeFromCompat(                                      \
	    DT_NODE_HAS_COMPAT(node_id, rp_dji_can_motor),                                   \
	    DT_NODE_HAS_COMPAT(node_id, rp_lk_can_motor),                                    \
	    DT_NODE_HAS_COMPAT(node_id, rp_dm_can_motor))


} // namespace breeze::device
