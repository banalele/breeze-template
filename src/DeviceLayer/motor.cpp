#include "motor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(motor, LOG_LEVEL_INF);

namespace breeze::device{
namespace
{

constexpr float kPi = 3.14159265358979323846f;

/* 绝对编码器值 -> 输出轴单圈角度(度) */
constexpr float EncoderToDegrees(std::int32_t encoder, float countsPerRev, float gearRatio)
{
	return static_cast<float>(encoder) / countsPerRev * 360.0f / gearRatio;
}

} // namespace

Motor::Motor(const MotorConfig &config)
    : Config(config),
      Feedback([this](MotorFeedback &out) { ReadInto(out); })
{
}

/**
 * @brief 取 LK 专有 API, 仅 LK 设备有效
 */
const lk_special_api_t *Motor::Lk() const
{
	if (Config.Type == MotorType::Lk)
	{
		return static_cast<const motor_driver_api_t *>(Config.Dev->api)->lk_api;
	}
	return nullptr;
}

/**
 * @brief 取 DM 专有 API, 仅 DM 设备有效
 */
const dm_special_api_t *Motor::Dm() const
{
	if (Config.Type == MotorType::Dm)
	{
		return static_cast<const motor_driver_api_t *>(Config.Dev->api)->dm_api;
	}
	return nullptr;
}

/**
 * @brief 设备就绪 -> 注册 -> 必要使能
 */
int Motor::Init()
{
	int ret = -EINVAL;

	if (Config.Dev != nullptr)
	{
		if (Config.Type == MotorType::Unknown)
		{
			LOG_ERR("motor %s: unsupported compatible, check dts", Config.Dev->name);
			ret = -ENOTSUP;
		}
		else
		{
			/* 未用 BREEZE_MOTOR_DT() 时补默认值, 避免反馈除法除零 */
			if (Config.GearRatio <= 0.0f)
			{
				LOG_WRN("motor %s: gear ratio missing, fallback 1.0", Config.Dev->name);
				Config.GearRatio = 1.0f;
			}
			if (Config.EncoderCounts <= 0.0f)
			{
				LOG_WRN("motor %s: encoder counts missing, fallback 8192", Config.Dev->name);
				Config.EncoderCounts = 8192.0f;
			}

			if (device_is_ready(Config.Dev))
			{
				ret = register_motor(Config.Dev);
				if ((ret == 0) || (ret == -EALREADY))
				{
					/* DM 上电默认失能; LK 单电机需要 0x88 使能后才响应 0xA1 */
					if ((Config.Type == MotorType::Dm) ||
					    ((Config.Type == MotorType::Lk) && (Config.LkMode == LkControlMode::Single)))
					{
						(void)motor_enable(Config.Dev);
					}
					ret = 0;
				}
				else
				{
					LOG_ERR("motor register failed: %d", ret);
				}
			}
			else
			{
				ret = -ENODEV;
			}
		}
	}

	return ret;
}

int Motor::Enable()
{
	return motor_enable(Config.Dev);
}

int Motor::Disable()
{
	return motor_disable(Config.Dev);
}

int Motor::ClearError()
{
	return motor_clear_error(Config.Dev);
}

int Motor::Heartbeat()
{
	return get_motor_heartbeat_status(Config.Dev);
}

/**
 * @brief 统一输出指令: DJI/LK 为协议原始码, DM 为力矩(N·m)
 */
int Motor::SetOutput(float value)
{
	int ret = -ENOTSUP; // 未知协议时的默认返回值

	if (Config.Type == MotorType::Dji)
	{
		value = std::clamp(value, -Config.Conversion.MaxOutput.Value, Config.Conversion.MaxOutput.Value);
		ret = motor_torque_control(Config.Dev, static_cast<std::int16_t>(value));
	}
	else if (Config.Type == MotorType::Lk)
	{
		const auto raw = static_cast<std::int16_t>(value);
		if (Config.LkMode == LkControlMode::Single)
		{
			/* 单电机模式: 0xA1 闭环力矩, 不能用通用 0x280 */
			value = std::clamp(value, -Config.Conversion.MaxOutput.Value, Config.Conversion.MaxOutput.Value);
			ret = Lk()->single_closedloop_control(Config.Dev, raw);
		}
		else
		{
			/* 多电机模式: 广播帧 0x280 */
			value = std::clamp(value, -Config.Conversion.MaxOutput.Value, Config.Conversion.MaxOutput.Value);
			ret = motor_torque_control(Config.Dev, raw);
		}
	}
	else if (Config.Type == MotorType::Dm)
	{
		/* DM 只有 MIT: Kp=Kd=0 即纯力矩 */
		const float maxOutput = Config.Conversion.MaxOutput.Value;
		float torque = value;
		if (maxOutput > 0.0f)
		{
			torque = std::clamp(torque, -maxOutput, maxOutput);
		}
		ret = Dm()->dm_mit_control(Config.Dev, 0.0f, 0.0f, 0.0f, 0.0f, torque);
	}

	return ret;
}

/**
 * @brief 速度指令(输出轴 度/秒)
 * @note DM 用 MIT 的 Kd 做阻尼; LK 用电机内部速度环; DJI 没有速度环
 */
int Motor::SetSpeed(float outputDegPerSec, float limitRaw)
{
	int ret = -ENOTSUP; // 未知协议或 DJI 时的默认返回值
	const float dir = Config.Conversion.Direction;

	if (Config.Type == MotorType::Dm)
	{
		/* MIT 速度环: Kp 必须为 0, 只用 Kd */
		const float radPerSec = outputDegPerSec * dir * (kPi / 180.0f);
		ret = Dm()->dm_mit_control(Config.Dev, 0.0f, radPerSec, 0.0f, Config.SpeedPid.Kd, 0.0f);
	}
	else if (Config.Type == MotorType::Lk)
	{
		const float dps = outputDegPerSec * dir;
		if (Config.LkMode == LkControlMode::Single)
		{
			/* 第二参数是 iq 限幅原始码; limitRaw <= 0 时给满量程 */
			std::int16_t iq = 2048;
			if (limitRaw > 0.0f)
			{
				iq = static_cast<std::int16_t>(limitRaw);
			}
			ret = Lk()->single_speedcontrol(Config.Dev, dps, iq);
		}
		else
		{
			ret = Lk()->multi_speedcontrol(Config.Dev, dps);
		}
	}

	return ret;
}

/**
 * @brief 位置指令(输出轴 度, 多圈绝对)
 * @note 对外入参统一为"度"; 减速比由驱动内部乘, 这里只做单位换算, 不能再乘 GearRatio:
 *       DM 转 rad, LK 多电机按驱动要求转 0.01 度, LK 单电机直接传度.
 */
int Motor::SetPosition(float positionDeg, float maxSpeedDegPerSec)
{
	int ret = -ENOTSUP; // 未知协议时的默认返回值
	const float dir = Config.Conversion.Direction;
	const float deg = positionDeg * dir;

	if (Config.Type == MotorType::Dm)
	{
		/* MIT 位置环: 增益取 PositionPid 的 Kp / Kd; 驱动要 rad, 在这里把度换算成 rad */
		const float rad = deg * (kPi / 180.0f);
		ret = Dm()->dm_mit_control(Config.Dev, rad, 0.0f, Config.PositionPid.Kp, Config.PositionPid.Kd, 0.0f);
	}
	else if (Config.Type == MotorType::Lk)
	{
		if (Config.LkMode == LkControlMode::Single)
		{
			if (maxSpeedDegPerSec > 0.0f)
			{
				ret = Lk()->single_mulposctrl2(Config.Dev, deg, maxSpeedDegPerSec);
			}
			else
			{
				ret = Lk()->single_mulposctrl1(Config.Dev, deg);
			}
		}
		else
		{
			/* 多电机只有多圈位置且不支持限速; 驱动要 0.01 度/LSB, 在这里把度换算成 0.01 度 */
			if (maxSpeedDegPerSec > 0.0f)
			{
				ret = -ENOTSUP;
			}
			else
			{
				ret = Lk()->multi_positcontrol(Config.Dev, deg * 100.0f);
			}
		}
	}

	return ret;
}

/**
 * @brief 从驱动取反馈并换算到输出轴(度 / 度每秒 / N·m)
 */
void Motor::ReadInto(MotorFeedback &out)
{
	out = MotorFeedback{};

	if (Config.Dev == nullptr)
	{
		return;
	}

	const bool online = get_motor_heartbeat_status(Config.Dev) > 0;
	out.Online = online;
	if (!online)
	{
		return;
	}

	const smotor_receive_data_t *rx = get_motor_rxdata(Config.Dev);

	out.Encoder = static_cast<std::uint16_t>(rx->encoder);

	/* DJI/LK 反馈是转子绝对编码器值, 直接换成输出轴单圈角度, 不乘方向 */
	const float encoderDeg = EncoderToDegrees(rx->encoder, Config.EncoderCounts, Config.GearRatio);

	switch (Config.Type)
	{
	case MotorType::Dji:
	{
		/* 转子 rpm -> 输出轴 度/秒 */
		out.Position = encoderDeg;
		out.Speed = static_cast<float>(rx->speed) * 6.0f / Config.GearRatio;
		out.Current = static_cast<float>(rx->iq);
		out.Temperature = static_cast<float>(rx->specific_data.m3508.temp);
		break;
	}

	case MotorType::Lk:
	{
		/* 转子 1dps/LSB -> 输出轴 度/秒 */
		out.Position = encoderDeg;
		out.Speed = static_cast<float>(rx->speed) / Config.GearRatio;
		if (motor_rx_has(rx, MOTOR_RX_VALID_IQ))
		{
			out.Current = static_cast<float>(rx->iq);
		}
		out.Temperature = static_cast<float>(rx->specific_data.lk.temp);
		break;
	}

	case MotorType::Dm:
	{
		/* DM 反馈已是输出轴量(rad / rad/s / N·m) */
		const auto &dm = rx->specific_data.dm;
		out.Position = dm.pos_real * (180.0f / kPi);
		out.Speed = dm.vel_real * (180.0f / kPi);
		out.Torque = dm.iq_real;
		out.Temperature = static_cast<float>(dm.motor_temp);
		break;
	}

	case MotorType::Unknown:
	default:
		break;
	}
}

} // namespace breeze::device