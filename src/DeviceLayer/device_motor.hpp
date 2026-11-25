#pragma once

#include "motor.hpp"

/**
 * @brief 板级电机初始化(模板板为拨盘电机)
 * @note 只供 Device_Init() 调用; 应用自建电机请直接用 breeze::device::Motor.
 */
int Motor_Init(void);

namespace breeze::device
{

extern Motor dial_motor;

} // namespace breeze::device