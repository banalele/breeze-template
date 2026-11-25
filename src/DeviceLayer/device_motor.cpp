#include "device_motor.hpp"

#include <zephyr/devicetree.h>

/* ------------------------------------------------------------------------- */
/* 板级电机实例: 模板板只带 dial_motor(LK), 其他应用自行实例化 breeze::device::Motor */
/* ------------------------------------------------------------------------- */

namespace breeze::device
{

Motor dial_motor{MotorConfig{
    BREEZE_MOTOR_DT(DT_NODELABEL(dial_motor)),
    .LkMode = LkControlMode::Single,
    .Conversion =
        {
            .MaxOutput = 2048.0f,
            .Direction = 1.0f,
        },
}};

} // namespace breeze::device

int Motor_Init(void)
{
	int ret = breeze::device::dial_motor.Init();

	return ret;
}
