#include <zephyr/logging/log.h>
#include "device.hpp"
LOG_MODULE_REGISTER(device, LOG_LEVEL_INF);

//------------------------------获取设备节点---------------------------------------
/* 看门狗 */
const struct device *iwdg_dev = DEVICE_DT_GET(DT_ALIAS(watchdog0));

//-------------------------------------------------------------------------------

int Device_Init(void)
{
    int ret = 0;

    ret = IWDG_Init(iwdg_dev);
    if (ret != 0)
    {
        LOG_ERR("IWDG_Init failed with error code: %d", ret);
        return ret;
    }

    vofa_rtt_init();
    return 0;
}