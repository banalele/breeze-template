#pragma once

#include "zephyr/kernel.h"

/* 任务优先级定义（数值越小优先级越高，有效范围 0~14）
 *   1 System    : ///<这里更新的是键鼠、裁判系统、视觉系统、esp32数据、板间通信
 *   2 Update    : 1ms 控制主循环
 *   3 CAN RX    : 反馈解析（见 prj.conf）
 *   4 CAN TX    : 1ms 发送节拍（见 prj.conf）
 *   5 Heartbeat : 100ms 喂狗 / 离线计数
 *   7 Monitor   : 10ms 调试输出（最低）
 */
#define proc_SystemTaskPriority 1    // 系统更新任务（最高）
#define proc_UpdateTaskPriority 2    // 主更新任务
#define proc_HeartbeatTaskPriority 5 // 心跳任务
#define proc_MonitorTaskPriority 7   // 监控任务（最低）

/* 任务栈大小定义 */
#define STACK_SIZE_SYSTEM 4096
#define STACK_SIZE_UPDATE 4096
#define STACK_SIZE_HEARTBEAT 1024 // 1024 时只剩 32B 余量，喂狗线程必挂
#define STACK_SIZE_MONITOR 4096

namespace breeze
{
    /* 线程控制块（替代 TaskHandle_t） */
    extern struct k_thread system_update_thread_ctrl;
    extern struct k_thread update_thread_ctrl;
    extern struct k_thread heartbeat_thread_ctrl;
    extern struct k_thread monitor_thread_ctrl;

    /* 任务入口函数（C++ 链接，由 k_thread_create 直接引用） */
    void StartSystemUpdateTask(void *arg1, void *arg2, void *arg3);
    void StartUpdateTask(void *arg1, void *arg2, void *arg3);
    void StartHeartbeatTask(void *arg1, void *arg2, void *arg3);
    void StartMonitorTask(void *arg1, void *arg2, void *arg3);

    /* 初始化所有任务 */
    void InitProcess(void);

} // namespace breeze