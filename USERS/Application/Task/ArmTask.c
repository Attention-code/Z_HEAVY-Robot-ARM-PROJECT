/**
 * @file ArmTask.c
 * @brief WS2812 心跳灯任务（任务存活指示）
 *
 * @note 100ms 周期驱动 bsp_led：检查各任务打卡情况并刷灯
 *       - 彩虹渐变+呼吸：所有被监控任务存活（当前仅 ArmTask 控制环）
 *       - 红灯 500ms 快闪：有任务 >200ms 未打卡（卡死/装配失败）
 *       - 常暗：本任务自身或调度器未跑
 *       本任务无条件创建（robot_tasks_create 调 led_task_create），
 *       装配失败时控制任务不创建 → 红灯快闪即装配失败信号。
 */

#include "ArmTask.h"

#include "cmsis_os.h"
#include "led.hpp" /* WS2812 心跳灯（C 接口经 extern "C" 导出） */

/* ======================== 任务资源（静态分配，heap 仅 15KB） ======================== */

#define LED_TASK_STACK_WORDS (512U) /* 2KB：呼吸渐变含 sinf/HSV 浮点计算 + HAL SPI 嵌套，
                                     * 1KB 偏紧，栈溢出会 HardFault 冻结整机（灯卡一色） */

static uint32_t s_led_task_stack[LED_TASK_STACK_WORDS];
static osStaticThreadDef_t s_led_task_control_block;

static void robot_led_task_entry(void const *argument);

/* CMSIS-OS v1：stacksize 直接作为 xTaskCreateStatic 的栈深度（单位：字）
 * 低优先级：单次刷灯阻塞约 200us（SPI 发 124 字节），可被控制环随时抢占 */
osThreadStaticDef(LedTask, robot_led_task_entry, osPriorityNormal, 0,
                  LED_TASK_STACK_WORDS, s_led_task_stack, &s_led_task_control_block);

/**
 * @brief WS2812 心跳灯任务主体
 */
static void robot_led_task_entry(void const *argument)
{
    (void)argument;
    for (;;)
    {
        LED_Heartbeat();
        osDelay(100U);
    }
}

void led_task_create(void)
{
    (void)osThreadCreate(osThread(LedTask), NULL);
}
