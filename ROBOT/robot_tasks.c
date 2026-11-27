/**
 * @file robot_tasks.c
 * @brief 工程适配层 — 控制任务实现
 *
 * @note 单任务 1kHz 控制环（FreeRTOS 静态分配，heap 仅 15KB）：
 *       板间链路计时 → 安全周期 → 遥控心跳 → 操作者放行 → 命令映射 → 安全门控 →
 *       机械臂周期 → DM 总线周期发送，每周期向心跳灯打卡。
 *       任务存活指示灯任务在 USERS/Application/Task/ArmTask.c（led_task_create）。
 *       CAN 反馈与 SBUS 解析均在中断内完成（见 robot_port.c）。
 */

#include "robot_tasks.h"

#include "cmsis_os.h"

#include "main.h" /* HAL_GetTick */
#include "module_dm_motor_bus.h"
#include "bsp_log.h"
#include "led.hpp" /* WS2812 心跳灯打卡（C 接口经 extern "C" 导出） */
#include "ArmTask.h" /* led_task_create */
#include "robot_adapter.h"
#include "robot_config.h"
#include "robot_port.h"

#include <stddef.h>

/* ======================== 任务资源（静态分配） ======================== */

#define ROBOT_TASK_STACK_WORDS (512U) /* 2KB */

static uint32_t s_arm_task_stack[ROBOT_TASK_STACK_WORDS];
static osStaticThreadDef_t s_arm_task_control_block;

static void robot_arm_task_entry(void const *argument);

/* CMSIS-OS v1：stacksize 直接作为 xTaskCreateStatic 的栈深度（单位：字） */
osThreadStaticDef(ArmTask, robot_arm_task_entry, osPriorityHigh, 0,
                  ROBOT_TASK_STACK_WORDS, s_arm_task_stack, &s_arm_task_control_block);

/* ======================== 控制周期 ======================== */

#if ROBOT_REMOTE_DEBUG_LOG
/** @brief 板间联调：死区后的摇杆值（|v|≤ROBOT_REMOTE_DEADBAND 置 0，与控制层规则一致） */
static int16_t robot_debug_apply_deadband(int16_t v)
{
    return ((v > (int16_t)ROBOT_REMOTE_DEADBAND) || (v < -(int16_t)ROBOT_REMOTE_DEADBAND))
               ? v
               : 0;
}

/** @brief 遥控日志变化判定：与上次打印相比，任一摇杆（死区后）/拨杆/在线状态不同即为变化 */
static bool robot_debug_remote_changed(const app_remote_input_t *current,
                                       const app_remote_input_t *printed)
{
    if (current->online != printed->online)
    {
        return true;
    }
    for (uint32_t i = 0U; i < 4U; ++i)
    {
        if ((robot_debug_apply_deadband(current->channel[i]) !=
             robot_debug_apply_deadband(printed->channel[i])) ||
            (current->switch_raw[i] != printed->switch_raw[i]))
        {
            return true;
        }
    }
    return false;
}
#endif

/**
 * @brief 1kHz 机械臂控制环
 */
static void robot_control_run(void)
{
    bsp_dwt_time_point_t previous_time = {0};
    bool first_cycle = true;
    uint32_t previous_tick_ms = 0U; /* 板间链路计时：上周期 HAL_GetTick */
    bool first_tick = true;

    for (;;)
    {
        bsp_dwt_time_point_t current_time;
        uint32_t elapsed_us;
        uint32_t elapsed_ms = 0U; /* 板间链路计时用（首周期传 0） */
        float delta_time_s = 0.001F; /* 异常时刻的兜底周期 */
        uint32_t now_ms = HAL_GetTick();

        /* 板间链路计时用整数 ms：HAL_GetTick 差值无截断累积误差
           （原 elapsed_us/1000 每周期向下取整，超时判定被拉长约一倍） */
        if (!first_tick) {
            elapsed_ms = now_ms - previous_tick_ms;
        }
        previous_tick_ms = now_ms;
        first_tick = false;

        /* ---- 真实 dt（DWT 单调周期计数） ---- */
        if ((bsp_dwt_now(robot_get_dwt(), &current_time) == BSP_STATUS_OK))
        {
            if (!first_cycle &&
                (bsp_dwt_elapsed_cycles(robot_get_dwt(), previous_time, &elapsed_us) ==
                 BSP_STATUS_OK) &&
                (bsp_dwt_cycles_to_us(robot_get_dwt(), elapsed_us, &elapsed_us) ==
                 BSP_STATUS_OK))
            {
                const float measured_s = (float)elapsed_us * 0.000001F;
                /* 任务延迟过大时限制最大 dt，避免控制器吃下异常步长 */
                if ((measured_s > 0.0002F) && (measured_s < 0.005F))
                {
                    delta_time_s = measured_s;
                }
            }
            previous_time = current_time;
            first_cycle = false;
        }

        /* ---- 1. 板间链路超时计时（Robot Link 时间所有者：全机仅此一处累加） ---- */
        module_board_comm_update_time(robot_get_board_comm(), elapsed_ms);

        /* ---- 2. 安全周期：刷新监控器状态与输出门 ---- */
        app_safety_process(robot_get_safety(), now_ms);

        /* ---- 3. 遥控输入与心跳 ---- */
        app_remote_input_t remote_input;
        if (robot_adapter_remote_update(now_ms, &remote_input))
        {
            app_safety_notify_online(robot_get_remote_monitor(), now_ms);
        }

        /* ---- 4. 操作者放行：遥控在线且左拨杆不扣在下方 ---- */
        app_safety_set_output_enabled(robot_get_safety(),
                                      remote_input.online &&
                                          (remote_input.left_switch != APP_SWITCH_DOWN));

        /* ---- 5. 命令映射 ---- */
        app_arm_command_t arm_command;
        robot_adapter_arm_command(&remote_input, delta_time_s, &arm_command);

#if ROBOT_REMOTE_DEBUG_LOG
        /* 遥控日志（bsp_log → SEGGER RTT Channel 0）：变化即打印 + 心跳。
         * 数据取自 remote_input（robot_adapter 已在临界区内快照，与控制层同源，
         * 不会读到被 CAN 中断覆盖到一半的数据）。
         * t=时间戳(ms)；src=遥控来源（BOARD 板间 / UART 本地回退 / NONE 离线）；
         * on=当前来源的遥控在线标志；uc/age=板间链路提交计数与距上次提交的毫秒数
         * （仅 src=BOARD 时有效；uc 不变 → 链路没收到完整四帧）；
         * d1~d4=死区后的摇杆值（|v|≤30 置 0，与控制层规则一致）；
         * sw=拨杆 SWA/SWB/SWC/SWD 原始档位（+1=上 0=中 -1=下，SWC 为三档，
         * SWA/SWB/SWD 两档不会出现 0）；
         * rx=FDCAN1/2/3 RX FIFO0 中断实收帧计数（robot_port.c ISR 递增）：
         * rx3 不涨 → 帧根本没进 FDCAN3（底盘板未发/接线/终端电阻/波特率）；
         * rx3 涨而 uc=0 → 帧到了但被协议层拒收（ID/DLC/帧类型不符） */
        {
            /* printed_input 始终保存"最近一次已打印"的值：变化被限速挡下时，
             * 下一拍仍判定为变化，最终状态不会丢 */
            static app_remote_input_t printed_input;
            static bool printed_valid = false;
            static uint32_t last_print_ms = 0U;
            const module_board_comm_t *link = robot_get_board_comm();
            const bool changed =
                !printed_valid || robot_debug_remote_changed(&remote_input, &printed_input);
            const uint32_t since_print_ms = now_ms - last_print_ms;

            if ((changed && (since_print_ms >= ROBOT_REMOTE_LOG_MIN_INTERVAL_MS)) ||
                (since_print_ms >= ROBOT_REMOTE_LOG_HEARTBEAT_MS))
            {
                last_print_ms = now_ms;
                printed_input = remote_input;
                printed_valid = true;
                BSP_LOG_INFO(
                    "[board-rx] t=%lu src=%s on=%u uc=%lu age=%lu rx=%lu,%lu,%lu "
                    "d1=%d d2=%d d3=%d d4=%d "
                    "sw=%d,%d,%d,%d",
                    (unsigned long)now_ms,
                    remote_input.online ? (link->remote_online ? "BOARD" : "UART") : "NONE",
                    remote_input.online ? 1U : 0U,
                    (unsigned long)link->remote_data.update_count,
                    (unsigned long)link->remote_elapsed_time_ms,
                    (unsigned long)g_fdcan_rx_frame_count[0],
                    (unsigned long)g_fdcan_rx_frame_count[1],
                    (unsigned long)g_fdcan_rx_frame_count[2],
                    (int)robot_debug_apply_deadband(remote_input.channel[0]),
                    (int)robot_debug_apply_deadband(remote_input.channel[1]),
                    (int)robot_debug_apply_deadband(remote_input.channel[2]),
                    (int)robot_debug_apply_deadband(remote_input.channel[3]),
                    (int)remote_input.switch_raw[0], (int)remote_input.switch_raw[1],
                    (int)remote_input.switch_raw[2], (int)remote_input.switch_raw[3]);
            }
        }
#endif

        /* ---- 6. 安全门控：输出门未放行时强制断使能 ---- */
        if (!app_safety_output_allowed(robot_get_safety()))
        {
            arm_command.enabled = false;
            arm_command.gripper_enabled = false;
        }

        /* ---- 7. 机械臂周期（FDCAN2 总线 3 电机） ---- */
        (void)app_arm_update(robot_get_arm(), &arm_command, delta_time_s);

        /* ---- 8. DM 总线周期发送（每周期一次，内部做输出门批量失能） ---- */
        (void)module_dm_motor_bus_update(robot_get_gantry_bus(), delta_time_s);
        (void)module_dm_motor_bus_update(robot_get_arm_bus(), delta_time_s);

        /* ---- 9. 任务存活打卡：心跳灯据此判断控制环是否卡死 ---- */
        LED_TaskKick(LED_TASK_CONTROL);

        osDelay(1);
    }
}

static void robot_arm_task_entry(void const *argument)
{
    (void)argument;
    robot_control_run();
}

/* ======================== 任务创建入口 ======================== */

void robot_tasks_create(void)
{
    /* 心跳灯任务（实现在 ArmTask.c）始终创建：装配失败时不创建控制任务，
       LED 久无打卡 → 红灯快闪即装配失败信号 */
    led_task_create();

    /* 整机装配：失败时保持电机失能，不再创建控制任务 */
    const bsp_status_t init_status = robot_init();
    if (init_status != BSP_STATUS_OK)
    {
        BSP_LOG_ERROR("[boot] robot_init 失败 status=%d", (int)init_status);
        robot_enter_safe_state();
        return;
    }
    (void)osThreadCreate(osThread(ArmTask), NULL);
}
