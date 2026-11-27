/**
 * @file robot_adapter.c
 * @brief 工程适配层 — i6x 遥控数据转换与轴命令映射实现
 *
 * @note 通道映射约定（i6x 全 8 通道：CH1~CH4 摇杆 ±660 + CH5~CH8 拨杆 SWA~SWD，
 *       上机前需按实际遥控器核对，见 robot_config.h 宏）：
 *       ch[0] → 关节1（肩），ch[1] → 关节2（肘），ch[2]/ch[3] 预留龙门架。
 *       全部开关档位以 i6x 原始值（+1=上 0=中 -1=下）经板间协议第 4 分片直传，
 *       板间与本地 UART 两个来源共用 robot_adapter_map_switch() 映射：
 *       SWA → 左拨杆（使能门控）、SWB → 右拨杆（夹爪）、
 *       SWC/SWD 原始档位存于 switch_raw[2]/[3] 备用。
 *       安全策略：左拨杆 DOWN = 整机断使能；右拨杆 DOWN = 夹爪闭合。
 */

#include "robot_adapter.h"

#include <math.h>
#include <stddef.h>

#include "cmsis_os.h" /* taskENTER/EXIT_CRITICAL：与 CAN ISR 的临界区互斥 */
#include "robot_config.h"

/* ======================== 遥控失联检测（本地 UART i6x 回退） ======================== */

static uint32_t s_last_frame_count;    /* 上次看到的合法帧计数 */
static uint32_t s_last_frame_time_ms;  /* 最近一次新帧时刻 */
static bool s_frame_seen;              /* 是否至少收到过一帧 */

/** @brief 本地 i6x（UART5 SBUS）快照，仅在板间快照失联时作为兜底来源 */
static bool robot_adapter_remote_from_uart(uint32_t now_ms, app_remote_input_t *output)
{
    const i6x_ctrl_t *remote = get_i6x_point();

    if (remote == NULL)
    {
        return false;
    }
    /* 检测新帧：SBUS 约 7~14ms 一帧，超时视为失联 */
    if (remote->frame_cnt != s_last_frame_count)
    {
        s_last_frame_count = remote->frame_cnt;
        s_last_frame_time_ms = now_ms;
        s_frame_seen = true;
    }
    const bool online = s_frame_seen && !remote->failsafe &&
                        ((now_ms - s_last_frame_time_ms) < ROBOT_REMOTE_TIMEOUT_MS);

    *output = (app_remote_input_t){
        .channel = {remote->ch[0], remote->ch[1], remote->ch[2], remote->ch[3]},
        .left_switch = robot_adapter_map_switch(remote->s[ROBOT_REMOTE_LEFT_SWITCH_INDEX]),
        .right_switch = robot_adapter_map_switch(remote->s[ROBOT_REMOTE_RIGHT_SWITCH_INDEX]),
        .switch_raw = {remote->s[0], remote->s[1], remote->s[2], remote->s[3]},
        .sequence = remote->frame_cnt,
        .online = online,
    };
    return online;
}

bool robot_adapter_remote_update(uint32_t now_ms, app_remote_input_t *output)
{
    if (output == NULL)
    {
        return false;
    }
    /* 先整体清零：离线/回退失败时日志与下游读到的是稳定 0，而非栈上残留值 */
    *output = (app_remote_input_t){0};

#if ROBOT_REMOTE_BOARD_SOURCE
    /* 正式方案：底盘板 Robot Link（module_board_comm，复用 ECF 组件）。
       快照读取：remote_data 在 CAN ISR 中整体提交，任务读取时进入临界区
       做一次拷贝，避免读到被 ISR 打断的混合数据 */
    const module_board_comm_t *link = robot_get_board_comm();
    module_board_comm_remote_process_data_t snapshot;
    bool board_online;
    taskENTER_CRITICAL();
    board_online = link->remote_online;
    snapshot = link->remote_data;
    taskEXIT_CRITICAL();
    if (board_online)
    {
        /* 原始开关档位直传（+1=上 0=中 -1=下），与本地 UART 回退共用同一映射函数 */
        *output = (app_remote_input_t){
            .channel = {snapshot.channel[0], snapshot.channel[1], snapshot.channel[2],
                        snapshot.channel[3]},
            .left_switch = robot_adapter_map_switch(snapshot.switch_raw[0]),  /* SWA 两档 */
            .right_switch = robot_adapter_map_switch(snapshot.switch_raw[1]), /* SWB 两档 */
            .switch_raw = {snapshot.switch_raw[0], snapshot.switch_raw[1],
                           snapshot.switch_raw[2], snapshot.switch_raw[3]},
            .dial = snapshot.dial,
            .sequence = snapshot.update_count,
            .online = true,
        };
        return true;
    }
    output->online = false; /* 遥控离线（超时后 remote_online 为 false） */
#endif

#if ROBOT_REMOTE_UART_FALLBACK
    /* 调试回退：板间快照失联时尝试本地接收机 */
    if (robot_adapter_remote_from_uart(now_ms, output))
    {
        return true;
    }
#endif

    output->online = false;
    return false;
}

/** @brief i6x 三态开关 (-1/0/+1) → 应用层三段开关枚举 */
app_switch_t robot_adapter_map_switch(int8_t raw)
{
    switch (raw)
    {
    case I6X_SW_UP:
        return APP_SWITCH_UP;
    case I6X_SW_DOWN:
        return APP_SWITCH_DOWN;
    default:
        return APP_SWITCH_MIDDLE;
    }
}

/* ======================== 机械臂命令映射 ======================== */

/** @brief 通道值限幅 + 死区 + 归一化到 [-1, 1] */
static float robot_adapter_normalize_channel(int16_t value)
{
    float normalized;

    if (value > ROBOT_REMOTE_DEADBAND)
    {
        normalized = (float)(value - ROBOT_REMOTE_DEADBAND);
    }
    else if (value < -ROBOT_REMOTE_DEADBAND)
    {
        normalized = (float)(value + ROBOT_REMOTE_DEADBAND);
    }
    else
    {
        return 0.0F;
    }
    normalized /= (float)(ROBOT_REMOTE_CHANNEL_MAX - ROBOT_REMOTE_DEADBAND);
    if (normalized > 1.0F)
    {
        normalized = 1.0F;
    }
    return normalized;
}

/** @brief 速率累积目标角并限幅 */
static float robot_adapter_advance_target(float current, float normalized_channel,
                                          float rate_rad_per_s, float delta_time_s,
                                          float minimum_rad, float maximum_rad)
{
    current += normalized_channel * rate_rad_per_s * delta_time_s;
    if (current > maximum_rad)
    {
        current = maximum_rad;
    }
    else if (current < minimum_rad)
    {
        current = minimum_rad;
    }
    return current;
}

void robot_adapter_arm_command(const app_remote_input_t *remote, float delta_time_s,
                               app_arm_command_t *output)
{
    static float joint1_target_rad = ROBOT_J1_DEFAULT_RAD;
    static float joint2_target_rad = ROBOT_J2_DEFAULT_RAD;
    const bool online = (remote != NULL) && remote->online;
    const bool enabled = online && (remote->left_switch != APP_SWITCH_DOWN);

    if (output == NULL)
    {
        return;
    }

    if (enabled)
    {
        joint1_target_rad =
            robot_adapter_advance_target(joint1_target_rad,
                                         robot_adapter_normalize_channel(remote->channel[0]),
                                         ROBOT_J1_MAX_RATE_RAD_PER_S, delta_time_s,
                                         ROBOT_J1_MIN_RAD, ROBOT_J1_MAX_RAD);
        joint2_target_rad =
            robot_adapter_advance_target(joint2_target_rad,
                                         robot_adapter_normalize_channel(remote->channel[1]),
                                         ROBOT_J2_MAX_RATE_RAD_PER_S, delta_time_s,
                                         ROBOT_J2_MIN_RAD, ROBOT_J2_MAX_RAD);
    }

    *output = (app_arm_command_t){
        .joint1_target_rad = joint1_target_rad,
        .joint2_target_rad = joint2_target_rad,
        .gripper_target_rad = (enabled && (remote->right_switch == APP_SWITCH_DOWN))
                                  ? ROBOT_GRIPPER_CLOSE_RAD
                                  : ROBOT_GRIPPER_OPEN_RAD,
        .gripper_enabled = enabled,
        .enabled = enabled,
        .sequence = online ? remote->sequence : 0U,
    };
}
