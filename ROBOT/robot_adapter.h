/**
 * @file robot_adapter.h
 * @brief 工程适配层 — i6x 遥控数据转换与轴命令映射
 *
 * @note 按《机械臂开发架构》约定：SBUS 通道 → 轴命令的映射（死区/方向/限幅）
 *       全部集中在本文件，ECF 组件与应用层保持设备无关。
 */

#ifndef ROBOT_ADAPTER_H
#define ROBOT_ADAPTER_H

#include "app_arm.h"
#include "app_types.h"
#include "Ix6_Remote.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 更新遥控输入快照（每个控制周期调用一次）
 * @param now_ms  当前时刻 [ms]（HAL_GetTick）
 * @param output  输出快照（始终写入，online 反映失联状态）
 * @return 遥控是否在线（新帧在超时窗口内且未触发失控保护）
 */
bool robot_adapter_remote_update(uint32_t now_ms, app_remote_input_t *output);

/**
 * @brief i6x 三态开关原始值 (-1/0/+1) → 应用层三段开关枚举
 */
app_switch_t robot_adapter_map_switch(int8_t raw);

/**
 * @brief 将遥控输入映射为机械臂命令（增量速率累积 + 限幅 + 夹爪开关）
 * @param remote        遥控输入快照
 * @param delta_time_s  控制周期 [s]
 * @param output        机械臂命令输出
 */
void robot_adapter_arm_command(const app_remote_input_t *remote, float delta_time_s,
                               app_arm_command_t *output);

#ifdef __cplusplus
}
#endif
#endif /* ROBOT_ADAPTER_H */
