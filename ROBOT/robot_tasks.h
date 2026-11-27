/**
 * @file robot_tasks.h
 * @brief 工程适配层 — FreeRTOS 任务创建入口
 */

#ifndef ROBOT_TASKS_H
#define ROBOT_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  创建整机全部 FreeRTOS 任务（由 MX_FREERTOS_Init 调用）。
 * @note   内部先调用 robot_init() 完成整机装配，装配失败则保持电机失能。
 */
void robot_tasks_create(void);

#ifdef __cplusplus
}
#endif
#endif /* ROBOT_TASKS_H */
