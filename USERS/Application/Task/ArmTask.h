/**
 * @file ArmTask.h
 * @brief WS2812 心跳灯任务接口
 */

#ifndef ARMTASK_H
#define ARMTASK_H

/**
 * @brief 创建心跳灯任务（无条件创建，作为存活/装配失败指示）
 */
void led_task_create(void);

#endif /* ARMTASK_H */
