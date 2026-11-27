/**
 * @file app_arm.h
 * @brief 机械臂应用模块接口 -- 双关节 + 夹爪三轴控制
 *
 * @note 由 ECF app_gimbal（俯仰/偏航双轴）改造而来：以 DM 电机 MIT 模式
 *       驱动机械臂两个关节电机（J1/J2）与末端夹爪电机，反馈统一使用电机
 *       编码器（本项目无 IMU）。夹爪夹持力由 MIT kp 决定，应用层仅下发
 *       目标角度。使能/失能帧只在状态切换瞬间发送，避免每个周期重发。
 */

#ifndef APP_ARM_H
#define APP_ARM_H

#include "bsp_common.h"
#include "app_types.h"
#include "module_dm_motor.h"

#include <stdbool.h>

/** @brief 机械臂模块的静态配置。 */
typedef struct
{
    module_dm_motor_t *joint1_motor;    /**< 关节1电机实例（肩部，对应原云台偏航轴）。 */
    module_dm_motor_t *joint2_motor;    /**< 关节2电机实例（肘部，对应原云台俯仰轴）。 */
    module_dm_motor_t *gripper_motor;   /**< 夹爪电机实例。 */
    float joint_kp;                     /**< 关节 MIT 比例增益。 */
    float joint_kd;                     /**< 关节 MIT 微分增益。 */
    float gripper_kp;                   /**< 夹爪 MIT 比例增益（决定夹持力）。 */
    float gripper_kd;                   /**< 夹爪 MIT 微分增益。 */
    float joint_tolerance_rad;          /**< 关节到位判定误差 [rad]。 */
    float gripper_tolerance_rad;        /**< 夹爪到位判定误差 [rad]。 */
} app_arm_config_t;

/** @brief 机械臂运行时实例。 */
typedef struct
{
    app_arm_config_t config;     /**< 静态配置的副本。 */
    app_arm_feedback_t feedback; /**< 最近一次机械臂反馈。 */
    bool joints_output_active;   /**< 关节电机当前处于使能状态（切换检测）。 */
    bool gripper_output_active;  /**< 夹爪电机当前处于使能状态（切换检测）。 */
    bool initialized;            /**< 初始化阶段已成功完成。 */
} app_arm_t;

/**
 * @brief  初始化机械臂模块。
 * @param  me      指向调用方分配的实例。
 * @param  config  静态配置（内部拷贝）。
 * @return 成功返回 BSP_STATUS_OK。
 */
bsp_status_t app_arm_init(app_arm_t *me, const app_arm_config_t *config);

/**
 * @brief  执行一个机械臂控制周期。
 * @param  me            已初始化的机械臂实例。
 * @param  command       机械臂指令（目标角度与使能标志）。
 * @param  delta_time_s  距上次调用的经过时间 [s]。
 * @note   三轴均为 DM MIT 模式：目标角度以零前馈速度、配置 kp/kd 下发。
 *         command->gripper_enabled 为 false 时仅禁用夹爪电机，
 *         关节电机仍受 command->enabled 控制。
 */
bsp_status_t app_arm_update(app_arm_t *me,
                            const app_arm_command_t *command,
                            float delta_time_s);

const app_arm_feedback_t *app_arm_get_feedback(const app_arm_t *me);

#endif
