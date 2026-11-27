/**
 * @file app_arm.c
 * @brief 机械臂应用模块实现
 *
 * @note 双关节（J1/J2）+ 夹爪三轴 MIT 位置控制，
 *       纯编码器反馈。关节电机随 command->enabled 使能/禁用，夹爪电机
 *       额外受 command->gripper_enabled 独立控制。
 *       使能/失能帧只在状态切换瞬间发送（module_motor_enable/disable
 *       不做幂等检查，每周期重发会淹没 CAN 总线）。
 */

#include "app_arm.h"

#include <math.h>

/* ======================== 内部工具 ======================== */

/**
 * @brief 构造 MIT 位置命令（零前馈速度、零前馈扭矩，kp/kd 由配置提供）
 * @return true 表示命令已被电机接受
 */
static bool app_arm_set_mit_position(module_dm_motor_t *motor, float target_rad,
                                     float kp, float kd)
{
    const module_dm_mit_command_t mit_command = {
        .position_rad = target_rad,
        .velocity_rad_per_s = 0.0F,
        .proportional_gain = kp,
        .derivative_gain = kd,
        .torque_nm = 0.0F,
    };
    return (module_dm_motor_set_mit_target(motor, &mit_command) == MODULE_MOTOR_STATUS_OK);
}

/* ======================== 公共 API ======================== */

/**
 * @brief  初始化机械臂模块。
 * @param  me      指向调用方分配的实例。
 * @param  config  静态配置（内部拷贝）。
 * @return 成功返回 BSP_STATUS_OK，参数无效返回 BSP_STATUS_INVALID_ARGUMENT。
 */
bsp_status_t app_arm_init(app_arm_t *me, const app_arm_config_t *config)
{
    if ((me == NULL) || (config == NULL) || (config->joint1_motor == NULL) ||
        (config->joint2_motor == NULL) || (config->gripper_motor == NULL) ||
        (config->joint_tolerance_rad < 0.0F) ||
        (config->gripper_tolerance_rad < 0.0F))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *me = (app_arm_t){
        .config = *config,
        .joints_output_active = false,
        .gripper_output_active = false,
        .initialized = true,
    };
    return BSP_STATUS_OK;
}

/**
 * @brief  执行一个机械臂控制周期。
 * @param  me       已初始化的机械臂实例。
 * @param  command  机械臂指令。
 * @param  delta_time_s  距上次调用的经过时间 [s]。
 *
 * 读取三轴电机编码器反馈；使能请求 = 指令使能 && 反馈在线。使能状态
 * 只在发生切换时下发使能/失能帧；受控期间每周期下发 MIT 位置目标。
 * 任一使能轴离线或电机操作失败时返回 IO 错误，但已使能轴继续受控。
 */
bsp_status_t app_arm_update(app_arm_t *me,
                            const app_arm_command_t *command,
                            float delta_time_s)
{
    app_arm_feedback_t feedback = {0};
    const module_motor_feedback_t *joint1_feedback;
    const module_motor_feedback_t *joint2_feedback;
    const module_motor_feedback_t *gripper_feedback;

    bool joints_requested;
    bool gripper_requested;
    bool had_motor_error = false;

    if ((me == NULL) || (command == NULL) || !isfinite(delta_time_s) ||
        (delta_time_s <= 0.0F))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (!me->initialized)
    {
        return BSP_STATUS_NOT_INITIALIZED;
    }

    joint1_feedback = module_motor_get_feedback(&me->config.joint1_motor->super);
    joint2_feedback = module_motor_get_feedback(&me->config.joint2_motor->super);
    gripper_feedback = module_motor_get_feedback(&me->config.gripper_motor->super);

    /* ---- 关节：使能请求 = 指令使能 && 双关节反馈在线 ---- */
    joints_requested = command->enabled && (joint1_feedback != NULL) &&
                       joint1_feedback->is_online && (joint2_feedback != NULL) &&
                       joint2_feedback->is_online;

    if (joints_requested && !me->joints_output_active)
    {
        /* 状态切换：发送使能帧（需要反馈在线，module_motor_enable 内部校验） */
        if ((module_motor_enable(&me->config.joint1_motor->super) == MODULE_MOTOR_STATUS_OK) &&
            (module_motor_enable(&me->config.joint2_motor->super) == MODULE_MOTOR_STATUS_OK))
        {
            me->joints_output_active = true;
        }
        else
        {
            had_motor_error = true;
        }
    }
    else if (!joints_requested && me->joints_output_active)
    {
        /* 状态切换：发送失能帧 */
        (void)module_motor_disable(&me->config.joint1_motor->super);
        (void)module_motor_disable(&me->config.joint2_motor->super);
        me->joints_output_active = false;
    }

    /* 受控期间每周期下发 MIT 位置目标 */
    if (me->joints_output_active)
    {
        had_motor_error =
            !app_arm_set_mit_position(me->config.joint1_motor, command->joint1_target_rad,
                                      me->config.joint_kp, me->config.joint_kd) ||
            !app_arm_set_mit_position(me->config.joint2_motor, command->joint2_target_rad,
                                      me->config.joint_kp, me->config.joint_kd) ||
            had_motor_error;
    }

    /* ---- 夹爪：使能请求 = 指令使能 && 夹爪反馈在线 ---- */
    gripper_requested = command->gripper_enabled && (gripper_feedback != NULL) &&
                        gripper_feedback->is_online;

    if (gripper_requested && !me->gripper_output_active)
    {
        if (module_motor_enable(&me->config.gripper_motor->super) == MODULE_MOTOR_STATUS_OK)
        {
            me->gripper_output_active = true;
        }
        else
        {
            had_motor_error = true;
        }
    }
    else if (!gripper_requested && me->gripper_output_active)
    {
        (void)module_motor_disable(&me->config.gripper_motor->super);
        me->gripper_output_active = false;
    }

    if (me->gripper_output_active)
    {
        had_motor_error =
            !app_arm_set_mit_position(me->config.gripper_motor, command->gripper_target_rad,
                                      me->config.gripper_kp, me->config.gripper_kd) ||
            had_motor_error;
    }

    /* 汇总反馈：使能但反馈缺失的轴位置记 0。 */
    feedback.joint1_rad = (joint1_feedback != NULL) ? joint1_feedback->position_rad : 0.0F;
    feedback.joint2_rad = (joint2_feedback != NULL) ? joint2_feedback->position_rad : 0.0F;
    feedback.gripper_rad = (gripper_feedback != NULL) ? gripper_feedback->position_rad : 0.0F;
    feedback.joint1_velocity_rad_per_s =
        (joint1_feedback != NULL) ? joint1_feedback->velocity_rad_per_s : 0.0F;
    feedback.joint2_velocity_rad_per_s =
        (joint2_feedback != NULL) ? joint2_feedback->velocity_rad_per_s : 0.0F;
    feedback.motors_online =
        (joint1_feedback != NULL) && joint1_feedback->is_online &&
        (joint2_feedback != NULL) && joint2_feedback->is_online &&
        (gripper_feedback != NULL) && gripper_feedback->is_online;

    /* 三轴误差均在各自容差范围内时判定目标已锁定。 */
    feedback.target_locked =
        (fabsf(command->joint1_target_rad - feedback.joint1_rad) <=
         me->config.joint_tolerance_rad) &&
        (fabsf(command->joint2_target_rad - feedback.joint2_rad) <=
         me->config.joint_tolerance_rad) &&
        (!command->gripper_enabled ||
         (fabsf(command->gripper_target_rad - feedback.gripper_rad) <=
          me->config.gripper_tolerance_rad));

    me->feedback = feedback;
    return had_motor_error ? BSP_STATUS_IO_ERROR : BSP_STATUS_OK;
}

const app_arm_feedback_t *app_arm_get_feedback(const app_arm_t *me)
{
    return ((me != NULL) && me->initialized) ? &me->feedback : NULL;
}
