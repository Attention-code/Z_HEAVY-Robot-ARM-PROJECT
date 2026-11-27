/**
 * @file robot_config.c
 * @brief 工程适配层 — 整机装配与实例定义
 *
 * @note 总线划分：FDCAN1=龙门2电机(Classic 1Mbps)、FDCAN2=机械臂3电机(Classic 1Mbps)、
 *       FDCAN3=板间通信(Classic 1Mbps，Robot Link 协议，复用 ECF module_board_comm)。
 *       装配顺序遵循 ECF 移植手册：BSP driver ops → Module（DM 总线与电机、板间通信）→
 *       App（机械臂、安全）→ 注册心跳监控（输出保持禁止，等待操作者放行）。
 *       CAN 反馈在中断内直接路由到对应 DM 总线；板间遥控帧在中断内解析。
 */

#include "robot_config.h"
#include "robot_port.h"
#include "robot_adapter.h"

#include "Ix6_Remote.h"
#include "bsp_log.h"
#include "fdcan.h" /* hfdcan1 / hfdcan2 / hfdcan3 */
#include "usart.h" /* huart5（遥控本地回退用） */

#include <stddef.h>

/* ======================== 静态实例 ======================== */

static robot_t s_robot;

static bsp_fdcan_t s_fdcan_gantry;                    /* FDCAN1 BSP 实例 */
static bsp_fdcan_t s_fdcan_arm;                       /* FDCAN2 BSP 实例 */
static bsp_fdcan_t s_fdcan_board;                     /* FDCAN3 BSP 实例（板间 Classic） */
static bsp_fdcan_classic_adapter_t s_gantry_adapter;  /* 龙门 Classic 适配器 */
static bsp_fdcan_classic_adapter_t s_arm_adapter;     /* 机械臂 Classic 适配器 */
static bsp_fdcan_classic_adapter_t s_board_adapter;   /* 板间 Classic 适配器 */
static bsp_usart_t s_remote_usart;                    /* 遥控串口（仅 UART 回退用） */

/** @brief DM 总线电机指针数组（bus_init 要求） */
static module_dm_motor_t *s_gantry_motor_slots[ROBOT_GANTRY_MOTOR_COUNT];
static module_dm_motor_t *s_arm_motor_slots[ROBOT_ARM_MOTOR_COUNT];

/** @brief DM 电机实例：龙门 2 台 + 机械臂 3 台 */
static module_dm_motor_t s_dm_motors[ROBOT_GANTRY_MOTOR_COUNT + ROBOT_ARM_MOTOR_COUNT];

/** @brief 机械臂配置（指向电机实例） */
static app_arm_config_t s_arm_config;

/* ======================== 电机参数表 ======================== */

/**
 * 【TODO-上机核对】CAN ID 与机械布局。
 * DM 协议约定：控制帧发往电机 CAN_ID（= master_identifier，MIT 模式无偏移），
 * 电机反馈帧 ID = 电机的 MASTER_ID（= feedback_identifier），
 * 反馈 D0 低 4 位必须等于电机 CAN_ID 低 4 位。
 * 龙门电机在 FDCAN1（gantry_bus），机械臂电机在 FDCAN2（arm_bus）。
 */
static const module_dm_motor_config_t s_dm_motor_configs[ROBOT_GANTRY_MOTOR_COUNT +
                                                        ROBOT_ARM_MOTOR_COUNT] = {
    /* ---- FDCAN1 / gantry_bus：龙门 ---- */
    /* 0: 龙门左 */
    {.name = "gantry_left", .motor_bus = &s_robot.gantry_bus, .control_mode = MODULE_DM_MODE_MIT,
     .master_identifier = 0x01U, .feedback_identifier = 0x11U,
     .transmit_timeout_ms = 2U,
     .limits = {.position_min_rad = ROBOT_DM_POSITION_MIN_RAD,
                .position_max_rad = ROBOT_DM_POSITION_MAX_RAD,
                .velocity_min_rad_per_s = ROBOT_DM_VELOCITY_MIN_RAD_PER_S,
                .velocity_max_rad_per_s = ROBOT_DM_VELOCITY_MAX_RAD_PER_S,
                .torque_min_nm = ROBOT_DM_TORQUE_MIN_NM, .torque_max_nm = ROBOT_DM_TORQUE_MAX_NM,
                .proportional_gain_min = ROBOT_DM_KP_MIN, .proportional_gain_max = ROBOT_DM_KP_MAX,
                .derivative_gain_min = ROBOT_DM_KD_MIN, .derivative_gain_max = ROBOT_DM_KD_MAX}},
    /* 1: 龙门右 */
    {.name = "gantry_right", .motor_bus = &s_robot.gantry_bus, .control_mode = MODULE_DM_MODE_MIT,
     .master_identifier = 0x02U, .feedback_identifier = 0x12U,
     .transmit_timeout_ms = 2U,
     .limits = {.position_min_rad = ROBOT_DM_POSITION_MIN_RAD,
                .position_max_rad = ROBOT_DM_POSITION_MAX_RAD,
                .velocity_min_rad_per_s = ROBOT_DM_VELOCITY_MIN_RAD_PER_S,
                .velocity_max_rad_per_s = ROBOT_DM_VELOCITY_MAX_RAD_PER_S,
                .torque_min_nm = ROBOT_DM_TORQUE_MIN_NM, .torque_max_nm = ROBOT_DM_TORQUE_MAX_NM,
                .proportional_gain_min = ROBOT_DM_KP_MIN, .proportional_gain_max = ROBOT_DM_KP_MAX,
                .derivative_gain_min = ROBOT_DM_KD_MIN, .derivative_gain_max = ROBOT_DM_KD_MAX}},
    /* ---- FDCAN2 / arm_bus：机械臂 ---- */
    /* 2: 关节1（肩） */
    {.name = "arm_joint1", .motor_bus = &s_robot.arm_bus, .control_mode = MODULE_DM_MODE_MIT,
     .master_identifier = 0x03U, .feedback_identifier = 0x13U,
     .transmit_timeout_ms = 2U,
     .limits = {.position_min_rad = ROBOT_DM_POSITION_MIN_RAD,
                .position_max_rad = ROBOT_DM_POSITION_MAX_RAD,
                .velocity_min_rad_per_s = ROBOT_DM_VELOCITY_MIN_RAD_PER_S,
                .velocity_max_rad_per_s = ROBOT_DM_VELOCITY_MAX_RAD_PER_S,
                .torque_min_nm = ROBOT_DM_TORQUE_MIN_NM, .torque_max_nm = ROBOT_DM_TORQUE_MAX_NM,
                .proportional_gain_min = ROBOT_DM_KP_MIN, .proportional_gain_max = ROBOT_DM_KP_MAX,
                .derivative_gain_min = ROBOT_DM_KD_MIN, .derivative_gain_max = ROBOT_DM_KD_MAX}},
    /* 3: 关节2（肘） */
    {.name = "arm_joint2", .motor_bus = &s_robot.arm_bus, .control_mode = MODULE_DM_MODE_MIT,
     .master_identifier = 0x04U, .feedback_identifier = 0x14U,
     .transmit_timeout_ms = 2U,
     .limits = {.position_min_rad = ROBOT_DM_POSITION_MIN_RAD,
                .position_max_rad = ROBOT_DM_POSITION_MAX_RAD,
                .velocity_min_rad_per_s = ROBOT_DM_VELOCITY_MIN_RAD_PER_S,
                .velocity_max_rad_per_s = ROBOT_DM_VELOCITY_MAX_RAD_PER_S,
                .torque_min_nm = ROBOT_DM_TORQUE_MIN_NM, .torque_max_nm = ROBOT_DM_TORQUE_MAX_NM,
                .proportional_gain_min = ROBOT_DM_KP_MIN, .proportional_gain_max = ROBOT_DM_KP_MAX,
                .derivative_gain_min = ROBOT_DM_KD_MIN, .derivative_gain_max = ROBOT_DM_KD_MAX}},
    /* 4: 夹爪 */
    {.name = "arm_gripper", .motor_bus = &s_robot.arm_bus, .control_mode = MODULE_DM_MODE_MIT,
     .master_identifier = 0x05U, .feedback_identifier = 0x15U,
     .transmit_timeout_ms = 2U,
     .limits = {.position_min_rad = ROBOT_DM_POSITION_MIN_RAD,
                .position_max_rad = ROBOT_DM_POSITION_MAX_RAD,
                .velocity_min_rad_per_s = ROBOT_DM_VELOCITY_MIN_RAD_PER_S,
                .velocity_max_rad_per_s = ROBOT_DM_VELOCITY_MAX_RAD_PER_S,
                .torque_min_nm = ROBOT_DM_TORQUE_MIN_NM, .torque_max_nm = ROBOT_DM_TORQUE_MAX_NM,
                .proportional_gain_min = ROBOT_DM_KP_MIN, .proportional_gain_max = ROBOT_DM_KP_MAX,
                .derivative_gain_min = ROBOT_DM_KD_MIN, .derivative_gain_max = ROBOT_DM_KD_MAX}},
};

/* ======================== 中断回调桥接 ======================== */

/** @brief 龙门总线反馈帧 → DM 总线路由（ISR 上下文） */
static void robot_gantry_can_frame_handler(const bsp_can_frame_t *frame)
{
    (void)module_dm_motor_bus_handle_feedback(&s_robot.gantry_bus, frame);
}

/** @brief 机械臂总线反馈帧 → DM 总线路由（ISR 上下文） */
static void robot_arm_can_frame_handler(const bsp_can_frame_t *frame)
{
    (void)module_dm_motor_bus_handle_feedback(&s_robot.arm_bus, frame);
}

/** @brief 板间遥控帧 → Robot Link 模块解析（ISR 上下文，复用 ECF module_board_comm） */
static void robot_board_comm_frame_handler(const bsp_can_frame_t *frame)
{
    (void)module_board_comm_handle_frame(&s_robot.board_comm, frame);
}

/** @brief SBUS 帧解析 → i6x 模块（ISR 上下文，仅遥控本地回退用） */
static void robot_sbus_frame_handler(const uint8_t *data, size_t size)
{
    sbus_to_i6x(&i6x_ctrl, data);
    (void)size; /* 处理器已保证 25 字节 */
}

/** @brief module 层状态 → BSP 状态（仅用于初始化失败路径） */
static bsp_status_t robot_motor_status_to_bsp(module_motor_status_t status)
{
    return (status == MODULE_MOTOR_STATUS_OK) ? BSP_STATUS_OK : BSP_STATUS_IO_ERROR;
}

/* ======================== 总线装配 ======================== */

/**
 * @brief 装配一条 Classic 电机总线：FDCAN 实例 → Classic 适配器 → 过滤器 → 启动
 */
static bsp_status_t robot_init_motor_bus(bsp_fdcan_t *fdcan, void *hal_handle,
                                         bsp_fdcan_classic_adapter_t *adapter,
                                         bsp_can_t **out_can)
{
    bsp_status_t status = bsp_fdcan_init(
        fdcan, &(bsp_fdcan_config_t){.device_handle = hal_handle,
                                     .driver_ops = robot_port_fdcan_ops(),
                                     .callback = NULL,
                                     .user_context = NULL});
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    status = bsp_fdcan_classic_adapter_init(
        adapter,
        &(bsp_fdcan_classic_adapter_config_t){.fdcan = fdcan,
                                              .callback = NULL,
                                              .user_context = NULL});
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    *out_can = bsp_fdcan_classic_adapter_as_can(adapter);

    /* 全接收过滤器：所有标准帧进 RX FIFO0（反馈路由在模块层按 ID 匹配） */
    status = bsp_can_configure_filter(
        *out_can,
        &(bsp_can_filter_t){.identifier = 0U, .mask = 0U,
                            .id_type = BSP_CAN_ID_STANDARD,
                            .receive_fifo = BSP_CAN_RX_FIFO_0,
                            .filter_index = 0U});
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    return bsp_can_start(*out_can);
}

/* ======================== 装配 ======================== */

bsp_status_t robot_init(void)
{
    bsp_status_t status;

    /* ---- 1. DWT 计时器 ---- */
    status = bsp_dwt_init(&s_robot.dwt,
                          &(bsp_dwt_config_t){.device_handle = NULL,
                                              .driver_ops = robot_port_dwt_ops()});
    if (status != BSP_STATUS_OK)
    {
        return status;
    }

    /* ---- 2. 电机总线：FDCAN1 龙门 + FDCAN2 机械臂 ---- */
    status = robot_init_motor_bus(&s_fdcan_gantry, &hfdcan1, &s_gantry_adapter,
                                  &s_robot.gantry_can);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    status = robot_init_motor_bus(&s_fdcan_arm, &hfdcan2, &s_arm_adapter, &s_robot.arm_can);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }

    /* ---- 3. 板间总线：FDCAN3 Classic（复用电机总线装配流程） ---- */
    status = robot_init_motor_bus(&s_fdcan_board, &hfdcan3, &s_board_adapter,
                                  &s_robot.board_can);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }

    /* ---- 4. 板间通信模块（Robot Link 协议，ECF 复用组件） ---- */
    if (module_board_comm_init(
            &s_robot.board_comm,
            &(module_board_comm_config_t){.can = s_robot.board_can,
                                          .transmit_base_identifier = ROBOT_BOARD_COMM_TRANSMIT_BASE_ID,
                                          .receive_base_identifier = ROBOT_BOARD_COMM_RECEIVE_BASE_ID,
                                          .transmit_timeout_ms = 2U,
                                          .offline_timeout_ms = ROBOT_BOARD_COMM_OFFLINE_TIMEOUT_MS}) !=
        MODULE_BOARD_COMM_STATUS_OK)
    {
        return BSP_STATUS_IO_ERROR;
    }

    /* ---- 5. 中断路由注册 ---- */
    robot_port_can_rx_handler_set(FDCAN1, robot_gantry_can_frame_handler);
    robot_port_can_rx_handler_set(FDCAN2, robot_arm_can_frame_handler);
    robot_port_can_rx_handler_set(FDCAN3, robot_board_comm_frame_handler);
#if ROBOT_REMOTE_UART_FALLBACK
    robot_port_uart_frame_handler_set(robot_sbus_frame_handler);
#endif

    /* ---- 6. 遥控本地回退串口（板间链路为主，UART 兜底） ---- */
#if ROBOT_REMOTE_UART_FALLBACK
    status = bsp_usart_init(&s_remote_usart,
                            &(bsp_usart_config_t){.device_handle = &huart5,
                                                  .driver_ops = robot_port_usart_ops(),
                                                  .callback = NULL,
                                                  .user_context = NULL});
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    status = robot_port_uart5_start_reception();
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
#endif

    /* ---- 7. DM 总线与电机 ---- */
    status = robot_motor_status_to_bsp(
        module_dm_motor_bus_init(&s_robot.gantry_bus, s_robot.gantry_can, s_gantry_motor_slots,
                                 ROBOT_GANTRY_MOTOR_COUNT, ROBOT_GANTRY_MOTOR_COUNT));
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    status = robot_motor_status_to_bsp(
        module_dm_motor_bus_init(&s_robot.arm_bus, s_robot.arm_can, s_arm_motor_slots,
                                 ROBOT_ARM_MOTOR_COUNT, ROBOT_ARM_MOTOR_COUNT));
    if (status != BSP_STATUS_OK)
    {
        return status;
    }

    for (size_t i = 0U; i < (ROBOT_GANTRY_MOTOR_COUNT + ROBOT_ARM_MOTOR_COUNT); ++i)
    {
        module_motor_status_t motor_status =
            module_dm_motor_init(&s_dm_motors[i], &s_dm_motor_configs[i]);
        if (motor_status != MODULE_MOTOR_STATUS_OK)
        {
            BSP_LOG_ERROR("motor %s init failed (%d)", s_dm_motor_configs[i].name,
                          (int)motor_status);
            return BSP_STATUS_IO_ERROR;
        }
        motor_status = module_dm_motor_bus_register(s_dm_motor_configs[i].motor_bus,
                                                    &s_dm_motors[i]);
        if (motor_status != MODULE_MOTOR_STATUS_OK)
        {
            BSP_LOG_ERROR("motor %s register failed (%d)", s_dm_motor_configs[i].name,
                          (int)motor_status);
            return BSP_STATUS_IO_ERROR;
        }
    }

    /* ---- 8. App 实例：机械臂（机械臂 3 电机 = s_dm_motors[2..4]） ---- */
    s_arm_config = (app_arm_config_t){
        .joint1_motor = &s_dm_motors[2],
        .joint2_motor = &s_dm_motors[3],
        .gripper_motor = &s_dm_motors[4],
        .joint_kp = ROBOT_JOINT_KP,
        .joint_kd = ROBOT_JOINT_KD,
        .gripper_kp = ROBOT_GRIPPER_KP,
        .gripper_kd = ROBOT_GRIPPER_KD,
        .joint_tolerance_rad = 0.05F,
        .gripper_tolerance_rad = 0.05F,
    };
    status = app_arm_init(&s_robot.arm, &s_arm_config);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }

    /* ---- 9. App 实例：安全（输出保持禁止，等待操作者放行） ---- */
    status = app_safety_init(&s_robot.safety, NULL); /* 看门狗暂不启用 */
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    status = app_safety_monitor_init(
        &s_robot.remote_monitor,
        &(app_safety_monitor_config_t){.name = "remote",
                                       .timeout_ms = ROBOT_REMOTE_TIMEOUT_MS,
                                       .required = true,
                                       .offline_callback = NULL,
                                       .online_callback = NULL,
                                       .user_context = NULL});
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    (void)app_safety_register(&s_robot.safety, &s_robot.remote_monitor);

    return BSP_STATUS_OK;
}

void robot_enter_safe_state(void)
{
    app_safety_set_output_enabled(&s_robot.safety, false);
    module_motor_set_output_allowed(false); /* bus_update 将批量失能全部电机 */
}

/* ======================== 访问器 ======================== */

const robot_t *robot_get(void)
{
    return &s_robot;
}

app_arm_t *robot_get_arm(void)
{
    return &s_robot.arm;
}

module_dm_motor_bus_t *robot_get_gantry_bus(void)
{
    return &s_robot.gantry_bus;
}

module_dm_motor_bus_t *robot_get_arm_bus(void)
{
    return &s_robot.arm_bus;
}

module_board_comm_t *robot_get_board_comm(void)
{
    return &s_robot.board_comm;
}

app_safety_t *robot_get_safety(void)
{
    return &s_robot.safety;
}

app_safety_monitor_t *robot_get_remote_monitor(void)
{
    return &s_robot.remote_monitor;
}

bsp_dwt_t *robot_get_dwt(void)
{
    return &s_robot.dwt;
}
