/**
 * @file robot_config.h
 * @brief 工程适配层 — 整机参数总表（CAN ID、机械限位、控制参数、遥控映射）
 *
 * @note 所有需要上机核对/调整的参数都集中在此。
 *       【TODO-上机核对】标记的参数必须按实际硬件修改后再带电测试。
 */

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "app_arm.h"
#include "app_safety.h"
#include "bsp_dwt.h"
#include "bsp_fdcan_classic_adapter.h"
#include "bsp_usart.h"
#include "module_board_comm.h"
#include "module_dm_motor.h"
#include "module_dm_motor_bus.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 遥控来源与映射参数 ======================== */

/** 遥控来源：1 = 板间 Robot Link（正式方案）；UART5 本地 i6x 仅作调试回退 */
#define ROBOT_REMOTE_BOARD_SOURCE (1)
#define ROBOT_REMOTE_UART_FALLBACK (1) /* 板间链路失联时允许本地接收机兜底（调试期） */

/** 【TODO-上机核对】i6x 拨杆数组下标：左/右拨杆对应 s[] 的索引 */
#define ROBOT_REMOTE_LEFT_SWITCH_INDEX (0)
#define ROBOT_REMOTE_RIGHT_SWITCH_INDEX (1)

#define ROBOT_REMOTE_CHANNEL_MAX (660)   /* i6x 模块已把通道映射到 ±660 */
#define ROBOT_REMOTE_DEADBAND (30)       /* 摇杆死区（原始通道值） */
#define ROBOT_REMOTE_TIMEOUT_MS (100U)   /* 遥控失联判定超时 */

/** 联调开关：1 = 经 RTT 打印遥控快照通道值，变化即打印 + 静止心跳（验证完毕置 0） */
#define ROBOT_REMOTE_DEBUG_LOG (1)
#define ROBOT_REMOTE_LOG_MIN_INTERVAL_MS (20U) /* 变化触发的最小打印间隔（ms），防刷屏 */
#define ROBOT_REMOTE_LOG_HEARTBEAT_MS    (500U) /* 遥控静止时的心跳打印周期（ms） */

/* ======================== 板间通信（Robot Link 协议） ======================== */

/**
 * 【TODO-上机核对】板间 Robot Link 基址。
 * 底盘板用 module_board_comm_send_remote() 发送（其发送基址须 = 本板接收基址），
 * 遥控数据占 4 帧：0x710 主通道 / 0x711 辅助通道 / 0x712 输入事件（鼠标+在线位）/
 * 0x713 原始开关（i6x SWA~SWD 档位 +1/0/-1 直传，各占 1 字节）。
 * 本板发送基址 0x720 预留给机械臂状态回传底盘板。
 */
#define ROBOT_BOARD_COMM_RECEIVE_BASE_ID (0x710U)
#define ROBOT_BOARD_COMM_TRANSMIT_BASE_ID (0x720U)
#define ROBOT_BOARD_COMM_OFFLINE_TIMEOUT_MS (40U) /* 板间掉线判定：SBUS 帧率 ~70Hz，
                                                     允许连丢 2~3 组后即断使能 */

/* ======================== 机械臂关节参数 ======================== */

/** 【TODO-上机核对】关节1（肩）行程与速率 */
#define ROBOT_J1_MIN_RAD (-3.0F)
#define ROBOT_J1_MAX_RAD (3.0F)
#define ROBOT_J1_MAX_RATE_RAD_PER_S (2.0F)
#define ROBOT_J1_DEFAULT_RAD (0.0F)

/** 【TODO-上机核对】关节2（肘）行程与速率 */
#define ROBOT_J2_MIN_RAD (-3.0F)
#define ROBOT_J2_MAX_RAD (3.0F)
#define ROBOT_J2_MAX_RATE_RAD_PER_S (2.0F)
#define ROBOT_J2_DEFAULT_RAD (0.0F)

/** 【TODO-上机核对】夹爪开/合目标角与速率（MIT kp 保持） */
#define ROBOT_GRIPPER_OPEN_RAD (0.0F)
#define ROBOT_GRIPPER_CLOSE_RAD (-1.0F)

/* ======================== DM 电机 MIT 增益 ======================== */

/** 【TODO-上机核对】关节 MIT 增益（首次带电先用小值，逐步加大） */
#define ROBOT_JOINT_KP (20.0F)
#define ROBOT_JOINT_KD (1.0F)
/** 【TODO-上机核对】夹爪 MIT 增益（夹持力由 kp 决定） */
#define ROBOT_GRIPPER_KP (50.0F)
#define ROBOT_GRIPPER_KD (1.5F)

/* ======================== DM4310 协议限幅 ======================== */

/** 【TODO-上机核对】按实际电机型号与上位机参数页填写 */
#define ROBOT_DM_POSITION_MIN_RAD (-12.5F)
#define ROBOT_DM_POSITION_MAX_RAD (12.5F)
#define ROBOT_DM_VELOCITY_MIN_RAD_PER_S (-30.0F)
#define ROBOT_DM_VELOCITY_MAX_RAD_PER_S (30.0F)
#define ROBOT_DM_TORQUE_MIN_NM (-10.0F)
#define ROBOT_DM_TORQUE_MAX_NM (10.0F)
#define ROBOT_DM_KP_MIN (0.0F)
#define ROBOT_DM_KP_MAX (500.0F)
#define ROBOT_DM_KD_MIN (0.0F)
#define ROBOT_DM_KD_MAX (5.0F)

/* ======================== 整机对象总表 ======================== */

/** @brief 龙门电机数量（FDCAN1 总线） */
#define ROBOT_GANTRY_MOTOR_COUNT (2U)
/** @brief 机械臂电机数量（FDCAN2 总线：2 关节 + 1 夹爪） */
#define ROBOT_ARM_MOTOR_COUNT (3U)

/** @brief 整机装配对象（只读，经访问器使用） */
typedef struct
{
    bsp_dwt_t dwt;                                  /**< DWT 计时器 */
    bsp_can_t *gantry_can;                          /**< 龙门总线（FDCAN1 经 Classic 适配器） */
    bsp_can_t *arm_can;                             /**< 机械臂总线（FDCAN2 经 Classic 适配器） */
    bsp_can_t *board_can;                           /**< 板间总线（FDCAN3 经 Classic 适配器） */
    module_board_comm_t board_comm;                 /**< 板间通信（Robot Link 协议） */
    module_dm_motor_bus_t gantry_bus;               /**< 龙门 DM 电机总线 */
    module_dm_motor_bus_t arm_bus;                  /**< 机械臂 DM 电机总线 */
    app_arm_t arm;                                  /**< 机械臂应用 */
    app_safety_t safety;                            /**< 安全模块 */
    app_safety_monitor_t remote_monitor;            /**< 遥控心跳监控器 */
} robot_t;

/**
 * @brief  整机装配：BSP 实例 → DM 总线与 5 电机 → App 实例 → 安全监控注册。
 * @note   初始化失败保持电机失能并返回错误，由调用方决定是否继续。
 */
bsp_status_t robot_init(void);

/** @brief 进入安全状态：关闭整机输出门（模块层电机会被 bus_update 批量失能） */
void robot_enter_safe_state(void);

/** @brief 访问器：整机对象与子模块 */
const robot_t *robot_get(void);
app_arm_t *robot_get_arm(void);
module_dm_motor_bus_t *robot_get_gantry_bus(void);
module_dm_motor_bus_t *robot_get_arm_bus(void);
module_board_comm_t *robot_get_board_comm(void);
app_safety_t *robot_get_safety(void);
app_safety_monitor_t *robot_get_remote_monitor(void);
bsp_dwt_t *robot_get_dwt(void);

#ifdef __cplusplus
}
#endif
#endif /* ROBOT_CONFIG_H */
