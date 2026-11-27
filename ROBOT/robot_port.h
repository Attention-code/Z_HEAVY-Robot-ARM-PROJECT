/**
 * @file robot_port.h
 * @brief 工程适配层 — HAL 到 ECF BSP 的驱动绑定
 *
 * @note 按 ECF 移植手册约定：本文件只负责 HAL 句柄到 BSP driver ops 的包装
 *       与 HAL 中断回调的绑定，不含任何机器人业务逻辑。
 *       总线划分（均为 Classic CAN 1Mbps）：FDCAN1=龙门电机 FDCAN2=机械臂电机
 *       FDCAN3=板间通信（遥控快照分片）。
 *       RX 采用"中断内直接排空 FIFO → 回调上层"的方式，避免高反馈速率下 FIFO 溢出。
 */

#ifndef ROBOT_PORT_H
#define ROBOT_PORT_H

#include "bsp_can.h"
#include "bsp_common.h"
#include "bsp_dwt.h"
#include "bsp_fdcan.h"
#include "bsp_usart.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief CAN 接收帧处理器（Classic ≤8 字节，在 RX FIFO0 中断上下文中调用） */
typedef void (*robot_port_can_rx_handler_t)(const bsp_can_frame_t *frame);

/** @brief SBUS 帧处理器（UART 空闲事件中断上下文中调用，仅遥控本地回退时使用） */
typedef void (*robot_port_uart_frame_handler_t)(const uint8_t *data, size_t size);

/**
 * @brief 获取 STM32H7 FDCAN driver ops（三条总线共用，按 device_handle 区分）
 */
const bsp_fdcan_driver_ops_t *robot_port_fdcan_ops(void);

/**
 * @brief 获取 STM32H7 USART driver ops（供 bsp_usart_init 使用）
 */
const bsp_usart_driver_ops_t *robot_port_usart_ops(void);

/**
 * @brief 获取 DWT driver ops（供 bsp_dwt_init 使用）
 */
const bsp_dwt_driver_ops_t *robot_port_dwt_ops(void);

/**
 * @brief 注册 CAN 接收帧处理器（Classic 帧）
 * @param can_instance FDCAN1 / FDCAN2 / FDCAN3（CAN 实例地址）
 * @param handler      处理函数，NULL 表示丢弃
 */
void robot_port_can_rx_handler_set(void *can_instance, robot_port_can_rx_handler_t handler);

/**
 * @brief 注册 SBUS 帧处理器（robot_config 装配时调用一次）
 * @param handler 处理函数，NULL 表示丢弃
 */
void robot_port_uart_frame_handler_set(robot_port_uart_frame_handler_t handler);

/**
 * @brief 启动遥控 UART（UART5）DMA 空闲接收（仅遥控本地回退启用时调用）
 * @return BSP_STATUS_OK 表示接收已启动
 */
bsp_status_t robot_port_uart5_start_reception(void);

/* 板间联调诊断：各 FDCAN RX FIFO0 实际取到的帧计数
   [0]=FDCAN1 [1]=FDCAN2 [2]=FDCAN3，ISR 递增，任务侧只读 */
extern volatile uint32_t g_fdcan_rx_frame_count[3];

#ifdef __cplusplus
}
#endif
#endif /* ROBOT_PORT_H */
