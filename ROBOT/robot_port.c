/**
 * @file robot_port.c
 * @brief 工程适配层 — STM32H7 HAL 到 ECF BSP driver ops 的实现
 *
 * @note FDCAN1 = 电机总线（Classic CAN 1Mbps，CubeMX 已配好 8 深 RX FIFO0 / 8 深 TX FIFO），
 *       UART5 = i6x 遥控 SBUS 输入（100000 波特，DMA 空闲接收），
 *       DWT = 内核周期计数器（控制周期 dt 计时）。
 *       RX 采用"中断内直接排空 FIFO → 回调上层"的方式，避免 1kHz 反馈速率下 FIFO 溢出。
 */

#include "robot_port.h"

#include "fdcan.h"    /* hfdcan1 */
#include "main.h"     /* SystemCoreClock */
#include "usart.h"    /* huart5 */
#include "stm32h7xx_hal.h"

#include <stddef.h>

/* ======================== 内部状态 ======================== */

static volatile robot_port_can_rx_handler_t s_can_rx_handler_fdcan1; /* 龙门总线反馈 */
static volatile robot_port_can_rx_handler_t s_can_rx_handler_fdcan2; /* 机械臂总线反馈 */
static volatile robot_port_can_rx_handler_t s_can_rx_handler_fdcan3; /* 板间快照分片 */
static volatile robot_port_uart_frame_handler_t s_uart_frame_handler; /* SBUS 帧处理器 */

/* 板间联调诊断：各 FDCAN RX FIFO0 实际取到的帧计数（ISR 递增）
   rx[0]=FDCAN1 rx[1]=FDCAN2 rx[2]=FDCAN3，用于判断帧到底进了哪条总线 */
volatile uint32_t g_fdcan_rx_frame_count[3];

/** @brief UART5 DMA 空闲接收缓冲（32 > 25 字节 SBUS 帧，半传输事件不早于帧尾） */
static uint8_t s_uart5_rx_buf[32];
#define UART5_RX_BUFFER_SIZE (sizeof(s_uart5_rx_buf))
#define SBUS_FRAME_LENGTH (25U)

/* ======================== FDCAN driver ops ======================== */

/** @brief 数据长度 → FDCAN DLC 编码（支持 CANFD 12/16/20/24/32/48/64） */
static uint32_t robot_fdcan_dlc_from_length(uint8_t length)
{
    if (length <= 8U)
    {
        static const uint32_t dlc_table[9] = {
            FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3,
            FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7,
            FDCAN_DLC_BYTES_8,
        };
        return dlc_table[length];
    }
    if (length <= 12U) { return FDCAN_DLC_BYTES_12; }
    if (length <= 16U) { return FDCAN_DLC_BYTES_16; }
    if (length <= 20U) { return FDCAN_DLC_BYTES_20; }
    if (length <= 24U) { return FDCAN_DLC_BYTES_24; }
    if (length <= 32U) { return FDCAN_DLC_BYTES_32; }
    if (length <= 48U) { return FDCAN_DLC_BYTES_48; }
    return FDCAN_DLC_BYTES_64;
}

/** @brief FDCAN DLC → 实际数据字节数（CANFD DLC 9..15 → 12..64）
 *  @note  STM32H7 HAL 的 GetRxMessage 返回的是原始 DLC 值（0~15，见
 *         hal_fdcan.c: DataLength = (*RxAddress & MASK_DLC) >> 16），
 *         直接按低 4 位查表，不再右移 16 位 */
static uint8_t robot_fdcan_length_from_dlc(uint32_t dlc)
{
    static const uint8_t length_table[16] = {0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U,
                                             8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U};
    return length_table[dlc & 0x0FU];
}

/** @brief bsp_can FIFO 枚举 → HAL FIFO 枚举 */
static uint32_t robot_fdcan_hal_fifo(bsp_can_receive_fifo_t fifo)
{
    return (fifo == BSP_CAN_RX_FIFO_0) ? FDCAN_RX_FIFO0 : FDCAN_RX_FIFO1;
}

/**
 * @brief 启动 FDCAN：HAL_Start + 激活 RX FIFO0 中断 + 非匹配帧进 FIFO0
 */
static bsp_status_t robot_fdcan_ops_start(void *device_handle)
{
    FDCAN_HandleTypeDef *const hfdcan = (FDCAN_HandleTypeDef *)device_handle;

    if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
    {
        return BSP_STATUS_IO_ERROR;
    }
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK)
    {
        return BSP_STATUS_IO_ERROR;
    }
    /* 未被过滤器匹配的帧也进 RX FIFO0，简化调试；远程帧不拒收 */
    (void)HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0,
                                       FDCAN_RX_FIFO0, DISABLE, DISABLE);
    return BSP_STATUS_OK;
}

static bsp_status_t robot_fdcan_ops_stop(void *device_handle)
{
    return (HAL_FDCAN_Stop((FDCAN_HandleTypeDef *)device_handle) == HAL_OK)
               ? BSP_STATUS_OK
               : BSP_STATUS_IO_ERROR;
}

/**
 * @brief 配置硬件滤波器：标准/扩展 ID，范围或精确匹配，进指定 FIFO
 */
static bsp_status_t robot_fdcan_ops_configure_filter(void *device_handle,
                                                     const bsp_can_filter_t *filter)
{
    FDCAN_HandleTypeDef *const hfdcan = (FDCAN_HandleTypeDef *)device_handle;
    FDCAN_FilterTypeDef hal_filter = {0};

    if (filter == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }

    hal_filter.IdType = (filter->id_type == BSP_CAN_ID_STANDARD) ? FDCAN_STANDARD_ID
                                                                 : FDCAN_EXTENDED_ID;
    if (filter->mask == 0U)
    {
        /* 掩码为 0 = 接收所有该类型帧 */
        hal_filter.FilterType = FDCAN_FILTER_RANGE;
        hal_filter.FilterID1 = 0U;
        hal_filter.FilterID2 = (filter->id_type == BSP_CAN_ID_STANDARD) ? 0x7FFU : 0x1FFFFFFFU;
    }
    else
    {
        /* 掩码模式：FilterID1 = 标识符，FilterID2 = 掩码 */
        hal_filter.FilterType = FDCAN_FILTER_MASK;
        hal_filter.FilterID1 = filter->identifier;
        hal_filter.FilterID2 = filter->mask;
    }
    hal_filter.FilterConfig = (filter->receive_fifo == BSP_CAN_RX_FIFO_0)
                                  ? FDCAN_FILTER_TO_RXFIFO0
                                  : FDCAN_FILTER_TO_RXFIFO1;

    return (HAL_FDCAN_ConfigFilter(hfdcan, &hal_filter) == HAL_OK) ? BSP_STATUS_OK
                                                                   : BSP_STATUS_IO_ERROR;
}

/**
 * @brief 发送一帧 Classic CAN（等待 TX FIFO 空闲，带超时）
 */
static bsp_status_t robot_fdcan_ops_transmit(void *device_handle,
                                             const bsp_fdcan_frame_t *frame,
                                             uint32_t timeout_ms)
{
    FDCAN_HandleTypeDef *const hfdcan = (FDCAN_HandleTypeDef *)device_handle;
    FDCAN_TxHeaderTypeDef tx_header = {0};
    uint32_t start_tick;

    if ((frame == NULL) || (frame->data_length > 64U))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }

    /* 等待 TX FIFO 空闲 */
    start_tick = HAL_GetTick();
    while (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0U)
    {
        if ((HAL_GetTick() - start_tick) > timeout_ms)
        {
            return BSP_STATUS_TIMEOUT;
        }
    }

    tx_header.Identifier = frame->identifier;
    tx_header.IdType = (frame->id_type == BSP_CAN_ID_STANDARD) ? FDCAN_STANDARD_ID
                                                               : FDCAN_EXTENDED_ID;
    tx_header.TxFrameType = (frame->frame_type == BSP_CAN_FRAME_DATA) ? FDCAN_DATA_FRAME
                                                                      : FDCAN_REMOTE_FRAME;
    tx_header.DataLength = robot_fdcan_dlc_from_length(frame->data_length);
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = (frame->format == BSP_FDCAN_FORMAT_FD_BRS) ? FDCAN_BRS_ON
                                                                         : FDCAN_BRS_OFF;
    tx_header.FDFormat = (frame->format == BSP_FDCAN_FORMAT_CLASSIC) ? FDCAN_CLASSIC_CAN
                                                                     : FDCAN_FD_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0U;

    return (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, (uint8_t *)frame->data) == HAL_OK)
               ? BSP_STATUS_OK
               : BSP_STATUS_IO_ERROR;
}

/**
 * @brief 从指定 RX FIFO 读取一帧（仅 Classic 帧需要经过适配器转换）
 */
static bsp_status_t robot_fdcan_ops_receive(void *device_handle,
                                            bsp_can_receive_fifo_t fifo,
                                            bsp_fdcan_frame_t *frame)
{
    FDCAN_HandleTypeDef *const hfdcan = (FDCAN_HandleTypeDef *)device_handle;
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[64];

    if (frame == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, robot_fdcan_hal_fifo(fifo)) == 0U)
    {
        return BSP_STATUS_IO_ERROR; /* 无数据 */
    }
    if (HAL_FDCAN_GetRxMessage(hfdcan, robot_fdcan_hal_fifo(fifo), &rx_header, rx_data) != HAL_OK)
    {
        return BSP_STATUS_IO_ERROR;
    }

    frame->identifier = rx_header.Identifier;
    frame->id_type = (rx_header.IdType == FDCAN_STANDARD_ID) ? BSP_CAN_ID_STANDARD
                                                             : BSP_CAN_ID_EXTENDED;
    frame->frame_type = (rx_header.RxFrameType == FDCAN_DATA_FRAME) ? BSP_CAN_FRAME_DATA
                                                                    : BSP_CAN_FRAME_REMOTE;
    frame->format = (rx_header.FDFormat == FDCAN_CLASSIC_CAN) ? BSP_FDCAN_FORMAT_CLASSIC
                  : (rx_header.BitRateSwitch != 0U)           ? BSP_FDCAN_FORMAT_FD_BRS
                                                              : BSP_FDCAN_FORMAT_FD_NO_BRS;
    frame->data_length = robot_fdcan_length_from_dlc(rx_header.DataLength);
    if (frame->data_length > 64U)
    {
        frame->data_length = 64U;
    }
    for (uint8_t i = 0U; i < frame->data_length; ++i)
    {
        frame->data[i] = rx_data[i];
    }
    return BSP_STATUS_OK;
}

/**
 * @brief 查询 TX FIFO 空闲等级
 */
static bsp_status_t robot_fdcan_ops_get_tx_free_level(const void *device_handle,
                                                      uint32_t *free_level)
{
    if (free_level == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *free_level = HAL_FDCAN_GetTxFifoFreeLevel((FDCAN_HandleTypeDef *)device_handle);
    return BSP_STATUS_OK;
}

static const bsp_fdcan_driver_ops_t s_robot_fdcan_ops = {
    .init = NULL, /* CubeMX MX_FDCANx_Init 已完成初始化 */
    .deinit = NULL,
    .start = robot_fdcan_ops_start,
    .stop = robot_fdcan_ops_stop,
    .configure_filter = robot_fdcan_ops_configure_filter,
    .transmit = robot_fdcan_ops_transmit,
    .receive = robot_fdcan_ops_receive,
    .get_protocol_status = NULL,
    .get_transmit_free_level = robot_fdcan_ops_get_tx_free_level,
};

const bsp_fdcan_driver_ops_t *robot_port_fdcan_ops(void)
{
    return &s_robot_fdcan_ops;
}

/**
 * @brief FDCAN RX FIFO0 中断回调（覆盖 HAL weak 定义）
 * @note 中断内排空 FIFO 并逐帧上交，防止高反馈速率下 FIFO 溢出；
 *       三条总线均为 Classic，按实例路由到各自的处理器。
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
    {
        return;
    }
    if ((hfdcan->Instance != FDCAN1) && (hfdcan->Instance != FDCAN2) &&
        (hfdcan->Instance != FDCAN3))
    {
        return;
    }
    (void)HAL_FDCAN_DeactivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);

    robot_port_can_rx_handler_t handler;
    uint32_t rx_count_index;
    if (hfdcan->Instance == FDCAN1)
    {
        handler = s_can_rx_handler_fdcan1;
        rx_count_index = 0U;
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        handler = s_can_rx_handler_fdcan2;
        rx_count_index = 1U;
    }
    else
    {
        handler = s_can_rx_handler_fdcan3;
        rx_count_index = 2U;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
    {
        FDCAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[64];
        bsp_can_frame_t frame;

        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        {
            break;
        }
        if (handler != NULL)
        {
            frame.identifier = rx_header.Identifier;
            frame.id_type = (rx_header.IdType == FDCAN_STANDARD_ID) ? BSP_CAN_ID_STANDARD
                                                                    : BSP_CAN_ID_EXTENDED;
            frame.frame_type = (rx_header.RxFrameType == FDCAN_DATA_FRAME)
                                   ? BSP_CAN_FRAME_DATA
                                   : BSP_CAN_FRAME_REMOTE;
            frame.data_length = robot_fdcan_length_from_dlc(rx_header.DataLength);
            if (frame.data_length > 8U)
            {
                frame.data_length = 8U;
            }
            for (uint8_t i = 0U; i < frame.data_length; ++i)
            {
                frame.data[i] = rx_data[i];
            }
            g_fdcan_rx_frame_count[rx_count_index]++;
            handler(&frame);
        }
        else
        {
            g_fdcan_rx_frame_count[rx_count_index]++; /* 无处理器也计数：帧确实到达 */
        }
    }
    (void)HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U);
}

/* ======================== USART driver ops ======================== */

static bsp_status_t robot_usart_ops_transmit(void *device_handle, const uint8_t *data,
                                             size_t size, bsp_transfer_mode_t mode,
                                             uint32_t timeout_ms)
{
    UART_HandleTypeDef *const huart = (UART_HandleTypeDef *)device_handle;
    HAL_StatusTypeDef status;
    switch (mode)
    {
    case BSP_TRANSFER_MODE_BLOCKING:
        status = HAL_UART_Transmit(huart, (uint8_t *)data, (uint16_t)size, timeout_ms);
        break;
    case BSP_TRANSFER_MODE_INTERRUPT:
        status = HAL_UART_Transmit_IT(huart, (uint8_t *)data, (uint16_t)size);
        break;
    case BSP_TRANSFER_MODE_DMA:
        status = HAL_UART_Transmit_DMA(huart, (uint8_t *)data, (uint16_t)size);
        break;
    default:
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return (status == HAL_OK) ? BSP_STATUS_OK : BSP_STATUS_IO_ERROR;
}

static bsp_status_t robot_usart_ops_receive(void *device_handle, uint8_t *data, size_t size,
                                            bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    UART_HandleTypeDef *const huart = (UART_HandleTypeDef *)device_handle;
    HAL_StatusTypeDef status;
    switch (mode)
    {
    case BSP_TRANSFER_MODE_BLOCKING:
        status = HAL_UART_Receive(huart, data, (uint16_t)size, timeout_ms);
        break;
    case BSP_TRANSFER_MODE_INTERRUPT:
        status = HAL_UART_Receive_IT(huart, data, (uint16_t)size);
        break;
    case BSP_TRANSFER_MODE_DMA:
        status = HAL_UART_Receive_DMA(huart, data, (uint16_t)size);
        break;
    default:
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return (status == HAL_OK) ? BSP_STATUS_OK : BSP_STATUS_IO_ERROR;
}

/**
 * @brief 空闲线接收：DMA 模式使用 HAL_UARTEx_ReceiveToIdle_DMA
 */
static bsp_status_t robot_usart_ops_receive_to_idle(void *device_handle, uint8_t *data,
                                                    size_t capacity, bsp_transfer_mode_t mode,
                                                    uint32_t timeout_ms)
{
    UART_HandleTypeDef *const huart = (UART_HandleTypeDef *)device_handle;
    HAL_StatusTypeDef status;
    switch (mode)
    {
    case BSP_TRANSFER_MODE_BLOCKING:
    {
        uint16_t received_length = 0U;
        status = HAL_UARTEx_ReceiveToIdle(huart, data, (uint16_t)capacity, &received_length,
                                          timeout_ms);
        break;
    }
    case BSP_TRANSFER_MODE_INTERRUPT:
        status = HAL_UARTEx_ReceiveToIdle_IT(huart, data, (uint16_t)capacity);
        break;
    case BSP_TRANSFER_MODE_DMA:
        status = HAL_UARTEx_ReceiveToIdle_DMA(huart, data, (uint16_t)capacity);
        break;
    default:
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return (status == HAL_OK) ? BSP_STATUS_OK : BSP_STATUS_IO_ERROR;
}

static bsp_status_t robot_usart_ops_abort(void *device_handle)
{
    return (HAL_UART_Abort((UART_HandleTypeDef *)device_handle) == HAL_OK) ? BSP_STATUS_OK
                                                                           : BSP_STATUS_IO_ERROR;
}

static bsp_status_t robot_usart_ops_get_busy(const void *device_handle, bool *is_busy)
{
    const UART_HandleTypeDef *const huart = (const UART_HandleTypeDef *)device_handle;
    if (is_busy == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *is_busy = (huart->gState != HAL_UART_STATE_READY) || (huart->RxState != HAL_UART_STATE_READY);
    return BSP_STATUS_OK;
}

static const bsp_usart_driver_ops_t s_robot_usart_ops = {
    .init = NULL, /* CubeMX MX_UARTx_Init 已完成初始化 */
    .deinit = NULL,
    .transmit = robot_usart_ops_transmit,
    .receive = robot_usart_ops_receive,
    .receive_to_idle = robot_usart_ops_receive_to_idle,
    .receive_to_idle_double_buffer = NULL,
    .abort = robot_usart_ops_abort,
    .get_busy = robot_usart_ops_get_busy,
};

const bsp_usart_driver_ops_t *robot_port_usart_ops(void)
{
    return &s_robot_usart_ops;
}

/**
 * @brief 在接收缓冲中定位一帧完整 SBUS（0x0F ... 0x00，25 字节）
 */
static void robot_port_parse_sbus(uint16_t received_size)
{
    robot_port_uart_frame_handler_t handler = s_uart_frame_handler;
    if ((handler == NULL) || (received_size < SBUS_FRAME_LENGTH))
    {
        return;
    }
    for (uint16_t i = 0U; (i + SBUS_FRAME_LENGTH) <= received_size; ++i)
    {
        if ((s_uart5_rx_buf[i] == 0x0FU) &&
            (s_uart5_rx_buf[i + SBUS_FRAME_LENGTH - 1U] == 0x00U))
        {
            handler(&s_uart5_rx_buf[i], SBUS_FRAME_LENGTH);
            return; /* 每次空闲事件只取最新一帧 */
        }
    }
}

/**
 * @brief UART 空闲事件回调（覆盖 HAL weak 定义）
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != UART5)
    {
        return;
    }
    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT)
    {
        return; /* 半传输事件不处理，等待空闲帧尾 */
    }
    robot_port_parse_sbus(Size);
    /* 非循环 DMA：空闲事件后需要重新武装接收 */
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart5, s_uart5_rx_buf, (uint16_t)UART5_RX_BUFFER_SIZE);
}

/**
 * @brief UART 错误回调：噪声/溢出错误后重启接收，避免遥控链路死掉
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart5, s_uart5_rx_buf, (uint16_t)UART5_RX_BUFFER_SIZE);
    }
}

bsp_status_t robot_port_uart5_start_reception(void)
{
    return (HAL_UARTEx_ReceiveToIdle_DMA(&huart5, s_uart5_rx_buf,
                                         (uint16_t)UART5_RX_BUFFER_SIZE) == HAL_OK)
               ? BSP_STATUS_OK
               : BSP_STATUS_IO_ERROR;
}

/* ======================== DWT driver ops ======================== */

static bsp_status_t robot_dwt_ops_init(void *device_handle)
{
    (void)device_handle;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; /* 使能调试单元 */
    DWT->LAR = 0xC5ACCE55U;                         /* 解锁 DWT（部分核心需要） */
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            /* 启动周期计数器 */
    return BSP_STATUS_OK;
}

static bsp_status_t robot_dwt_ops_reset(void *device_handle)
{
    (void)device_handle;
    DWT->CYCCNT = 0U;
    return BSP_STATUS_OK;
}

static bsp_status_t robot_dwt_ops_get_cycle_count(const void *device_handle,
                                                  uint32_t *cycle_count)
{
    (void)device_handle;
    if (cycle_count == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *cycle_count = DWT->CYCCNT;
    return BSP_STATUS_OK;
}

static bsp_status_t robot_dwt_ops_get_frequency_hz(const void *device_handle,
                                                   uint32_t *frequency_hz)
{
    (void)device_handle;
    if (frequency_hz == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *frequency_hz = SystemCoreClock;
    return BSP_STATUS_OK;
}

static const bsp_dwt_driver_ops_t s_robot_dwt_ops = {
    .init = robot_dwt_ops_init,
    .reset = robot_dwt_ops_reset,
    .get_cycle_count = robot_dwt_ops_get_cycle_count,
    .get_frequency_hz = robot_dwt_ops_get_frequency_hz,
};

const bsp_dwt_driver_ops_t *robot_port_dwt_ops(void)
{
    return &s_robot_dwt_ops;
}

/* ======================== 处理器注册 ======================== */

void robot_port_can_rx_handler_set(void *can_instance, robot_port_can_rx_handler_t handler)
{
    if (can_instance == FDCAN1)
    {
        s_can_rx_handler_fdcan1 = handler;
    }
    else if (can_instance == FDCAN2)
    {
        s_can_rx_handler_fdcan2 = handler;
    }
    else if (can_instance == FDCAN3)
    {
        s_can_rx_handler_fdcan3 = handler;
    }
}

void robot_port_uart_frame_handler_set(robot_port_uart_frame_handler_t handler)
{
    s_uart_frame_handler = handler;
}
