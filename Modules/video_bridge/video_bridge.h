/**
 * @file video_bridge.h
 * @brief 图传桥接的上下文与统计结构
 */

#ifndef BRIDGE_H
#define BRIDGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** @brief 图传桥接统计计数 */
typedef struct {
    uint32_t uart_bytes_in;
    uint32_t packets_ok;
    uint32_t packets_crc_error;
    uint32_t packets_length_error;
    uint32_t packets_queue_dropped;
    uint32_t packets_sent_0310;
    uint32_t packets_send_failed;
} bridge_stats_t;

/** @brief 图传桥接运行上下文(收发缓冲、分包队列、统计) */
typedef struct {
    uint8_t rx_dma_buffer[2048];
    uint8_t parse_buffer[300];
    uint8_t queue_storage[50][300];
    uint16_t queue_lengths[50];
    uint8_t queue_priority[50];
    uint8_t queue_head;
    uint8_t queue_tail;
    uint8_t queue_size;
    uint16_t parse_size;
    size_t rx_read_index;
    uint32_t last_send_ms;
    bridge_stats_t stats;
} bridge_context_t;

/** @brief 初始化桥接上下文并启动 UART DMA 接收 */
void bridge_init(bridge_context_t* context);
/** @brief 主循环轮询: 从 DMA 环形缓冲取字节并解析入队 */
void bridge_poll(bridge_context_t* context);
/** @brief 20ms 周期任务: 出队一包并以 0x0310 发送 */
void bridge_on_20ms_tick(bridge_context_t* context);
/** @brief 获取统计快照(只读) */
const bridge_stats_t* bridge_stats(const bridge_context_t* context);

#endif
