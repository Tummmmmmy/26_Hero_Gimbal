/**
 * @file video_bridge_platform.h
 * @brief 图传桥接平台接口(UART DMA / 时基 / 0x0310 发送)
 */

#ifndef BRIDGE_PLATFORM_H
#define BRIDGE_PLATFORM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** @brief 启动 UART DMA 接收, 写入指定缓冲区 */
void bridge_platform_uart_start_rx_dma(uint8_t* buffer, size_t size);
/** @brief 获取 DMA 当前写指针(环形缓冲位置) */
size_t bridge_platform_uart_rx_write_index(void);
/** @brief 获取系统毫秒计时 */
uint32_t bridge_platform_tick_ms(void);
/** @brief 以自定义 0x0310 帧发送一包数据 */
bool bridge_platform_send_custom_0310(const uint8_t* data, uint16_t size);

/** @brief 重置图传桥接(重新启动 UART DMA 接收) */
void Video_Bridge_Reset(void);

/** @brief 输出调试日志 */
void bridge_platform_debug_log(const char* message);

#endif
