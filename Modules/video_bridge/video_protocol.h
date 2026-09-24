/**
 * @file video_protocol.h
 * @brief 图传链路二进制报文格式定义
 *
 * 与视觉小电脑约定的包结构(帧头/类型/编解码/分片), 收发两侧共用。
 */

#ifndef VIDEO_PROTOCOL_H
#define VIDEO_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define VIDEO_MAGIC 0x4856u
#define VIDEO_VERSION 1u
#define VIDEO_HEADER_SIZE 16u
#define VIDEO_MAX_PACKET_SIZE 300u
#define VIDEO_MAX_PAYLOAD_SIZE (VIDEO_MAX_PACKET_SIZE - VIDEO_HEADER_SIZE)

/** @brief 报文类型 */
typedef enum {
    VIDEO_PACKET_CONFIG = 1,
    VIDEO_PACKET_STREAM_CHUNK = 2,
    VIDEO_PACKET_HEARTBEAT = 3,
    VIDEO_PACKET_RESET = 4,
} video_packet_type_t;

/** @brief 编码格式 ID */
typedef enum {
    VIDEO_CODEC_H264_ANNEXB = 1,
    VIDEO_CODEC_HEVC_ANNEXB = 2,
    VIDEO_CODEC_MJPEG = 3,
} video_codec_id_t;

/** @brief 报文标志位 */
enum {
    VIDEO_FLAG_KEYFRAME_HINT = 1u << 0,
    VIDEO_FLAG_CONFIG_REPEAT = 1u << 1,
    VIDEO_FLAG_DISCONTINUITY = 1u << 2,
};

#pragma pack(push, 1)
/** @brief 报文头(16 字节, 1 字节对齐) */
typedef struct {
    uint16_t magic;
    uint8_t version;
    uint8_t packet_type;
    uint8_t stream_id;
    uint8_t codec;
    uint8_t flags;
    uint8_t reserved0;
    uint32_t seq;
    uint16_t payload_len;
    uint16_t packet_crc16;
} video_packet_header_v1_t;

/** @brief 编码参数配置包 */
typedef struct {
    uint16_t width;
    uint16_t height;
    uint16_t fps;
    uint16_t target_bytes_per_sec;
} video_config_v1_t;

/** @brief 心跳包(发送端统计) */
typedef struct {
    uint32_t captured_frames;
    uint32_t encoded_frames;
    uint32_t sent_packets;
    uint32_t dropped_packets;
} video_heartbeat_v1_t;
#pragma pack(pop)

/** @brief 检查报文头魔数/版本/长度是否合法 */
static inline bool video_header_looks_valid(const video_packet_header_v1_t* header) {
    return header->magic == VIDEO_MAGIC &&
           header->version == VIDEO_VERSION &&
           header->payload_len <= VIDEO_MAX_PAYLOAD_SIZE;
}

#endif
