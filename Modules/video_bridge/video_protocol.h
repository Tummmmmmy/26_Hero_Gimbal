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

typedef enum {
    VIDEO_PACKET_CONFIG = 1,
    VIDEO_PACKET_STREAM_CHUNK = 2,
    VIDEO_PACKET_HEARTBEAT = 3,
    VIDEO_PACKET_RESET = 4,
} video_packet_type_t;

typedef enum {
    VIDEO_CODEC_H264_ANNEXB = 1,
    VIDEO_CODEC_HEVC_ANNEXB = 2,
    VIDEO_CODEC_MJPEG = 3,
} video_codec_id_t;

enum {
    VIDEO_FLAG_KEYFRAME_HINT = 1u << 0,
    VIDEO_FLAG_CONFIG_REPEAT = 1u << 1,
    VIDEO_FLAG_DISCONTINUITY = 1u << 2,
};

#pragma pack(push, 1)
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

typedef struct {
    uint16_t width;
    uint16_t height;
    uint16_t fps;
    uint16_t target_bytes_per_sec;
} video_config_v1_t;

typedef struct {
    uint32_t captured_frames;
    uint32_t encoded_frames;
    uint32_t sent_packets;
    uint32_t dropped_packets;
} video_heartbeat_v1_t;
#pragma pack(pop)

static inline bool video_header_looks_valid(const video_packet_header_v1_t* header) {
    return header->magic == VIDEO_MAGIC &&
           header->version == VIDEO_VERSION &&
           header->payload_len <= VIDEO_MAX_PAYLOAD_SIZE;
}

#endif
