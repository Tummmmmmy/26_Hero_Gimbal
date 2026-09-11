#ifndef SHOOTER_CONTROL_H
#define SHOOTER_CONTROL_H

#include <stdint.h>

typedef enum
{
    SHOOTER_OFF,
    SHOOTER_CALIBRATING,
    SHOOTER_READY,
    SHOOTER_SINGLE_SHOT,
    SHOOTER_JAMMED
} Shooter_State_e;

typedef enum
{
    SHOT_NOT_DETECTED = 0,
    SHOT_DETECTED = 1
} Shot_Detection_e;

typedef struct
{
    Shooter_State_e state;
    float friction_target_speed;
    float friction_linear_ratio;
    uint8_t friction_state;

    float dial_target_rad;
    float dial_last_normal_rad;

    uint8_t single_shot_fired_flag;
    uint16_t continuous_fire_timer;
    uint16_t jam_detect_timer;
    uint16_t jam_timeout_timer;
    uint8_t jam_process_step;
    uint8_t is_calibrated;
    uint8_t homing_step;
    uint16_t homing_timer;
} Shooter_Controller_t;

#endif
