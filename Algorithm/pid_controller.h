#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct
{
    float Kp, Ki, Kd;

    float MaxOutput;
    float Integral_Limit;

    float integral_separation_threshold;
    float dead_band;

    float target;
    float measured;
    float error;
    float pre_error;
    float pre_measured;
    float err_beforeLast;

    float p_out;
    float i_out;
    float d_out;
    float output;
} PID_Controller;

void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float Integral_Limit, float MaxOutput);

float PID_Calculate(PID_Controller *pid, float target, float measured);

float PID_Cal_Enhanced(PID_Controller *pid, float target, float measured);

float PID_Cal_Increment(PID_Controller *pid, float target, float measured);

#endif
