#pragma once
#include "filter.h"

typedef struct  //定义记录每个方向pid各量的结构
{
    float          desired;
    float          error;
    float          preerr;
    float          kp;
    float          ki;
    float          kd;
    float          dt;
    float          integ;
    float          deriv;
    biquadFilter_t dFilter;
} PID_type;

typedef struct  //最终的混控设定值
{
    float yaw;
    float pitch;
    float roll;
    float throttle;
} control_t;

typedef struct  //期望的角度值
{
    float yaw;
    float pitch;
    float roll;
} desired_t;

typedef union
{
    struct
    {
        float x;
        float y;
        float z;
    };
    float axis[3];
} Axis3f;

typedef struct  //传感器数据，用于角速度环
{
    Axis3f acc;   //各方向重力加速度分量
    Axis3f gyro;  //各方向的角速度
    Axis3f mag;   //磁力计参数
} sensor_t;

typedef struct
{
    float throttle;
    float yaw;
    float pitch;
    float roll;
} set_t;

typedef struct attitude  //飞行器当前各方向角度值，用于角度环
{
    float pitch;
    float roll;
    float yaw;
} Attitude;

extern PID_type PID_angle_roll;
extern PID_type PID_angle_pitch;
extern PID_type PID_angle_yaw;
extern PID_type PID_rate_roll;
extern PID_type PID_rate_pitch;
extern PID_type PID_rate_yaw;
