#ifndef _PID_H
#define _PID_H

#define INLIMIT_RATE 50   //速度积分限幅
#define INLIMIT_ANGLE 30  // 角度积分限幅

#define ANGLE_PID_RATE 250                   //角度环频率
#define ANGLE_PID_DT (1.0 / ANGLE_PID_RATE)  //角度环周期
#define RATE_PID_RATE 500                    //角速度环频率
#define RATE_PID_DT (1.0 / RATE_PID_RATE)    //角速度环周期

typedef struct  //定义记录每个方向pid各量的结构
{
    float desired;
    float error;
    float preerr;
    float kp;
    float ki;
    float kd;
    float dt;
    float integ;
    float deriv;
    //	biquadFilter_t dFilter;
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

int   LIM(double value);
void  controlinit(control_t* c);
void  pidinit(PID_type* pid, float dt, float cutoffFreq);
void  attitudecontrol(control_t* control, set_t* setpoint, Attitude* state, int time, sensor_t* sensor);
float anglePID(PID_type* pid, float state, float angledesired, float dt);
float ratePID(PID_type* pid, float state, float angledesired, float dt);
#endif
