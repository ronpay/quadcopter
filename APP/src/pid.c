#include "pid.h"
#include "motor.h"
#include "ucos_ii.h"

desired_t angledesired;
desired_t ratedesired;
set_t     setpoint;
sensor_t  sensordata;
Attitude  state;
control_t control;

// GY86 所需要的数组
extern float          Acel_mps[3];
extern short          Acel[3];
extern float          Gyro_dps[3];
extern short          Gyro[3];
extern short          Mag[3];
extern volatile float Mag_gs[3];
extern volatile float Temp;
extern volatile float pitch, roll, yaw;

PID_type PID_angle_roll = {
    .kp = 1.0,
    .ki = 0.0,
    .kd = 0.0,
};

PID_type PID_angle_pitch = {
    .kp = 0.0,
    .ki = 0.0,
    .kd = 0.0,
};

PID_type PID_angle_yaw = {
    .kp = 0.0,
    .ki = 0.0,
    .kd = 0.0,
};

PID_type PID_rate_roll = {
    .kp = 1.75,
    .ki = 0.0,
    .kd = 0.0,
};

PID_type PID_rate_pitch = {
    .kp = 0.0,
    .ki = 0.0,
    .kd = 0.0,
};

PID_type PID_rate_yaw = {
    .kp = 0.0,
    .ki = 0.0,
    .kd = 0.0,
};

void pidinit(PID_type* pid, float dt, float cutoffFreq)
{
    pid->error   = 0;
    pid->preerr  = 0;
    pid->desired = 0;
    pid->dt      = dt;
    // biquadFilterInitLPF(&pid->dFilter, (1.0f/dt), cutoffFreq);
}

void controlinit(control_t* c)
{
    pidinit(&PID_angle_roll, ANGLE_PID_DT, 0);
    pidinit(&PID_angle_pitch, ANGLE_PID_DT, 0);
    pidinit(&PID_angle_yaw, ANGLE_PID_DT, 0);
    pidinit(&PID_rate_roll, RATE_PID_DT, 0);
    pidinit(&PID_rate_pitch, RATE_PID_DT, 0);
    pidinit(&PID_rate_yaw, RATE_PID_DT, 0);
    c->pitch    = 0;
    c->roll     = 0;
    c->throttle = 1000;
    c->yaw      = 0;
}
void attitudecontrol(control_t* control, set_t* setpoint, Attitude* state, int time, sensor_t* sensor)
{
    //角度环
    //横滚角和俯仰角直接根据摇杆的状态进行期望值得设定
    angledesired.roll  = setpoint->roll;
    angledesired.pitch = setpoint->pitch;

    //偏航角叠加对应摇杆的状态，并且在1s秒转动摇杆对应的角度
    angledesired.yaw += setpoint->yaw / ANGLE_PID_RATE;
    if (angledesired.yaw > 180.0f)
        angledesired.yaw -= 360.0f;
    if (angledesired.yaw < -180.0f)
        angledesired.yaw += 360.0f;
    // printf("  angledesired yaw:%f\n",angledesired.yaw);
    // printf("  setpoint->yaw: %f\n",setpoint->yaw);

    //角度PID
    ratedesired.roll  = anglePID(&PID_angle_roll, state->roll, angledesired.roll, ANGLE_PID_DT);
    ratedesired.pitch = anglePID(&PID_angle_pitch, state->pitch, angledesired.pitch, ANGLE_PID_DT);
    ratedesired.yaw   = anglePID(&PID_angle_yaw, state->yaw, angledesired.yaw, ANGLE_PID_DT);

    //角速度PID
    control->roll     = ratePID(&PID_rate_roll, sensor->gyro.x, ratedesired.roll, RATE_PID_DT);
    control->pitch    = ratePID(&PID_rate_pitch, sensor->gyro.y, ratedesired.pitch, RATE_PID_DT);
    control->yaw      = ratePID(&PID_rate_yaw, sensor->gyro.z, ratedesired.yaw, RATE_PID_DT);
    control->throttle = setpoint->throttle;
}

float anglePID(PID_type* pid, float state, float angledesired, float dt)
{
    float ans;
    pid->error = angledesired - state;
    //主要针对航向角，因为航向有可能无视极限继续转动，以顺时针转动为例，当由180到达负角度时，其实只正向转动了一点点，但是error却是大于180
    if (pid->error > 180.0f) {
        pid->error -= 360.0f;
    }
    else if (pid->error < -180.0) {
        pid->error += 360.0f;
    }
    pid->integ += pid->error * pid->dt;
    pid->deriv  = (pid->error - pid->preerr) / pid->dt;
    pid->preerr = pid->error;
    //积分限幅
    if (pid->integ > INLIMIT_ANGLE) {
        pid->integ = INLIMIT_ANGLE;
    }
    else if (pid->integ < -INLIMIT_ANGLE) {
        pid->integ = -INLIMIT_ANGLE;
    }

    ans = pid->kp * pid->error + pid->ki * pid->integ;
    // ans=pid->kp*pid->error+pid->ki*pid->integ+pid->kd*pid->deriv;
    return ans;
}

float ratePID(PID_type* pid, float state, float angledesired, float dt)
{
    float ans;
    pid->error = angledesired - state;
    pid->integ += pid->error * pid->dt;
    pid->deriv = (pid->error - pid->preerr) / pid->dt;
    //  x++;
    //	if(x%1000==0)printf("%d\n",x);
    //	mmtt=(pid->error-pid->preerr);

    // printf("%f\n",pid->error);

    //积分限幅
    if (pid->integ > INLIMIT_RATE) {
        pid->integ = INLIMIT_RATE;
    }
    else if (pid->integ < -INLIMIT_RATE) {
        pid->integ = -INLIMIT_RATE;
    }

    //微分滤波
    // pid->deriv = biquadFilterApply(&pid->dFilter, pid->deriv);

    pid->preerr = pid->error;
    ans         = pid->kp * pid->error + pid->ki * pid->integ + pid->kd * pid->deriv;
    return ans;
}

void TASK_PID(void)
{
    int time;  //暂无
    int motor[4];
    controlinit(&control);
    while (1) {
        //获取遥控器信息
        // getsetpoint();

        //获取传感器数据
        // getsensor();

        //进行姿态解算
        sensordata.acc.x = Acel_mps[0];
        sensordata.acc.y = Acel_mps[1];
        sensordata.acc.z = Acel_mps[2];

        sensordata.gyro.x = Gyro_dps[0];
        sensordata.gyro.y = Gyro_dps[1];
        sensordata.gyro.z = Gyro_dps[2];

        sensordata.mag.x = Mag_gs[0];
        sensordata.mag.y = Mag_gs[1];
        sensordata.mag.z = Mag_gs[2];

        state.yaw   = yaw;
        state.roll  = roll;
        state.pitch = pitch;

        //			if(TIME_TODO(ATTITUDE_ESTIMAT_RATE,time)){
        //				imuUpdate(sensordata.acc,sensordata.gyro,&state,ATTITUDE_ESTIMAT_DT);
        //				float x=state.pitch;
        //				state.pitch=state.roll;
        //				state.roll=x;
        //			}

        //计算点击控制
        attitudecontrol(&control, &setpoint, &state, time, &sensordata);
        //设置电机转速
        // MOTOR_control(&control);
        //
        motor[0] = LIM(control.throttle - control.roll - control.pitch + control.yaw);
        motor[1] = LIM(control.throttle - control.roll + control.pitch - control.yaw);
        motor[2] = LIM(control.throttle + control.roll + control.pitch + control.yaw);
        motor[3] = LIM(control.throttle + control.roll - control.pitch - control.yaw);

        //			OSQPost(mm,&remotor[ptr]);
        //			ptr=(ptr+1)%128;
        if (1) {
            TIM_SetCompare1(TIM2, motor[0]);  // 如果电机解锁
            TIM_SetCompare2(TIM2, motor[1]);
            TIM_SetCompare3(TIM2, motor[2]);
            TIM_SetCompare4(TIM2, motor[3]);
        }
        //			else {
        //			motor[0]=motor[1]=motor[2]=motor[3]=1000;
        //					MOTOR_lock();
        //			}
        OSTimeDly(1);  //任务周期1ms
    }
}
int LIM(double value)
{
    if (value > 2000)
        return 2000;
    else if (value < 1000)
        return 1000;
    else
        return (int)value;
}
