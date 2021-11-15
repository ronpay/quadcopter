/**
 * @file  		quad_pid.c
 * @brief 		四轴飞行器姿态控制PID代码
 * @author   	Haozhe Tang
 * @date     	2021-11-12
 * @version   	A001
 * @copyright 	Haozhe Tang
 */

#include "quad_pid.h"

extern volatile float Pitch, Roll, Yaw;
volatile float        Pitch_T, Roll_T, Yaw_T;

PID_TYPE Roll_w_PID = {0}, Pitch_w_PID = {0}, Yaw_w_PID = {0};
PID_TYPE Roll_PID = {0}, Pitch_PID = {0}, Yaw_PID = {0};

#define PID_Rate_Err_MAX 180       // ÿ�ֻ����޷�, ����ʵ������޸�
#define PID_Rate_AccuErr_MAX 1200  // �����޷�, ����ʵ������޸�
#define PID_Rate_Output_MAX 2000
#define PID_Angle_Err_MAX 180       // ÿ�ֻ����޷�, ����ʵ������޸�
#define PID_Angle_AccuErr_MAX 1200  // �����޷�, ����ʵ������޸�
#define PID_Angle_Output_MAX 2000

void Gesture_PID_Init(void)
{
    Gain_Type K_pid[3];

    PID_Init(&Roll_w_PID);
    // 注册PID类型函数
    Roll_w_PID.Calculate_Output_Handler = Calculate_Position_PID_Output;
    // 注册更新反馈值函数
    Roll_w_PID.Update_Feedback_Handler = Update_Roll_w_Feedback;
    // 注册更新目标函数
    Roll_w_PID.Update_Target_Handler = Update_Roll_w_Target;
    // 注册增益设置函数
    Roll_w_PID.Set_PID_Arg_Handler = Set_PID_Arg;
    // 设置误差限幅
    Roll_w_PID.Err_Max = PID_Angle_Err_MAX;
    // 设置输出限幅
    Roll_w_PID.Output_Max = PID_Angle_Output_MAX;
    // 设置积分限幅
    Roll_w_PID.Accu_Err_Max = PID_Angle_AccuErr_MAX;
    // 设置增益Kp, Ki, Kd
    K_pid[0] = 0;
    K_pid[1] = 0;
    K_pid[2] = 0;
    Roll_w_PID.Set_PID_Arg_Handler(&Roll_w_PID, K_pid);

    PID_Init(&Pitch_w_PID);
    // 注册PID类型函数
    Pitch_w_PID.Calculate_Output_Handler = Calculate_Position_PID_Output;
    // 注册更新反馈值函数
    Pitch_w_PID.Update_Feedback_Handler = Update_Pitch_w_Feedback;
    // 注册更新目标函数
    Pitch_w_PID.Update_Target_Handler = Update_Pitch_w_Target;
    // 注册增益设置函数
    Pitch_w_PID.Set_PID_Arg_Handler = Set_PID_Arg;
    // 设置误差限幅
    Roll_w_PID.Err_Max = PID_Angle_Err_MAX;
    // 设置输出限幅
    Roll_w_PID.Output_Max = PID_Angle_Output_MAX;
    // 设置积分限幅
    Roll_w_PID.Accu_Err_Max = PID_Angle_AccuErr_MAX;
    // 设置增益Kp, Ki, Kd
    K_pid[0] = 0;
    K_pid[1] = 0;
    K_pid[2] = 0;
    Pitch_w_PID.Set_PID_Arg_Handler(&Pitch_w_PID, K_pid);

    PID_Init(&Yaw_w_PID);
    // 注册PID类型函数
    Yaw_w_PID.Calculate_Output_Handler = Calculate_Position_PID_Output;
    // 注册更新反馈值函数
    Yaw_w_PID.Update_Feedback_Handler = Update_Yaw_w_Feedback;
    // 注册更新目标函数
    Yaw_w_PID.Update_Target_Handler = Update_Yaw_w_Target;
    // 注册增益设置函数
    Yaw_w_PID.Set_PID_Arg_Handler = Set_PID_Arg;
    // 设置误差限幅
    Roll_w_PID.Err_Max = PID_Angle_Err_MAX;
    // 设置输出限幅
    Roll_w_PID.Output_Max = PID_Angle_Output_MAX;
    // 设置积分限幅
    Roll_w_PID.Accu_Err_Max = PID_Angle_AccuErr_MAX;
    // 设置增益Kp, Ki, Kd
    K_pid[0] = 0;
    K_pid[1] = 0;
    K_pid[2] = 0;
    Yaw_w_PID.Set_PID_Arg_Handler(&Yaw_w_PID, K_pid);

    PID_Init(&Roll_PID);
    // 注册PID类型函数
    Roll_PID.Calculate_Output_Handler = Calculate_Position_PID_Output;
    // 注册更新反馈值函数
    Roll_PID.Update_Feedback_Handler = Update_Roll_Feedback;
    // 注册更新目标函数
    Roll_PID.Update_Target_Handler = Update_Roll_Target;
    // 注册增益设置函数
    Roll_PID.Set_PID_Arg_Handler = Set_PID_Arg;
    // 设置误差限幅
    Roll_PID.Err_Max = PID_Rate_Err_MAX;
    // 设置输出限幅
    Roll_PID.Output_Max = PID_Angle_Output_MAX;
    // 设置积分限幅
    Roll_PID.Accu_Err_Max = PID_Angle_AccuErr_MAX;
    // 设置增益Kp, Ki, Kd
    K_pid[0] = 0;
    K_pid[1] = 0;
    K_pid[2] = 0;
    Roll_PID.Set_PID_Arg_Handler(&Roll_PID, K_pid);

    PID_Init(&Pitch_PID);
    // 注册PID类型函数
    Pitch_PID.Calculate_Output_Handler = Calculate_Position_PID_Output;
    // 注册更新反馈值函数
    Pitch_PID.Update_Feedback_Handler = Update_Pitch_Feedback;
    // 注册更新目标函数
    Pitch_PID.Update_Target_Handler = Update_Pitch_Target;
    // 注册增益设置函数
    Pitch_PID.Set_PID_Arg_Handler = Set_PID_Arg;
    // 设置误差限幅
    Roll_PID.Err_Max = PID_Rate_Err_MAX;
    // 设置输出限幅
    Roll_PID.Output_Max = PID_Angle_Output_MAX;
    // 设置积分限幅
    Roll_PID.Accu_Err_Max = PID_Angle_AccuErr_MAX;
    // 设置增益Kp, Ki, Kd
    K_pid[0] = 0;
    K_pid[1] = 0;
    K_pid[2] = 0;
    Pitch_PID.Set_PID_Arg_Handler(&Pitch_PID, K_pid);

    PID_Init(&Yaw_PID);
    // 注册PID类型函数
    Yaw_PID.Calculate_Output_Handler = Calculate_Position_PID_Output;
    // 注册更新反馈值函数
    Yaw_PID.Update_Feedback_Handler = Update_Yaw_Feedback;
    // 注册更新目标函数
    Yaw_PID.Update_Target_Handler = Update_Yaw_Target;
    // 注册增益设置函数
    Yaw_PID.Set_PID_Arg_Handler = Set_PID_Arg;
    // 设置误差限幅
    Roll_PID.Err_Max = PID_Rate_Err_MAX;
    // 设置输出限幅
    Roll_PID.Output_Max = PID_Angle_Output_MAX;
    // 设置积分限幅
    Roll_PID.Accu_Err_Max = PID_Angle_AccuErr_MAX;
    // 设置增益Kp, Ki, Kd
    K_pid[0] = 0;
    K_pid[1] = 0;
    K_pid[2] = 0;
    Yaw_PID.Set_PID_Arg_Handler(&Yaw_PID, K_pid);
}

// 内环PID目标值来自外环输出, 单环调试时来自上位机
void Update_Roll_w_Target(p_PID_TYPE PID)
{
    PID->Target = Roll_PID.Output;
}

void Update_Pitch_w_Target(p_PID_TYPE PID)
{
    PID->Target = Pitch_PID.Output;
}

void Update_Yaw_w_Target(p_PID_TYPE PID)
{
    PID->Target = Yaw_PID.Output;
}

// Roll_T, Pitch_T, Yaw_T是外环目标值, 来自遥控器
void Update_Roll_Target(p_PID_TYPE PID)
{
    PID->Target = Roll_T;
}

void Update_Pitch_Target(p_PID_TYPE PID)
{
    PID->Target = Pitch_T;
}

void Update_Yaw_Target(p_PID_TYPE PID)
{
    PID->Target = Yaw_T;
}

//! 好像没有用
// 角速度计的值, 来自GY86的数据
void Update_Roll_w_Feedback(p_PID_TYPE PID)
{
    // PID->Feedback = Gyro.wx;
}

void Update_Pitch_w_Feedback(p_PID_TYPE PID)
{
    // PID->Feedback = Gyro.wy;
}

void Update_Yaw_w_Feedback(p_PID_TYPE PID)
{
    // PID->Feedback = Gyro.wz;
}

// Roll, Pitch, Yaw角来自姿态解算四元数转欧拉角后的欧拉角
void Update_Roll_Feedback(p_PID_TYPE PID)
{
    PID->Feedback = Roll;
}

void Update_Pitch_Feedback(p_PID_TYPE PID)
{
    PID->Feedback = Pitch;
}

void Update_Yaw_Feedback(p_PID_TYPE PID)
{
    PID->Feedback = Yaw;
}
