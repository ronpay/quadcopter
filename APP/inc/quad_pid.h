#ifndef _QUAD_PID_H
#define _QUAD_PID_H
/**
 * @file  		quad_pid.h
 * @brief 		四轴飞行器姿态控制PID代码头文件
 * @author   	Haozhe Tang
 * @date     	2021-11-12
 * @version   	A001
 * @copyright 	Haozhe Tang
 */

#include "pid.h"

void Gesture_PID_Init(void);

// 内环PID, 以外环PID的输出值作为目标
void Update_Roll_w_Target(p_PID_TYPE PID);
void Update_Pitch_w_Target(p_PID_TYPE PID);
void Update_Yaw_w_Target(p_PID_TYPE PID);

// 内环PID, 角速度计的值作为反馈值
void Update_Roll_w_Feedback(p_PID_TYPE PID);
void Update_Pitch_w_Feedback(p_PID_TYPE PID);
void Update_Yaw_w_Feedback(p_PID_TYPE PID);

// 外环PID, 以上位机或遥控器设定的值为目标
void Update_Roll_Target(p_PID_TYPE PID);
void Update_Pitch_Target(p_PID_TYPE PID);
void Update_Yaw_Target(p_PID_TYPE PID);

// 外环PID, 四元数转欧拉角后的欧拉角作为反馈值
void Update_Roll_Feedback(p_PID_TYPE PID);
void Update_Pitch_Feedback(p_PID_TYPE PID);
void Update_Yaw_Feedback(p_PID_TYPE PID);

// // 内环执行任务
// void Quadcopter_Imple_Task(void);
// // 外环遥控任务
// void Quadcopter_Control_Task(void);

#endif
