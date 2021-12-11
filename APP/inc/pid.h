/**
 * @file  		pid.h
 * @brief 		PID封装库头文件
 * @author   	Haozhe Tang
 * @date     	2021-7-22
 * @version   	A001
 * @copyright 	Haozhe Tang
 */

#ifndef __PID_H
#define __PID_H

#include <stdint.h>

typedef float Target_Type;
typedef float Feedback_Type;
typedef float Error_Type;
typedef float Gain_Type;
typedef float Output_Type;

typedef struct PID
{
    Target_Type   Target;
    Feedback_Type Feedback;
    Feedback_Type LastFeedback;
    Feedback_Type PrevFeedback;
    Error_Type    Err;           // �������
    Error_Type    LastErr;       // �������
    Error_Type    PrevErr;       // ���������
    Error_Type    ErrDiff;       // �����������
    Error_Type    ErrAccu;       // ����ۻ�ֵ
    Error_Type    Err_Max;       // һ�ֵ�PID���ֻ���������ֵ
    Error_Type    Accu_Err_Max;  // ���ֻ��ڵ����ֵ, �����޷�
    Gain_Type     Kp;
    Gain_Type     Ki;
    Gain_Type     Kd;
    Output_Type   Delta;       // ����PID���������
    Output_Type   Output;      // λ��PID������PID�����
    Output_Type   Output_Max;  // PID��������ֵ, ����޷�
    void (*Set_PID_Arg_Handler)(struct PID*, Gain_Type*);
    void (*Update_Target_Handler)(struct PID*);
    void (*Update_Feedback_Handler)(struct PID*);
    void (*Update_Err_Handler)(struct PID*);
    void (*Calculate_Output_Handler)(struct PID*);
} PID_TYPE, *p_PID_TYPE;

void PID_Init(p_PID_TYPE PID);
void Set_PID_Arg(p_PID_TYPE PID, Gain_Type* K_pid);

void PID_Cycle(p_PID_TYPE PID);

void Calculate_Position_PID_Output(p_PID_TYPE PID);
void Calculate_Delta_PID_Output(p_PID_TYPE PID);

void Update_Target(p_PID_TYPE PID);
void Update_Feedback(p_PID_TYPE PID);
void Update_Err(p_PID_TYPE PID);

void PID_DEBUG_ANO_Send(Target_Type target, Feedback_Type* real);
int Limit(int target,int min,int max);
#endif /*__PID_H*/
