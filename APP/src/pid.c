#include "pid.h"

void PID_Init(p_PID_TYPE PID)
{
    PID->Set_PID_Arg_Handler = Set_PID_Arg;
    PID->Update_Err_Handler  = Update_Err;
}

void Set_PID_Arg(p_PID_TYPE PID, Gain_Type* K_pid)
{
    PID->Kp = K_pid[0];
    PID->Ki = K_pid[1];
    PID->Kd = K_pid[2];
}

// 计算位置式PID
void Calculate_Position_PID_Output(p_PID_TYPE PID)
{
    PID->Output = PID->Kp * PID->Err + PID->Ki * PID->ErrAccu + PID->Kd * PID->ErrDiff;
    //	printf("PID->Output %d\n", PID->Output);

    if (PID->Output > PID->Output_Max)
        PID->Output = PID->Output_Max;
    else if (PID->Output < (-1) * PID->Output_Max)
        PID->Output = (-1) * PID->Output_Max;
}

// 计算增量式PID
void Calculate_Delta_PID_Output(p_PID_TYPE PID)
{
    PID->Delta = PID->Kp * PID->ErrDiff + PID->Ki * PID->Err + PID->Kd * (PID->Err - 2 * PID->LastErr + PID->PrevErr);
    //	printf("PID->Delta %d\n", PID->Delta);

    PID->Output = PID->Output + PID->Delta;
    //	printf("PID->Output %d\n", PID->Output);
}

void PID_Cycle(p_PID_TYPE PID)
{
    // 更新反馈值
    PID->Update_Feedback_Handler(PID);
    //	printf("PID->Feedback %f\n", PID->Feedback);

    // 更新目标值
    PID->Update_Target_Handler(PID);
    //	printf("PID->Target %f\n", PID->Target);

    // 更新误差值
    PID->Update_Err_Handler(PID);
    //	printf("PID->Err %f\n", PID->Err);

    // 计算输出值
    PID->Calculate_Output_Handler(PID);
    //	printf("PID->Output %d\n", PID->Output);
}

void Update_Err(p_PID_TYPE PID)
{
    PID->PrevErr = PID->LastErr;
    //	printf("PID->PrevErr %f\n", PID->PrevErr);

    PID->LastErr = PID->Err;
    //	printf("PID->LastErr %f\n", PID->LastErr);

    PID->Err = PID->Target - PID->Feedback;
    //	printf("PID->Err %f\n", PID->Err);

    PID->ErrDiff = PID->Err - PID->LastErr;
    //	printf("PID->ErrDiff %f\n", PID->ErrDiff);

    if (PID->Err > PID->Err_Max)
        PID->ErrAccu += PID->Err_Max;
    else if (PID->Err < -1 * PID->Err_Max)
        PID->ErrAccu += -1 * PID->Err_Max;
    else
        PID->ErrAccu += PID->Err;

    // 积分限幅
    if (PID->Err > PID->Accu_Err_Max)
        PID->ErrAccu = PID->Accu_Err_Max;
    else if (PID->Err < -1 * PID->Accu_Err_Max)
        PID->ErrAccu = -1 * PID->Accu_Err_Max;

    //	printf("PID->ErrAccu %f\n", PID->ErrAccu);
}

int Limit(int target, int min, int max)
{
    return target < min ? min : (target > max ? max : target);
}
